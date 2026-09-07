// Copyright 2021 ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gravity_compensation_controller/gravity_compensation_controller.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <controller_interface/helpers.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gravity_compensation_controller
{
namespace
{
constexpr double kExternalWrenchTimeoutS = 0.1;

double now_seconds(const rclcpp::Clock::SharedPtr & clock)
{
  return static_cast<double>(clock->now().nanoseconds()) * 1e-9;
}
}  // namespace

GravityCompensationController::GravityCompensationController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto update_start = std::chrono::steady_clock::now();
  const double ros_time_s = now_seconds(get_node()->get_clock());
  const bool telemetry_enabled =
    params_.enable_controller_telemetry && controller_telemetry_rt_pub_ != nullptr;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < n_joints_; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_measured_, joint_state_interface_[1]);

  // Preserve the raw hardware velocity for telemetry while keeping the existing
  // scaled velocity as the value used by KDL and friction compensation.
  for (size_t i = 0; i < n_joints_; ++i) {
    joint_velocities_[i] =
      joint_velocities_measured_[i] * params_.input_velocity_scaling_factors[i];
  }

  // Calculate acceleration from the scaled velocity using the existing finite difference.
  for (size_t i = 0; i < n_joints_; ++i) {
    joint_accelerations_[i] =
      (joint_velocities_[i] - previous_velocities_[i]) / period.seconds() *
      params_.input_acceleration_scaling_factors[i];
  }

  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
  KDL::JntArray q(tree_.getNrOfJoints());
  KDL::JntArray q_dot(tree_.getNrOfJoints());
  KDL::JntArray q_ddot(tree_.getNrOfJoints());
  KDL::JntArray torques(tree_.getNrOfJoints());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];
    q_ddot(i) = joint_accelerations_[i];
  }

  f_ext_.clear();
  ExternalWrenchSample wrench_sample;
  const bool external_wrench_received = has_external_wrench_data_.load();
  bool external_wrench_applied = false;
  std::array<double, 6> applied_wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  if (params_.enable_external_wrench && external_wrench_segment_valid_ && external_wrench_received) {
    auto wrench_ptr = external_wrench_buffer_.readFromRT();
    if (wrench_ptr) {
      wrench_sample = *wrench_ptr;
      const double wrench_age_s = ros_time_s - wrench_sample.received_ros_time_s;
      if (wrench_age_s >= 0.0 && wrench_age_s <= kExternalWrenchTimeoutS) {
        double fx = wrench_sample.wrench[0] * params_.external_wrench_force_scale;
        double fy = wrench_sample.wrench[1] * params_.external_wrench_force_scale;
        double fz = wrench_sample.wrench[2] * params_.external_wrench_force_scale;
        double tx = wrench_sample.wrench[3] * params_.external_wrench_torque_scale;
        double ty = wrench_sample.wrench[4] * params_.external_wrench_torque_scale;
        double tz = wrench_sample.wrench[5] * params_.external_wrench_torque_scale;

        if (std::abs(fx) < params_.external_wrench_force_deadband) {fx = 0.0;}
        if (std::abs(fy) < params_.external_wrench_force_deadband) {fy = 0.0;}
        if (std::abs(fz) < params_.external_wrench_force_deadband) {fz = 0.0;}
        if (std::abs(tx) < params_.external_wrench_torque_deadband) {tx = 0.0;}
        if (std::abs(ty) < params_.external_wrench_torque_deadband) {ty = 0.0;}
        if (std::abs(tz) < params_.external_wrench_torque_deadband) {tz = 0.0;}

        applied_wrench = {{fx, fy, fz, tx, ty, tz}};
        f_ext_[params_.external_wrench_segment] = KDL::Wrench(
          KDL::Vector(fx, fy, fz), KDL::Vector(tx, ty, tz));
        external_wrench_applied = true;
      }
    }
  }

  // Compute the OMY compensation dynamics without the external wrench. When
  // reflection is active, a second RNE solve measures the exact joint-torque
  // contribution of the existing KDL external-load path. This preserves the
  // current wrench frame/sign convention while keeping reflection independent
  // of the compensation calculation.
  KDL::WrenchMap no_external_wrench;
  idsolver.CartToJnt(q, q_dot, q_ddot, no_external_wrench, torques);

  std::fill(tau_reflect_.begin(), tau_reflect_.end(), 0.0);
  if (external_wrench_applied) {
    KDL::JntArray torques_with_external(tree_.getNrOfJoints());
    idsolver.CartToJnt(q, q_dot, q_ddot, f_ext_, torques_with_external);
    for (size_t i = 0; i < n_joints_; ++i) {
      tau_reflect_[i] = torques_with_external(i) - torques(i);
    }
  }

  if (telemetry_enabled) {
    std::fill(tau_rne_.begin(), tau_rne_.end(), 0.0);
    std::fill(tau_spring_.begin(), tau_spring_.end(), 0.0);
    std::fill(tau_sync_.begin(), tau_sync_.end(), 0.0);
    std::fill(tau_friction_.begin(), tau_friction_.end(), 0.0);
    std::fill(tau_pre_scale_.begin(), tau_pre_scale_.end(), 0.0);
    std::fill(tau_cmd_.begin(), tau_cmd_.end(), 0.0);

    for (size_t i = 0; i < n_joints_; ++i) {
      tau_rne_[i] = torques(i);
    }
  }

  // Optional spring effect on joint 2. Preserve the exact existing contribution.
  if (params_.enable_spring_effect && q(2) < 0.5) {
    const double spring_tau = std::abs(q(2) - 0.5) * 2.5;
    torques(2) += spring_tau;
    if (telemetry_enabled && n_joints_ > 2) {
      tau_spring_[2] = spring_tau;
    }
  }

  // Add leader sync function.
  const double gain_joint_1_to_3 = 6.0;
  const double default_gain = 1.0;
  const bool collision = *collision_flag_buffer_.readFromRT();

  if (collision && has_follower_data_) {
    auto follower_positions_ptr = follower_joint_positions_buffer_.readFromRT();
    if (follower_positions_ptr) {
      for (size_t i = 0; i < n_joints_; ++i) {
        const double error = (*follower_positions_ptr)[i] - joint_positions_[i];
        const double gain = (i <= 2) ? gain_joint_1_to_3 : default_gain;
        const double sync_tau = gain * error;
        torques(i) += sync_tau;
        if (telemetry_enabled) {
          tau_sync_[i] = sync_tau;
        }
      }
    }
  }

  // Apply friction compensation only to the OMY compensation channel. Phase 3
  // also limits the historical per-joint torque scaling to that compensation
  // channel, then adds the reflected joint torque without distortion.
  for (size_t i = 0; i < tree_.getNrOfJoints(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }

    const double torque_before_friction = torques(i);
    double kinetic_friction_scalar = params_.kinetic_friction_scalars[i] *
      (1.0 + std::abs(torques(i) * params_.kinetic_friction_torque_scalars[i]));

    double kinetic_friction_rate = 1.0 -
      (std::abs(q_dot(i)) * 10.0 - params_.friction_compensation_velocity_thresholds[i]);
    if (kinetic_friction_rate < 0.0) {
      kinetic_friction_rate = 0.0;
    }
    kinetic_friction_scalar *= kinetic_friction_rate;

    if (q_dot(i) > 0.0) {
      torques(i) += kinetic_friction_scalar * std::abs(q_dot(i));

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) += params_.unloaded_effort_offsets[i];
      }
    } else if (q_dot(i) < 0.0) {
      torques(i) -= kinetic_friction_scalar * std::abs(q_dot(i));

      if (std::abs(torques(i)) < params_.unloaded_effort_thresholds[i]) {
        torques(i) -= params_.unloaded_effort_offsets[i];
      }
    }

    if (std::abs(q_dot(i)) < params_.static_friction_velocity_thresholds[i]) {
      if (dither_switch_) {
        torques(i) += params_.static_friction_scalars[i] * std::abs(torques(i));
      } else {
        torques(i) -= params_.static_friction_scalars[i] * std::abs(torques(i));
      }
    }

    const double compensation_pre_scale = torques(i);
    const double compensation_tau =
      compensation_pre_scale * params_.torque_scaling_factors[i];
    const double applied_tau = compensation_tau + tau_reflect_[i];
    joint_command_interface_[0][i].get().set_value(applied_tau);

    if (telemetry_enabled) {
      tau_friction_[i] = torques(i) - torque_before_friction;
      tau_pre_scale_[i] = compensation_pre_scale;
      tau_cmd_[i] = applied_tau;
    }
  }

  if (telemetry_enabled) {
    ++controller_telemetry_seq_;
    if (controller_telemetry_rt_pub_->trylock()) {
      auto & msg = controller_telemetry_rt_pub_->msg_;
      const double nan_value = std::numeric_limits<double>::quiet_NaN();
      const double receive_age_s = external_wrench_received ?
        ros_time_s - wrench_sample.received_ros_time_s : nan_value;
      const double stamp_age_s =
        external_wrench_received && wrench_sample.has_msg_stamp ?
        ros_time_s - wrench_sample.msg_stamp_ros_time_s : nan_value;

      const auto stamp_ns = time.nanoseconds();
      msg.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
      msg.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
      msg.controller_seq = controller_telemetry_seq_;
      msg.publish_missed_total = controller_telemetry_publish_missed_;
      msg.period_s = period.seconds();
      msg.update_compute_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - update_start).count();
      msg.collision_active = collision;

      msg.external_wrench_received = external_wrench_received;
      msg.external_wrench_fresh = external_wrench_applied;
      msg.external_wrench_has_stamp = external_wrench_received && wrench_sample.has_msg_stamp;
      msg.external_wrench_received_ros_time_s = external_wrench_received ?
        wrench_sample.received_ros_time_s : nan_value;
      msg.external_wrench_msg_stamp_ros_time_s =
        external_wrench_received && wrench_sample.has_msg_stamp ?
        wrench_sample.msg_stamp_ros_time_s : nan_value;
      msg.external_wrench_receive_age_s = receive_age_s;
      msg.external_wrench_stamp_age_s = stamp_age_s;

      for (size_t i = 0; i < 6; ++i) {
        msg.wrench_received[i] = external_wrench_received ? wrench_sample.wrench[i] : 0.0;
        msg.wrench_used[i] = applied_wrench[i];
      }

      for (size_t i = 0; i < n_joints_; ++i) {
        msg.q[i] = joint_positions_[i];
        msg.qdot_measured[i] = joint_velocities_measured_[i];
        msg.qdot_kdl[i] = joint_velocities_[i];
        msg.qddot_kdl[i] = joint_accelerations_[i];
        msg.tau_rne[i] = tau_rne_[i];
        msg.tau_reflect[i] = tau_reflect_[i];
        msg.tau_spring[i] = tau_spring_[i];
        msg.tau_sync[i] = tau_sync_[i];
        msg.tau_friction[i] = tau_friction_[i];
        msg.tau_pre_scale[i] = tau_pre_scale_[i];
        msg.tau_cmd[i] = tau_cmd_[i];
      }

      controller_telemetry_rt_pub_->unlockAndPublish();
    } else {
      ++controller_telemetry_publish_missed_;
    }
  }

  previous_velocities_ = joint_velocities_;
  dither_switch_ = !dither_switch_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  n_joints_ = params_.joints.size();
  joint_names_ = params_.joints;
  collision_flag_buffer_.writeFromNonRT(false);
  joint_positions_.resize(n_joints_);
  joint_velocities_.resize(n_joints_);
  joint_velocities_measured_.resize(n_joints_);
  joint_accelerations_.resize(n_joints_);
  previous_velocities_.resize(n_joints_);
  joint_name_to_index_.resize(joint_names_.size(), -1);
  tmp_positions_.resize(joint_names_.size(), 0.0);
  external_wrench_buffer_.writeFromNonRT(ExternalWrenchSample{});

  tau_rne_.resize(n_joints_);
  tau_reflect_.resize(n_joints_);
  tau_spring_.resize(n_joints_);
  tau_sync_.resize(n_joints_);
  tau_friction_.resize(n_joints_);
  tau_pre_scale_.resize(n_joints_);
  tau_cmd_.resize(n_joints_);

  configure_controller_telemetry();

  follower_joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "JointState message has mismatched name/position sizes");
        return;
      }

      if (!joint_index_initialized_) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
          if (it != msg->name.end()) {
            joint_name_to_index_[i] = static_cast<int>(std::distance(msg->name.begin(), it));
          } else {
            RCLCPP_ERROR(
              get_node()->get_logger(),
              "Joint name '%s' not found in the first joint state message",
              joint_names_[i].c_str());
            return;
          }
        }
        joint_index_initialized_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Joint index mapping initialized.");
      }

      for (size_t i = 0; i < joint_names_.size(); ++i) {
        tmp_positions_[i] = msg->position[joint_name_to_index_[i]];
      }

      follower_joint_positions_buffer_.writeFromNonRT(tmp_positions_);
      has_follower_data_ = true;
    });

  collision_flag_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/collision_flag", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      collision_flag_buffer_.writeFromNonRT(msg->data);
    });

  if (params_.enable_external_wrench) {
    external_wrench_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      params_.external_wrench_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        ExternalWrenchSample sample;
        sample.wrench = {{
          msg->wrench.force.x,
          msg->wrench.force.y,
          msg->wrench.force.z,
          msg->wrench.torque.x,
          msg->wrench.torque.y,
          msg->wrench.torque.z
        }};
        sample.received_ros_time_s = now_seconds(get_node()->get_clock());
        sample.has_msg_stamp =
          (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        if (sample.has_msg_stamp) {
          sample.msg_stamp_ros_time_s =
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        }
        external_wrench_buffer_.writeFromNonRT(sample);
        has_external_wrench_data_ = true;
      });
  }

  if (params_.joints.empty()) {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  std::string robot_description;
  get_node()->get_parameter("robot_description", robot_description);

  const std::string & urdf = robot_description;
  if (!urdf.empty()) {
    if (!kdl_parser::treeFromString(urdf, tree_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description!");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "[KDL] number of joints: %d", tree_.getNrOfJoints());

    for (const auto & segment : tree_.getSegments()) {
      RCLCPP_INFO(get_node()->get_logger(), "[KDL] segment name: %s", segment.first.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Successfully parsed the robot description.");

    q_ddot_.resize(tree_.getNrOfJoints());

    external_wrench_segment_valid_ =
      tree_.getSegments().find(params_.external_wrench_segment) != tree_.getSegments().end();
    if (params_.enable_external_wrench && !external_wrench_segment_valid_) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "external_wrench_segment '%s' is not in the parsed KDL tree.",
        params_.external_wrench_segment.c_str());
      return CallbackReturn::ERROR;
    }

  } else {
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController configured successfully.");
  return CallbackReturn::SUCCESS;
}

void GravityCompensationController::configure_controller_telemetry()
{
  controller_telemetry_rt_pub_.reset();
  controller_telemetry_pub_.reset();
  controller_telemetry_seq_ = 0;
  controller_telemetry_publish_missed_ = 0;

  if (!params_.enable_controller_telemetry) {
    return;
  }

  controller_telemetry_pub_ = get_node()->create_publisher<ControllerTelemetry>(
    params_.controller_telemetry_topic, rclcpp::SensorDataQoS());
  controller_telemetry_rt_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerTelemetry>>(
    controller_telemetry_pub_);

  auto & msg = controller_telemetry_rt_pub_->msg_;
  msg.q.resize(n_joints_);
  msg.qdot_measured.resize(n_joints_);
  msg.qdot_kdl.resize(n_joints_);
  msg.qddot_kdl.resize(n_joints_);
  msg.tau_rne.resize(n_joints_);
  msg.tau_reflect.resize(n_joints_);
  msg.tau_spring.resize(n_joints_);
  msg.tau_sync.resize(n_joints_);
  msg.tau_friction.resize(n_joints_);
  msg.tau_pre_scale.resize(n_joints_);
  msg.tau_cmd.resize(n_joints_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "OMY controller telemetry enabled on '%s'.",
    params_.controller_telemetry_topic.c_str());
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  for (const auto & interface : params_.command_interfaces) {
    auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", n_joints_, interface.c_str(),
        joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces) {
    auto it =
      std::find(state_interface_types_.begin(), state_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(state_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.", n_joints_, interface.c_str(),
        joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController activated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      command_interfaces_[i * command_interface_types_.size() + j].set_value(0.0);
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  dither_switch_ = false;

  tree_ = KDL::Tree();
  f_ext_.clear();
  controller_telemetry_rt_pub_.reset();
  controller_telemetry_pub_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in GravityCompensationController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn GravityCompensationController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down GravityCompensationController.");
  return CallbackReturn::SUCCESS;
}

std::string GravityCompensationController::formatVector(const std::vector<double> & vec)
{
  std::ostringstream oss;
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ", ";
    }
  }
  return oss.str();
}

}  // namespace gravity_compensation_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gravity_compensation_controller::GravityCompensationController,
  controller_interface::ControllerInterface)
