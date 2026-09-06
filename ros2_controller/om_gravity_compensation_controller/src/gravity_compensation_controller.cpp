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
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/helpers.hpp>

namespace gravity_compensation_controller
{
namespace
{
constexpr const char * kTeleopLogRoot =
  "/home/horowitzlab/project/robosuite_teleop/scripts/hardware_teleop/visualize/logs";
constexpr double kExternalWrenchTimeoutS = 0.1;

double now_seconds(const rclcpp::Clock::SharedPtr & clock)
{
  return static_cast<double>(clock->now().nanoseconds()) * 1e-9;
}

std::string trim_whitespace(std::string value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
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
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double ros_time_s = now_seconds(get_node()->get_clock());
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < n_joints_; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);

  // Apply velocity scaling factors from parameters
  for (size_t i = 0; i < joint_velocities_.size(); i++) {
    joint_velocities_[i] = joint_velocities_[i] * params_.input_velocity_scaling_factors[i];
  }
  // Calculate acceleration from velocity using finite difference
  std::vector<double> joint_accelerations(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) {
    joint_accelerations[i] = (joint_velocities_[i] - previous_velocities_[i]) / period.seconds() *
      params_.input_acceleration_scaling_factors[i];
  }

  // Create KDL objects for computation
  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
  KDL::JntArray q(tree_.getNrOfJoints());
  KDL::JntArray q_dot(tree_.getNrOfJoints());
  KDL::JntArray q_ddot(tree_.getNrOfJoints());
  KDL::JntArray torques(tree_.getNrOfJoints());

  // Populate joint positions, velocities and accelerations from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];
    q_ddot(i) = joint_accelerations[i];
  }

  f_ext_.clear();
  ExternalWrenchSample applied_wrench_sample;
  bool external_wrench_applied = false;
  std::array<double, 6> applied_wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (params_.enable_external_wrench && external_wrench_segment_valid_ && has_external_wrench_data_) {
    auto wrench_ptr = external_wrench_buffer_.readFromRT();
    if (wrench_ptr) {
      applied_wrench_sample = *wrench_ptr;
      const double wrench_age_s = ros_time_s - applied_wrench_sample.received_ros_time_s;
      if (wrench_age_s >= 0.0 && wrench_age_s <= kExternalWrenchTimeoutS) {
        double fx = applied_wrench_sample.wrench[0] * params_.external_wrench_force_scale;
        double fy = applied_wrench_sample.wrench[1] * params_.external_wrench_force_scale;
        double fz = applied_wrench_sample.wrench[2] * params_.external_wrench_force_scale;
        double tx = applied_wrench_sample.wrench[3] * params_.external_wrench_torque_scale;
        double ty = applied_wrench_sample.wrench[4] * params_.external_wrench_torque_scale;
        double tz = applied_wrench_sample.wrench[5] * params_.external_wrench_torque_scale;

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

  // Compute torques
  idsolver.CartToJnt(q, q_dot, q_ddot, f_ext_, torques);

  // Optional spring effect on joint 2
  if (params_.enable_spring_effect) {
    if (q(2) < 0.5) {
      torques(2) += std::abs(q(2) - 0.5) * 2.5;
    }
  }
  // Add leader sync function
  double gain_joint_1_to_3 = 6.0;
  double default_gain = 1.0;
  bool collision = *collision_flag_buffer_.readFromRT();

  if (collision && has_follower_data_) {
    auto follower_positions_ptr = follower_joint_positions_buffer_.readFromRT();
    if (follower_positions_ptr) {
      for (size_t i = 0; i < n_joints_; ++i) {
        double error = (*follower_positions_ptr)[i] - joint_positions_[i];
        double gain = (i <= 2) ? gain_joint_1_to_3 : default_gain;
        torques(i) += gain * error;
      }
    }
  }

  // Apply friction compensation
  std::vector<double> applied_tau(n_joints_, 0.0);
  for (size_t i = 0; i < tree_.getNrOfJoints(); ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }

    double kinetic_friction_scalar = params_.kinetic_friction_scalars[i] *
      (1.0 + std::abs(torques(i) * params_.kinetic_friction_torque_scalars[i]));

    // Kinetic friction compensation
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

    // Static friction compensation (dithering)
    if (std::abs(q_dot(i)) < params_.static_friction_velocity_thresholds[i]) {
      if (dither_switch_) {
        torques(i) += params_.static_friction_scalars[i] * std::abs(torques(i));
      } else {
        torques(i) -= params_.static_friction_scalars[i] * std::abs(torques(i));
      }
    }

    applied_tau[i] = torques(i) * params_.torque_scaling_factors[i];
    joint_command_interface_[0][i].get().set_value(applied_tau[i]);
  }

  if (force_feedback_telemetry_initialized_ && external_wrench_applied) {
    log_force_feedback_telemetry(
      ros_time_s,
      applied_wrench_sample,
      applied_wrench,
      applied_tau,
      compute_end_effector_twist(q_dot));
  }

  // Update previous velocities for next iteration
  previous_velocities_ = joint_velocities_;

  dither_switch_ = !dither_switch_;  // Flip the dither switch

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
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

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // get degrees of freedom
  n_joints_ = params_.joints.size();
  joint_names_ = params_.joints;
  collision_flag_buffer_.writeFromNonRT(false);
  joint_positions_.resize(n_joints_);
  joint_velocities_.resize(n_joints_);
  previous_velocities_.resize(n_joints_);  // Initialize previous velocities vector
  joint_name_to_index_.resize(joint_names_.size(), -1);
  tmp_positions_.resize(joint_names_.size(), 0.0);
  external_wrench_buffer_.writeFromNonRT(ExternalWrenchSample{});
  if (!initialize_force_feedback_telemetry()) {
    return CallbackReturn::ERROR;
  }

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
        if (params_.enable_force_feedback_telemetry && !force_feedback_telemetry_initialized_) {
          initialize_force_feedback_telemetry();
        }
      });
  }

  if (params_.joints.empty()) {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
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

    const std::string base_segment = tree_.getRootSegment()->first;
    external_wrench_chain_valid_ =
      tree_.getChain(base_segment, params_.external_wrench_segment, external_wrench_chain_);
    if (external_wrench_chain_valid_) {
      external_wrench_jac_solver_ =
        std::make_unique<KDL::ChainJntToJacSolver>(external_wrench_chain_);
    } else if (params_.enable_force_feedback_telemetry) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to build KDL chain from '%s' to '%s'; master_ee_twist telemetry will be zeroed.",
        base_segment.c_str(),
        params_.external_wrench_segment.c_str());
    }
  } else {
    // empty URDF is used for some tests
    RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  // order all joints in the storage
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
  close_force_feedback_telemetry();
  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController deactivated successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset flags and parameters
  dither_switch_ = false;

  // Clear KDL tree and joint name map
  tree_ = KDL::Tree();
  external_wrench_chain_ = KDL::Chain();
  external_wrench_jac_solver_.reset();
  external_wrench_chain_valid_ = false;

  // Clear vectors
  f_ext_.clear();
  close_force_feedback_telemetry();

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

bool GravityCompensationController::initialize_force_feedback_telemetry()
{
  close_force_feedback_telemetry();

  if (!params_.enable_force_feedback_telemetry) {
    return true;
  }

  namespace fs = std::filesystem;
  const std::string resolved_path = resolve_force_feedback_telemetry_path();
  if (resolved_path.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Force feedback telemetry enabled, but no path was provided and TELEOP_LOG_DIR is unset. "
      "Master telemetry logging will stay disabled.");
    return true;
  }

  const fs::path telemetry_path(resolved_path);
  std::error_code ec;
  if (telemetry_path.has_parent_path()) {
    fs::create_directories(telemetry_path.parent_path(), ec);
    if (ec) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to create force feedback telemetry directory '%s': %s",
        telemetry_path.parent_path().string().c_str(),
        ec.message().c_str());
      return false;
    }
  }

  const bool file_exists = fs::exists(telemetry_path);
  force_feedback_telemetry_stream_.open(telemetry_path, std::ios::out | std::ios::app);
  if (!force_feedback_telemetry_stream_.is_open()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to open force feedback telemetry file '%s'.",
      telemetry_path.string().c_str());
    return false;
  }

  force_feedback_telemetry_stream_ << std::fixed << std::setprecision(9);

  if (!file_exists) {
    write_force_feedback_telemetry_header();
  }

  force_feedback_telemetry_initialized_ = true;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Master force feedback telemetry logging enabled: %s",
    telemetry_path.string().c_str());
  return true;
}

void GravityCompensationController::close_force_feedback_telemetry()
{
  std::lock_guard<std::mutex> lock(force_feedback_telemetry_mutex_);
  if (force_feedback_telemetry_stream_.is_open()) {
    force_feedback_telemetry_stream_.flush();
    force_feedback_telemetry_stream_.close();
  }
  force_feedback_telemetry_initialized_ = false;
}

void GravityCompensationController::write_force_feedback_telemetry_header()
{
  force_feedback_telemetry_stream_
    << "ros_time_s,source_received_ros_time_s,source_stamp_ros_time_s,wrench_age_ms,"
    << "received_wrench_fx_n,received_wrench_fy_n,received_wrench_fz_n,"
    << "received_wrench_tx_nm,received_wrench_ty_nm,received_wrench_tz_nm,"
    << "applied_wrench_fx_n,applied_wrench_fy_n,applied_wrench_fz_n,"
    << "applied_wrench_tx_nm,applied_wrench_ty_nm,applied_wrench_tz_nm,"
    << "applied_tau_joint1_nm,applied_tau_joint2_nm,applied_tau_joint3_nm,applied_tau_joint4_nm,"
    << "applied_tau_joint5_nm,applied_tau_joint6_nm,"
    << "master_q_joint1_rad,master_q_joint2_rad,master_q_joint3_rad,master_q_joint4_rad,"
    << "master_q_joint5_rad,master_q_joint6_rad,"
    << "master_qdot_joint1_radps,master_qdot_joint2_radps,master_qdot_joint3_radps,"
    << "master_qdot_joint4_radps,master_qdot_joint5_radps,master_qdot_joint6_radps,"
    << "master_ee_twist_vx_mps,master_ee_twist_vy_mps,master_ee_twist_vz_mps,"
    << "master_ee_twist_wx_radps,master_ee_twist_wy_radps,master_ee_twist_wz_radps\n";
}

std::string GravityCompensationController::resolve_force_feedback_telemetry_path() const
{
  namespace fs = std::filesystem;

  if (!params_.force_feedback_telemetry_path.empty()) {
    const fs::path configured_path(params_.force_feedback_telemetry_path);
    if (configured_path.has_extension()) {
      return configured_path.string();
    }
    return (configured_path / "master_force_feedback_telemetry.csv").string();
  }

  const char * teleop_log_dir = std::getenv("TELEOP_LOG_DIR");
  if (teleop_log_dir != nullptr && teleop_log_dir[0] != '\0') {
    return (fs::path(teleop_log_dir) / "master_force_feedback_telemetry.csv").string();
  }

  const fs::path current_run_file = fs::path(kTeleopLogRoot) / ".current_run_dir";
  std::ifstream current_run_stream(current_run_file);
  if (current_run_stream.is_open()) {
    std::ostringstream buffer;
    buffer << current_run_stream.rdbuf();
    const std::string current_run_dir = trim_whitespace(buffer.str());
    if (!current_run_dir.empty()) {
      const fs::path run_dir(current_run_dir);
      if (fs::exists(run_dir) && fs::is_directory(run_dir)) {
        return (run_dir / "master_force_feedback_telemetry.csv").string();
      }
    }
  }

  return {};
}

void GravityCompensationController::log_force_feedback_telemetry(
  double ros_time_s,
  const ExternalWrenchSample & wrench_sample,
  const std::array<double, 6> & applied_wrench,
  const std::vector<double> & applied_tau,
  const std::array<double, 6> & ee_twist)
{
  std::lock_guard<std::mutex> lock(force_feedback_telemetry_mutex_);
  if (!force_feedback_telemetry_stream_.is_open()) {
    return;
  }

  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const double msg_stamp_ros_time_s = wrench_sample.has_msg_stamp ?
    wrench_sample.msg_stamp_ros_time_s : nan_value;
  const double wrench_age_ms = wrench_sample.has_msg_stamp ?
    (ros_time_s - wrench_sample.msg_stamp_ros_time_s) * 1e3 : nan_value;

  force_feedback_telemetry_stream_
    << ros_time_s << ','
    << wrench_sample.received_ros_time_s << ','
    << msg_stamp_ros_time_s << ','
    << wrench_age_ms;

  for (const double value : wrench_sample.wrench) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  for (const double value : applied_wrench) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  for (const double value : applied_tau) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  for (const double value : joint_positions_) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  for (const double value : joint_velocities_) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  for (const double value : ee_twist) {
    force_feedback_telemetry_stream_ << ',' << value;
  }
  force_feedback_telemetry_stream_ << '\n';
}

std::array<double, 6> GravityCompensationController::compute_end_effector_twist(
  const KDL::JntArray & q_dot) const
{
  std::array<double, 6> ee_twist{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (!external_wrench_chain_valid_ || !external_wrench_jac_solver_) {
    return ee_twist;
  }

  KDL::JntArray chain_q(external_wrench_chain_.getNrOfJoints());
  KDL::JntArray chain_q_dot(external_wrench_chain_.getNrOfJoints());
  for (unsigned int i = 0; i < external_wrench_chain_.getNrOfJoints(); ++i) {
    if (i >= joint_positions_.size()) {
      break;
    }
    chain_q(i) = joint_positions_[i];
    chain_q_dot(i) = q_dot(i);
  }

  KDL::Jacobian jacobian(external_wrench_chain_.getNrOfJoints());
  if (external_wrench_jac_solver_->JntToJac(chain_q, jacobian) < 0) {
    return ee_twist;
  }

  for (unsigned int row = 0; row < 6; ++row) {
    double value = 0.0;
    for (unsigned int col = 0; col < external_wrench_chain_.getNrOfJoints(); ++col) {
      value += jacobian(row, col) * chain_q_dot(col);
    }
    ee_twist[row] = value;
  }

  return ee_twist;
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