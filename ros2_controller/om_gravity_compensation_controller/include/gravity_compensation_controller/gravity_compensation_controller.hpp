// Copyright 2017 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <atomic>
#include <array>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <map>

#include "gravity_compensation_controller/visibility_control.h"

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <controller_interface/controller_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <om_gravity_compensation_controller/gravity_compensation_controller_parameters.hpp>
#include <realtime_tools/realtime_buffer.hpp>


namespace gravity_compensation_controller
{
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  GravityCompensationController();

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  GRAVITY_COMPENSATION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct ExternalWrenchSample
  {
    std::array<double, 6> wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    double received_ros_time_s{0.0};
    double msg_stamp_ros_time_s{0.0};
    bool has_msg_stamp{false};
  };

  std::string formatVector(const std::vector<double> & vec);
  bool initialize_force_feedback_telemetry();
  void close_force_feedback_telemetry();
  std::string resolve_force_feedback_telemetry_path() const;
  void write_force_feedback_telemetry_header();
  void log_force_feedback_telemetry(
    double ros_time_s,
    const ExternalWrenchSample & wrench_sample,
    const std::array<double, 6> & applied_wrench,
    const std::vector<double> & applied_tau,
    const std::array<double, 6> & ee_twist);
  std::array<double, 6> compute_end_effector_twist(const KDL::JntArray & q_dot) const;

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // KDL
  KDL::Tree tree_;
  KDL::JntArray q_ddot_;
  KDL::WrenchMap f_ext_;

  // State
  bool dither_switch_;

  // Interfaces
  std::vector<std::string> state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
  };
  std::vector<std::string> command_interface_types_ = {
    hardware_interface::HW_IF_EFFORT
  };

  // Storing command joint names for interfaces
  std::vector<std::string> command_joint_names_;

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template<typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  std::vector<std::string> joint_names_;
  size_t n_joints_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> previous_velocities_;
  std::vector<double> tmp_positions_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_flag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_sub_;
  realtime_tools::RealtimeBuffer<bool> collision_flag_buffer_;
  realtime_tools::RealtimeBuffer<ExternalWrenchSample> external_wrench_buffer_;
  bool joint_index_initialized_ = false;
  std::vector<int> joint_name_to_index_;
  realtime_tools::RealtimeBuffer<std::vector<double>> follower_joint_positions_buffer_;
  std::atomic<bool> has_follower_data_{false};
  std::atomic<bool> has_external_wrench_data_{false};
  bool external_wrench_segment_valid_{false};
  KDL::Chain external_wrench_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> external_wrench_jac_solver_;
  bool external_wrench_chain_valid_{false};
  mutable std::mutex force_feedback_telemetry_mutex_;
  std::ofstream force_feedback_telemetry_stream_;
  bool force_feedback_telemetry_initialized_{false};
};
}  // namespace gravity_compensation_controller

#endif  // GRAVITY_COMPENSATION_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_
