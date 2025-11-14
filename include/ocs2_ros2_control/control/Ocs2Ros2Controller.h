#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

#include "ocs2_ros2_control/visualization/MobileManipulatorVisualization.h"

namespace ocs2_ros2_control {

class Ocs2Ros2Controller : public controller_interface::ControllerInterface {
public:
  Ocs2Ros2Controller();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  template <typename T>
  void declareParameterIfNotDeclared(const std::string &name, const T &default_value);

  void resetMpc();
  ocs2::SystemObservation buildObservation(const rclcpp::Time &time) const;
  void applyCommand(const ocs2::vector_t &command);
  bool initializeHandles();
  ocs2::TargetTrajectories computeInitialTarget() const;

  std::vector<std::string> arm_joint_names_;

  // State handles
  hardware_interface::LoanedStateInterface *base_x_state_{nullptr};
  hardware_interface::LoanedStateInterface *base_y_state_{nullptr};
  hardware_interface::LoanedStateInterface *base_yaw_state_{nullptr};
  hardware_interface::LoanedStateInterface *base_forward_velocity_state_{nullptr};
  hardware_interface::LoanedStateInterface *base_yaw_velocity_state_{nullptr};
  std::vector<hardware_interface::LoanedStateInterface *> joint_position_states_;

  // Command handles
  hardware_interface::LoanedCommandInterface *base_forward_command_{nullptr};
  hardware_interface::LoanedCommandInterface *base_yaw_command_{nullptr};
  std::vector<hardware_interface::LoanedCommandInterface *> joint_velocity_commands_;

  // OCS2 components
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;
  rclcpp::Node::SharedPtr mrt_node_;
  ocs2::vector_t last_command_;
  ocs2::SystemObservation initial_observation_;
  ocs2::TargetTrajectories initial_target_;

  // Parameters
  std::string task_file_;
  std::string lib_folder_;
  std::string urdf_file_;
  double future_time_offset_{0.02};
  double command_smoothing_alpha_{1.0};

  size_t state_dim_{0};
  size_t input_dim_{0};
  bool mpc_reset_done_{false};
  bool handles_initialized_{false};

  rclcpp::Node::SharedPtr visualization_node_;
  std::unique_ptr<MobileManipulatorVisualization> visualization_;
};

template <typename T>
void Ocs2Ros2Controller::declareParameterIfNotDeclared(const std::string &name, const T &default_value) {
  auto node = get_node();
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
}

} // namespace ocs2_ros2_control
