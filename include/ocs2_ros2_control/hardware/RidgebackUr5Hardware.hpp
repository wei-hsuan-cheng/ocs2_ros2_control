#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>

namespace ocs2_ros2_control {

class RidgebackUr5Hardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RidgebackUr5Hardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void integrateBase(const rclcpp::Duration &period);
  void integrateArm(const rclcpp::Duration &period);
  void loadInitialPoseFromFile(const std::string &path);

  std::vector<std::string> joint_names_;

  // Arm joint state
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_position_commands_;
  std::vector<double> initial_joint_positions_;

  std::string initial_pose_file_;

  double max_joint_velocity_{2.0};
};

} // namespace ocs2_ros2_control
