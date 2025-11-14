#include "ocs2_ros2_control/hardware/RidgebackUr5Hardware.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ocs2_ros2_control {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

CallbackReturn RidgebackUr5Hardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("RidgebackUr5Hardware"), "No joints defined in hardware info.");
    return CallbackReturn::ERROR;
  }

  if (auto it = info_.hardware_parameters.find("max_joint_velocity"); it != info_.hardware_parameters.end()) {
    max_joint_velocity_ = std::stod(it->second);
  }
  if (auto it = info_.hardware_parameters.find("max_base_velocity"); it != info_.hardware_parameters.end()) {
    max_base_velocity_ = std::stod(it->second);
  }
  if (auto it = info_.hardware_parameters.find("max_yaw_velocity"); it != info_.hardware_parameters.end()) {
    max_yaw_velocity_ = std::stod(it->second);
  }
  if (auto it = info_.hardware_parameters.find("initial_pose_file"); it != info_.hardware_parameters.end()) {
    initial_pose_file_ = it->second;
  }

  auto parse_double_list = [](const std::string &data) {
    std::vector<double> values;
    std::string copy = data;
    std::replace(copy.begin(), copy.end(), ',', ' ');
    std::stringstream ss(copy);
    double value = 0.0;
    while (ss >> value) {
      values.push_back(value);
    }
    return values;
  };

  if (auto it = info_.hardware_parameters.find("initial_base_pose"); it != info_.hardware_parameters.end()) {
    const auto values = parse_double_list(it->second);
    if (values.size() == 3) {
      initial_base_pose_ = {values[0], values[1], values[2]};
    } else if (!values.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("RidgebackUr5Hardware"),
                  "initial_base_pose expects 3 values, got %zu. Using defaults.", values.size());
    }
  }

  joint_names_.clear();
  for (const auto &joint : info_.joints) {
    if (joint.name == BASE_FORWARD_JOINT || joint.name == BASE_YAW_JOINT || joint.name == BASE_X_JOINT || joint.name == BASE_Y_JOINT) {
      continue;
    }
    joint_names_.push_back(joint.name);
  }

  joint_positions_.assign(joint_names_.size(), 0.0);
  joint_velocities_.assign(joint_names_.size(), 0.0);
  joint_velocity_commands_.assign(joint_names_.size(), 0.0);
  joint_position_commands_.assign(joint_names_.size(), 0.0);
  initial_joint_positions_.assign(joint_names_.size(), 0.0);

  if (auto it = info_.hardware_parameters.find("initial_joint_positions"); it != info_.hardware_parameters.end()) {
    const auto values = parse_double_list(it->second);
    if (values.size() == joint_names_.size()) {
      initial_joint_positions_ = values;
    } else if (!values.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("RidgebackUr5Hardware"),
                  "initial_joint_positions expects %zu values, got %zu. Using defaults.",
                  joint_names_.size(), values.size());
    }
  }

  if (!initial_pose_file_.empty()) {
    loadInitialPoseFromFile(initial_pose_file_);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RidgebackUr5Hardware::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  state_interfaces.emplace_back(BASE_X_JOINT, hardware_interface::HW_IF_POSITION, &base_pose_[0]);
  state_interfaces.emplace_back(BASE_Y_JOINT, hardware_interface::HW_IF_POSITION, &base_pose_[1]);
  state_interfaces.emplace_back(BASE_YAW_JOINT, hardware_interface::HW_IF_POSITION, &base_pose_[2]);
  state_interfaces.emplace_back(BASE_FORWARD_JOINT, hardware_interface::HW_IF_VELOCITY, &base_velocity_state_[0]);
  state_interfaces.emplace_back(BASE_YAW_JOINT, hardware_interface::HW_IF_VELOCITY, &base_velocity_state_[1]);

  for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
    state_interfaces.emplace_back(joint_names_[idx], hardware_interface::HW_IF_POSITION, &joint_positions_[idx]);
    state_interfaces.emplace_back(joint_names_[idx], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[idx]);
  }

  return state_interfaces;
}

std::vector<CommandInterface> RidgebackUr5Hardware::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.emplace_back(BASE_FORWARD_JOINT, hardware_interface::HW_IF_VELOCITY, &base_velocity_command_[0]);
  command_interfaces.emplace_back(BASE_YAW_JOINT, hardware_interface::HW_IF_VELOCITY, &base_velocity_command_[1]);

  for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
    command_interfaces.emplace_back(joint_names_[idx], hardware_interface::HW_IF_POSITION, &joint_position_commands_[idx]);
    command_interfaces.emplace_back(joint_names_[idx], hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[idx]);
  }

  return command_interfaces;
}

CallbackReturn RidgebackUr5Hardware::on_activate(const rclcpp_lifecycle::State &) {
  base_pose_ = initial_base_pose_;
  std::fill(base_velocity_state_.begin(), base_velocity_state_.end(), 0.0);
  std::fill(base_velocity_command_.begin(), base_velocity_command_.end(), 0.0);
  joint_positions_ = initial_joint_positions_;
  std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
  std::fill(joint_velocity_commands_.begin(), joint_velocity_commands_.end(), 0.0);
  joint_position_commands_ = joint_positions_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn RidgebackUr5Hardware::on_deactivate(const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

return_type RidgebackUr5Hardware::read(const rclcpp::Time &, const rclcpp::Duration &period) {
  integrateBase(period);
  integrateArm(period);
  return return_type::OK;
}

return_type RidgebackUr5Hardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // Commands are applied directly via pointers; nothing to do here.
  return return_type::OK;
}

void RidgebackUr5Hardware::integrateBase(const rclcpp::Duration &period) {
  const double dt = period.seconds();
  const double v = std::clamp(base_velocity_command_[0], -max_base_velocity_, max_base_velocity_);
  const double w = std::clamp(base_velocity_command_[1], -max_yaw_velocity_, max_yaw_velocity_);
  base_velocity_state_[0] = v;
  base_velocity_state_[1] = w;

  base_pose_[2] += w * dt;
  base_pose_[2] = std::atan2(std::sin(base_pose_[2]), std::cos(base_pose_[2]));
  base_pose_[0] += std::cos(base_pose_[2]) * v * dt;
  base_pose_[1] += std::sin(base_pose_[2]) * v * dt;
}

void RidgebackUr5Hardware::integrateArm(const rclcpp::Duration &period) {
  const double dt = period.seconds();
  for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
    const double cmd = std::clamp(joint_velocity_commands_[idx], -max_joint_velocity_, max_joint_velocity_);
    joint_velocities_[idx] = cmd;
    joint_positions_[idx] += cmd * dt;
    joint_position_commands_[idx] = joint_positions_[idx];
  }
}

void RidgebackUr5Hardware::loadInitialPoseFromFile(const std::string &path) {
  try {
    const auto node = YAML::LoadFile(path);
    if (node["base_pose"]) {
      const auto base = node["base_pose"];
      if (base.IsSequence() && base.size() == 3) {
        initial_base_pose_ = {base[0].as<double>(), base[1].as<double>(), base[2].as<double>()};
      } else if (base.IsMap()) {
        initial_base_pose_[0] = base["x"].as<double>(initial_base_pose_[0]);
        initial_base_pose_[1] = base["y"].as<double>(initial_base_pose_[1]);
        initial_base_pose_[2] = base["yaw"].as<double>(initial_base_pose_[2]);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("RidgebackUr5Hardware"),
                    "base_pose entry in %s must be sequence of 3 or map.", path.c_str());
      }
    }

    if (node["arm_joints"]) {
      const auto joints = node["arm_joints"];
      if (joints.IsMap()) {
        for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
          const auto &name = joint_names_[idx];
          if (joints[name]) {
            initial_joint_positions_[idx] = joints[name].as<double>();
          }
        }
      } else if (joints.IsSequence() && joints.size() == joint_names_.size()) {
        for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
          initial_joint_positions_[idx] = joints[idx].as<double>();
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("RidgebackUr5Hardware"),
                    "arm_joints entry in %s must be map or sequence of size %zu.",
                    path.c_str(), joint_names_.size());
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("RidgebackUr5Hardware"),
                "Failed to load initial pose file %s: %s", path.c_str(), e.what());
  }
}

} // namespace ocs2_ros2_control

PLUGINLIB_EXPORT_CLASS(ocs2_ros2_control::RidgebackUr5Hardware, hardware_interface::SystemInterface)
