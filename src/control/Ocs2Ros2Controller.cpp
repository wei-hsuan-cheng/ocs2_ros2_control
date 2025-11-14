#include "ocs2_ros2_control/control/Ocs2Ros2Controller.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using ocs2::vector_t;
using ocs2::scalar_t;
using ocs2::SystemObservation;
using ocs2::TargetTrajectories;
using ocs2::mobile_manipulator::MobileManipulatorInterface;

namespace ocs2_ros2_control {

namespace {

constexpr char kDefaultTaskFile[] = "";
constexpr char kDefaultLibFolder[] = "";
constexpr char kDefaultUrdfFile[] = "";

} // namespace

Ocs2Ros2Controller::Ocs2Ros2Controller() = default;

controller_interface::CallbackReturn Ocs2Ros2Controller::on_init() {
  arm_joint_names_ = {
      "ur_arm_shoulder_pan_joint",
      "ur_arm_shoulder_lift_joint",
      "ur_arm_elbow_joint",
      "ur_arm_wrist_1_joint",
      "ur_arm_wrist_2_joint",
      "ur_arm_wrist_3_joint",
  };

  std::string default_task = kDefaultTaskFile;
  std::string default_lib = kDefaultLibFolder;
  std::string default_urdf = kDefaultUrdfFile;
  bool local_paths_set = false;
  try {
    const auto local_share = ament_index_cpp::get_package_share_directory("ocs2_ros2_control");
    default_task = local_share + "/config/ridgeback_ur5/task.info";
    default_lib = local_share + "/auto_generated/ridgeback_ur5";
    default_urdf = local_share + "/description/urdf/ridgeback_ur5.urdf";
    local_paths_set = true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_node()->get_logger(), "Unable to locate ocs2_ros2_control share directory: %s", e.what());
  }
  if (!local_paths_set) {
    try {
      const auto manip_share = ament_index_cpp::get_package_share_directory("ocs2_mobile_manipulator");
      default_task = manip_share + "/config/ridgeback_ur5/task.info";
      default_lib = manip_share + "/auto_generated/ridgeback_ur5";
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_node()->get_logger(), "Unable to locate ocs2_mobile_manipulator share directory: %s", e.what());
    }
    try {
      const auto assets_share = ament_index_cpp::get_package_share_directory("ocs2_robotic_assets");
      default_urdf = assets_share + "/resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf";
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_node()->get_logger(), "Unable to locate ocs2_robotic_assets share directory: %s", e.what());
    }
  }

  declareParameterIfNotDeclared<std::string>("task_file", default_task);
  declareParameterIfNotDeclared<std::string>("lib_folder", default_lib);
  declareParameterIfNotDeclared<std::string>("urdf_file", default_urdf);
  declareParameterIfNotDeclared<double>("future_time_offset", future_time_offset_);
  declareParameterIfNotDeclared<double>("command_smoothing_alpha", command_smoothing_alpha_);
  declareParameterIfNotDeclared<std::vector<std::string>>("arm_joints", arm_joint_names_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Ocs2Ros2Controller::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  task_file_ = node->get_parameter("task_file").as_string();
  lib_folder_ = node->get_parameter("lib_folder").as_string();
  urdf_file_ = node->get_parameter("urdf_file").as_string();
  future_time_offset_ = node->get_parameter("future_time_offset").as_double();
  command_smoothing_alpha_ = node->get_parameter("command_smoothing_alpha").as_double();
  arm_joint_names_ = node->get_parameter("arm_joints").as_string_array();
  command_smoothing_alpha_ = std::max(0.0, std::min(1.0, command_smoothing_alpha_));

  RCLCPP_INFO(node->get_logger(), "Controller params: task_file='%s' lib_folder='%s' urdf_file='%s'",
              task_file_.c_str(), lib_folder_.c_str(), urdf_file_.c_str());
  if (task_file_.empty() || lib_folder_.empty() || urdf_file_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Controller parameters are empty; unable to configure.");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    interface_ = std::make_unique<MobileManipulatorInterface>(task_file_, lib_folder_, urdf_file_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    visualization_node_ = std::make_shared<rclcpp::Node>(node->get_name() + std::string("_visualizer"), node->get_namespace());
    visualization_ = std::make_unique<MobileManipulatorVisualization>(visualization_node_, *interface_, task_file_, urdf_file_);
  } catch (const std::exception &e) {
    RCLCPP_WARN(node->get_logger(), "Failed to initialize visualization: %s", e.what());
    visualization_.reset();
    visualization_node_.reset();
  }

  mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>("mobile_manipulator");
  mrt_->initRollout(&interface_->getRollout());

  auto mrt_options = rclcpp::NodeOptions()
                         .allow_undeclared_parameters(true)
                         .automatically_declare_parameters_from_overrides(true);
  const auto bridge_name = node->get_name() + std::string("_mrt_bridge");
  mrt_node_ = std::make_shared<rclcpp::Node>(bridge_name, node->get_namespace(), mrt_options);
  mrt_->launchNodes(mrt_node_);

  const auto &info = interface_->getManipulatorModelInfo();
  state_dim_ = info.stateDim;
  input_dim_ = info.inputDim;

  if (arm_joint_names_.size() != static_cast<size_t>(info.armDim)) {
    RCLCPP_WARN(node->get_logger(),
                "Configured %zu arm joints but model expects %zu. Controller will use the provided list.",
                arm_joint_names_.size(), static_cast<size_t>(info.armDim));
  }

  initial_observation_.state = interface_->getInitialState();
  initial_observation_.input = vector_t::Zero(input_dim_);
  initial_observation_.time = 0.0;
  initial_observation_.mode = 0;
  initial_target_ = computeInitialTarget();
  last_command_ = vector_t::Zero(input_dim_);

  mpc_reset_done_ = false;
  handles_initialized_ = false;

  RCLCPP_INFO(node->get_logger(), "Configured OCS2 controller (stateDim=%zu, inputDim=%zu).", state_dim_, input_dim_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Ocs2Ros2Controller::on_activate(const rclcpp_lifecycle::State &) {
  if (!initializeHandles()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize hardware handles.");
    return controller_interface::CallbackReturn::ERROR;
  }

  mrt_->reset();
  resetMpc();
  handles_initialized_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Ocs2Ros2Controller::on_deactivate(const rclcpp_lifecycle::State &) {
  handles_initialized_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Ocs2Ros2Controller::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(std::string("base_forward_joint/") + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(std::string("base_yaw_joint/") + hardware_interface::HW_IF_VELOCITY);
  for (const auto &joint : arm_joint_names_) {
    config.names.push_back(joint + std::string("/") + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration Ocs2Ros2Controller::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(std::string("base_x_joint/") + hardware_interface::HW_IF_POSITION);
  config.names.push_back(std::string("base_y_joint/") + hardware_interface::HW_IF_POSITION);
  config.names.push_back(std::string("base_yaw_joint/") + hardware_interface::HW_IF_POSITION);
  config.names.push_back(std::string("base_forward_joint/") + hardware_interface::HW_IF_VELOCITY);
  config.names.push_back(std::string("base_yaw_joint/") + hardware_interface::HW_IF_VELOCITY);

  for (const auto &joint : arm_joint_names_) {
    config.names.push_back(joint + std::string("/") + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + std::string("/") + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

void Ocs2Ros2Controller::resetMpc() {
  if (mpc_reset_done_) {
    return;
  }
  try {
    mrt_->resetMpcNode(initial_target_);
    mpc_reset_done_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Requested MPC reset with initial target.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset MPC node: %s", e.what());
  }
}

SystemObservation Ocs2Ros2Controller::buildObservation(const rclcpp::Time &time) const {
  SystemObservation obs;
  obs.time = time.seconds();
  obs.state = vector_t::Zero(state_dim_);
  obs.input = last_command_;
  obs.mode = 0;

  // Update OCS2 state from current robot feedback state
  // Base (3-DoFs)
  obs.state(0) = base_x_state_ ? base_x_state_->get_value() : 0.0;
  obs.state(1) = base_y_state_ ? base_y_state_->get_value() : 0.0;
  obs.state(2) = base_yaw_state_ ? base_yaw_state_->get_value() : 0.0;
  // Arm (n-DoFs)
  for (size_t idx = 0; idx < arm_joint_names_.size(); ++idx) {
    obs.state(3 + idx) = joint_position_states_[idx]->get_value();
  }

  return obs;
}

void Ocs2Ros2Controller::applyCommand(const vector_t &command) {
  if (static_cast<size_t>(command.size()) < 2 + joint_velocity_commands_.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Received command of size %ld, expected at least %zu. Skipping.",
                         command.size(), 2 + joint_velocity_commands_.size());
    return;
  }

  if (base_forward_command_) {
    base_forward_command_->set_value(command(0));
  }
  if (base_yaw_command_) {
    base_yaw_command_->set_value(command(1));
  }

  for (size_t idx = 0; idx < arm_joint_names_.size(); ++idx) {
    if (joint_velocity_commands_[idx]) {
      joint_velocity_commands_[idx]->set_value(command(2 + idx));
    }
  }
}

TargetTrajectories Ocs2Ros2Controller::computeInitialTarget() const {
  vector_t init_target;
  const auto &pin_interface = interface_->getPinocchioInterface();
  const auto &model = pin_interface.getModel();
  auto data = pin_interface.getData();

  pinocchio::forwardKinematics(model, data, initial_observation_.state);
  pinocchio::updateFramePlacements(model, data);

  const auto &info = interface_->getManipulatorModelInfo();
  if (interface_->dual_arm_) {
    init_target.resize(14);
    const auto left_id = model.getFrameId(info.eeFrame);
    const auto &left = data.oMf[left_id];
    Eigen::Quaterniond left_q(left.rotation());
    init_target.segment<3>(0) = left.translation();
    init_target.segment<4>(3) = left_q.coeffs();

    const auto right_id = model.getFrameId(info.eeFrame1);
    const auto &right = data.oMf[right_id];
    Eigen::Quaterniond right_q(right.rotation());
    init_target.segment<3>(7) = right.translation();
    init_target.segment<4>(10) = right_q.coeffs();
  } else {
    init_target.resize(7);
    const auto ee_id = model.getFrameId(info.eeFrame);
    const auto &ee = data.oMf[ee_id];
    Eigen::Quaterniond quat(ee.rotation());
    init_target.head<3>() = ee.translation();
    init_target.tail<4>() = quat.coeffs();
  }

  const vector_t zero_input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
  return TargetTrajectories({initial_observation_.time}, {init_target}, {zero_input});
}

controller_interface::return_type Ocs2Ros2Controller::update(const rclcpp::Time &time, const rclcpp::Duration &) {
  if (!handles_initialized_) {
    return controller_interface::return_type::ERROR;
  }

  auto observation = buildObservation(time);
  mrt_->setCurrentObservation(observation);
  mrt_->spinMRT();
  mrt_->updatePolicy();

  vector_t mpc_state(state_dim_);
  vector_t mpc_input(input_dim_);
  size_t mode = 0;

  if (mrt_->initialPolicyReceived()) {
    mrt_->evaluatePolicy(observation.time + future_time_offset_, observation.state, mpc_state, mpc_input, mode);
    if (command_smoothing_alpha_ < 1.0) {
      mpc_input = command_smoothing_alpha_ * mpc_input + (1.0 - command_smoothing_alpha_) * last_command_;
    }
    applyCommand(mpc_input);
    last_command_ = mpc_input;
  } else {
    // Hold commands at zero until MPC is ready
    vector_t zero = vector_t::Zero(input_dim_);
    applyCommand(zero);
    last_command_ = zero;
  }
  
  if (visualization_ && mrt_->initialPolicyReceived()) {
    visualization_->update(observation.state, mrt_->getPolicy(), mrt_->getCommand());
  }

  return controller_interface::return_type::OK;
}

bool Ocs2Ros2Controller::initializeHandles() {
  auto matches_name = [](const std::string &resource_name, const std::string &expected_joint) {
    if (resource_name == expected_joint) {
      return true;
    }
    const auto slash_idx = resource_name.find('/');
    if (slash_idx == std::string::npos) {
      return false;
    }
    return resource_name.substr(0, slash_idx) == expected_joint;
  };

  auto find_state = [this, &matches_name](const std::string &name, const std::string &interface) -> hardware_interface::LoanedStateInterface * {
    for (auto &state : state_interfaces_) {
      if (matches_name(state.get_name(), name) && state.get_interface_name() == interface) {
        return &state;
      }
    }
    return nullptr;
  };

  auto find_command = [this, &matches_name](const std::string &name, const std::string &interface) -> hardware_interface::LoanedCommandInterface * {
    for (auto &cmd : command_interfaces_) {
      if (matches_name(cmd.get_name(), name) && cmd.get_interface_name() == interface) {
        return &cmd;
      }
    }
    return nullptr;
  };

  base_x_state_ = find_state("base_x_joint", hardware_interface::HW_IF_POSITION);
  base_y_state_ = find_state("base_y_joint", hardware_interface::HW_IF_POSITION);
  base_yaw_state_ = find_state("base_yaw_joint", hardware_interface::HW_IF_POSITION);
  base_forward_velocity_state_ = find_state("base_forward_joint", hardware_interface::HW_IF_VELOCITY);
  base_yaw_velocity_state_ = find_state("base_yaw_joint", hardware_interface::HW_IF_VELOCITY);
  base_forward_command_ = find_command("base_forward_joint", hardware_interface::HW_IF_VELOCITY);
  base_yaw_command_ = find_command("base_yaw_joint", hardware_interface::HW_IF_VELOCITY);

  joint_position_states_.clear();
  joint_velocity_commands_.clear();

  bool ok = base_x_state_ && base_y_state_ && base_yaw_state_ && base_forward_command_ && base_yaw_command_;

  if (!ok) {
    RCLCPP_ERROR(get_node()->get_logger(), "Base interfaces missing. Available state interfaces:");
    for (auto &state : state_interfaces_) {
      RCLCPP_ERROR(get_node()->get_logger(), "  %s/%s", state.get_name().c_str(), state.get_interface_name().c_str());
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Available command interfaces:");
    for (auto &cmd : command_interfaces_) {
      RCLCPP_ERROR(get_node()->get_logger(), "  %s/%s", cmd.get_name().c_str(), cmd.get_interface_name().c_str());
    }
  }

  for (const auto &joint : arm_joint_names_) {
    auto *pos = find_state(joint, hardware_interface::HW_IF_POSITION);
    auto *vel = find_state(joint, hardware_interface::HW_IF_VELOCITY);
    auto *cmd = find_command(joint, hardware_interface::HW_IF_VELOCITY);
    if (!pos || !vel || !cmd) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interfaces for joint '%s'.", joint.c_str());
      for (auto &state : state_interfaces_) {
        RCLCPP_ERROR(get_node()->get_logger(), "  state %s/%s", state.get_name().c_str(), state.get_interface_name().c_str());
      }
      ok = false;
      continue;
    }
    (void)vel;
    joint_position_states_.push_back(pos);
    joint_velocity_commands_.push_back(cmd);
  }

  return ok;
}

} // namespace ocs2_ros2_control

PLUGINLIB_EXPORT_CLASS(ocs2_ros2_control::Ocs2Ros2Controller, controller_interface::ControllerInterface)
