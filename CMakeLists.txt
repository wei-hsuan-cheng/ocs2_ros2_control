cmake_minimum_required(VERSION 3.14)
project(ocs2_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/control/Ocs2Ros2Controller.cpp
  src/visualization/MobileManipulatorVisualization.cpp
  src/hardware/RidgebackUr5Hardware.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  ocs2_core
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_mobile_manipulator
  tf2_ros
  geometry_msgs
  nav_msgs
  visualization_msgs
  ocs2_self_collision_visualization
  xacro
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

set(OCS2_CONTROL_NODE_DEPENDENCIES
  rclcpp
  ocs2_mobile_manipulator
  ocs2_ros_interfaces
  ocs2_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
)

add_executable(mobile_manipulator_mpc_node src/mpc/MobileManipulatorMpcNode.cpp)
ament_target_dependencies(mobile_manipulator_mpc_node ${OCS2_CONTROL_NODE_DEPENDENCIES} ocs2_core ocs2_mpc)
target_link_libraries(mobile_manipulator_mpc_node yaml-cpp)

add_executable(mobile_manipulator_marker_target src/command/MobileManipulatorMarkerTarget.cpp)
ament_target_dependencies(mobile_manipulator_marker_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_marker_target yaml-cpp)

add_executable(mobile_manipulator_twist_target src/command/MobileManipulatorTwistTarget.cpp)
ament_target_dependencies(mobile_manipulator_twist_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_twist_target yaml-cpp)

add_executable(mobile_manipulator_trajectory_target src/command/MobileManipulatorTrajectoryTarget.cpp)
ament_target_dependencies(mobile_manipulator_trajectory_target ${OCS2_CONTROL_NODE_DEPENDENCIES})
target_link_libraries(mobile_manipulator_trajectory_target yaml-cpp)

pluginlib_export_plugin_description_file(controller_interface ocs2_ros2_control.xml)
pluginlib_export_plugin_description_file(hardware_interface ocs2_ros2_control.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mobile_manipulator_mpc_node
  mobile_manipulator_marker_target
  mobile_manipulator_twist_target
  mobile_manipulator_trajectory_target
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY auto_generated/
  DESTINATION share/${PROJECT_NAME}/auto_generated
)

install(FILES ocs2_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
