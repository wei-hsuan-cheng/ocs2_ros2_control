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

find_program(XACRO_EXECUTABLE xacro)
if(NOT XACRO_EXECUTABLE)
  message(FATAL_ERROR "xacro executable not found")
endif()

# Generate a single flattened URDF with ros2_control enabled from the xacro.
set(XACRO_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/ridgeback_ur5.urdf.xacro)
set(GENERATED_URDF_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_urdf)
set(GENERATED_URDF ${GENERATED_URDF_DIR}/ridgeback_ur5.urdf)
set(INITIAL_POSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/ridgeback_ur5/ridgeback_ur5_initial_pose.yaml)

file(MAKE_DIRECTORY ${GENERATED_URDF_DIR})

add_custom_command(
  OUTPUT ${GENERATED_URDF}
  COMMAND ${XACRO_EXECUTABLE} -o ${GENERATED_URDF} ${XACRO_INPUT} ros2_control_mode:=true initial_pose_file:=${INITIAL_POSE_FILE}
  DEPENDS ${XACRO_INPUT}
)

add_custom_target(generate_ridgeback_urdf ALL
  DEPENDS ${GENERATED_URDF}
)

add_dependencies(${PROJECT_NAME} generate_ridgeback_urdf)

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

# Install the generated flattened URDF so that both ros2_control and OCS2
# nodes can use the same robot description file.
install(FILES ${GENERATED_URDF}
  DESTINATION share/${PROJECT_NAME}/description/urdf
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY auto_generated/
  DESTINATION share/${PROJECT_NAME}/auto_generated
)

install(FILES
  ${GENERATED_DEFAULT}
  ${GENERATED_CONTROL}
  DESTINATION share/${PROJECT_NAME}/description/urdf
)

install(FILES ocs2_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
