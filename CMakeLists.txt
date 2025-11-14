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
find_package(visualization_msgs REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)

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
  visualization_msgs
  ocs2_self_collision_visualization
  xacro
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

find_program(XACRO_EXECUTABLE xacro)
if(NOT XACRO_EXECUTABLE)
  message(FATAL_ERROR "xacro executable not found")
endif()

set(XACRO_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/ridgeback_ur5.urdf.xacro)
set(GENERATED_URDF_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_urdf)
set(GENERATED_DEFAULT ${GENERATED_URDF_DIR}/ridgeback_ur5.urdf)
set(GENERATED_CONTROL ${GENERATED_URDF_DIR}/ridgeback_ur5_ros2_control.urdf)

file(MAKE_DIRECTORY ${GENERATED_URDF_DIR})

add_custom_command(
  OUTPUT ${GENERATED_DEFAULT}
  COMMAND ${XACRO_EXECUTABLE} -o ${GENERATED_DEFAULT} ${XACRO_INPUT} ros2_control_mode:=false
  DEPENDS ${XACRO_INPUT}
)

add_custom_command(
  OUTPUT ${GENERATED_CONTROL}
  COMMAND ${XACRO_EXECUTABLE} -o ${GENERATED_CONTROL} ${XACRO_INPUT} ros2_control_mode:=true
  DEPENDS ${XACRO_INPUT}
)

add_custom_target(generate_ridgeback_urdf ALL
  DEPENDS ${GENERATED_DEFAULT} ${GENERATED_CONTROL}
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

install(FILES
  ${GENERATED_DEFAULT}
  ${GENERATED_CONTROL}
  DESTINATION share/${PROJECT_NAME}/description/urdf
)

install(FILES ocs2_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
