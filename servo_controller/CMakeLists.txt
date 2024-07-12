cmake_minimum_required(VERSION 3.16)
project(servo_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  servo_control
  SHARED
  hardware/rrbot.cpp
  hardware/servo_controller.cpp
)
target_compile_features(servo_control PUBLIC cxx_std_17)
target_include_directories(servo_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/servo_control>
)
ament_target_dependencies(
  servo_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SERVO_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface servo_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/servo_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/servo_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/servo_control
)
install(TARGETS servo_control
  EXPORT export_servo_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
endif()


## EXPORTS
ament_export_targets(export_servo_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
