cmake_minimum_required(VERSION 3.8)
project(micro_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(executable_name micro_sensor_node)
set(library_name ${executable_name}_core)
set(dependencies
  rclcpp
  interfaces
)

find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# 创建一个库
add_library(${library_name}
  src/air_sensor.cpp
  src/i2c_sensor.cpp
)
ament_target_dependencies(${library_name}  ${dependencies})
target_link_libraries(${library_name} serial i2c)

target_include_directories(${library_name} PUBLIC include/micro_sensor)

#执行文件
add_executable(${executable_name} src/main.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name})


install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
