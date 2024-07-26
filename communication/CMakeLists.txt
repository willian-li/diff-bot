cmake_minimum_required(VERSION 3.8)
project(communication)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED) 
find_package(micro_sensor REQUIRED) 
find_package(rclcpp_lifecycle REQUIRED) 
find_package(geometry_msgs REQUIRED) 


set(executable_name communication_node)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  interfaces
  micro_sensor
  rclcpp_lifecycle
  geometry_msgs
)




# 创建一个库
add_library(${library_name}
  src/communication_server.cpp
  src/communication_client.cpp
)
ament_target_dependencies(${library_name}  ${dependencies})

target_include_directories(${library_name} PUBLIC include/communication)
target_link_libraries(${library_name} serial)

# 管理库
add_library(manager_core
  src/sensor_manager.cpp
)
ament_target_dependencies(manager_core  ${dependencies})
target_include_directories(manager_core PUBLIC include/communication )
target_link_libraries(manager_core serial)

#执行文件
add_executable(${executable_name} src/main.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} manager_core ${library_name})
target_include_directories(${executable_name} PUBLIC include )

add_executable(test_lif src/test_cl.cpp)
ament_target_dependencies(test_lif ${dependencies})
target_link_libraries(test_lif manager_core ${library_name})
target_include_directories(test_lif PUBLIC include )


install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS test_lif
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
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()
