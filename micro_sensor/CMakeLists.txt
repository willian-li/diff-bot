cmake_minimum_required(VERSION 3.8)
project(micro_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(library_name ${PROJECT_NAME}_core)

find_package(ament_cmake REQUIRED)

# 创建一个库
add_library(${library_name}
  src/air_sensor.cpp
  src/i2c_sensor.cpp
)
target_link_libraries(${library_name} serial i2c)
target_include_directories(${library_name} PUBLIC include)


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

# ament_export_targets(HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()
