cmake_minimum_required(VERSION 3.8)
project(speech_recognition)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include/speech_recognition
  include/speech_recognition/cxx_api
  include/speech_recognition/c_api
)
link_directories(libs/arm64)

add_executable(${PROJECT_NAME}_node src/keyworlds_recognition_node.cpp src/linuxrec.c src/speech_recognizer.c)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  -lmsc
  -lrt 
  -ldl 
  -lpthread 
  -lasound 
  -lstdc++ 
  -lMicrosoft.CognitiveServices.Speech.core
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS speech_recognition_node
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
