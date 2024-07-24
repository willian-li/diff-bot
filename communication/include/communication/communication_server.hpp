#ifndef COMMUNICATION_SERVER_H
#define COMMUNICATION_SERVER_H

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor_config.hpp"       
#include "interfaces/msg/command.hpp"
#include "geometry_msgs/msg/twist.hpp"


using std::placeholders::_1;

class CommunicationServer : public rclcpp::Node
{
public:
  CommunicationServer();

private:
  enum class MotionID {
    GET_CURRENT_POSITION = 1,
    START_RECOGNIZING_SPEECH = 3,
    FACE_ENROLLMENT = 4,
    SIMPLE_MOVE = 21,
    PUBLISH_NAV_POINT = 22,
    START_CHARGING = 23,
    SPIN_IN_PLACE = 24,
    STOP_SPEAKING_OR_MOVING = 25,
    START_MAPPING = 26,
    STOP_MAPPING = 27,
    ENABLE_FOLLOW_MODE = 28,
    DISABLE_FOLLOW_MODE = 29,
    ENABLE_GESTURE_RECOGNITION = 30,
    DISABLE_GESTURE_RECOGNITION = 31
  };


  void sorcfg_callback(const interfaces::msg::SensorConfig & msg) const;
  void command_callback(const interfaces::msg::Command & msg) const;
  void handle_get_current_position() const;
  void handle_start_recognizing_speech() const;
  void handle_face_enrollment() const;
  void handle_simple_move() const;
  void handle_publish_nav_point() const;
  void handle_start_charging() const;
  void handle_spin_in_place() const;
  void handle_stop_speaking_or_moving() const;
  void handle_start_mapping() const;
  void handle_stop_mapping() const;
  void handle_enable_follow_mode() const;
  void handle_disable_follow_mode() const;
  void handle_enable_gesture_recognition() const;
  void handle_disable_gesture_recognition() const;

  //上位机发送
  rclcpp::Subscription<interfaces::msg::SensorConfig>::SharedPtr sorcfg_sub_;
  rclcpp::Subscription<interfaces::msg::Command>::SharedPtr command_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

  size_t count_;
};

#endif // COMMUNICATION_SERVER_H
