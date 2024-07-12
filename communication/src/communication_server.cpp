#include "communication_server.hpp"

CommunicationServer::CommunicationServer()
: Node("CommunicationServer"), count_(0)
{
  sorcfg_sub_ = this->create_subscription<interfaces::msg::SensorConfig>(
    "sensor_config", 10, std::bind(&CommunicationServer::sorcfg_callback, this, _1));
  command_sub_ = this->create_subscription<interfaces::msg::Command>(
    "command", 10, std::bind(&CommunicationServer::command_callback, this, _1));
}

void CommunicationServer::sorcfg_callback(const interfaces::msg::SensorConfig & msg) const
{
  RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.fire << "'");
}

void CommunicationServer::command_callback(const interfaces::msg::Command & msg) const
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received motion_id: '" << msg.motion_id << "'");
  switch (static_cast<MotionID>(msg.motion_id)) {
    case MotionID::GET_CURRENT_POSITION:
      handle_get_current_position();
      break;
    case MotionID::START_RECOGNIZING_SPEECH:
      handle_start_recognizing_speech();
      break;
    case MotionID::FACE_ENROLLMENT:
      handle_face_enrollment();
      break;
    case MotionID::SIMPLE_MOVE:
      handle_simple_move();
      break;
    case MotionID::PUBLISH_NAV_POINT:
      handle_publish_nav_point();
      break;
    case MotionID::START_CHARGING:
      handle_start_charging();
      break;
    case MotionID::SPIN_IN_PLACE:
      handle_spin_in_place();
      break;
    case MotionID::STOP_SPEAKING_OR_MOVING:
      handle_stop_speaking_or_moving();
      break;
    case MotionID::START_MAPPING:
      handle_start_mapping();
      break;
    case MotionID::STOP_MAPPING:
      handle_stop_mapping();
      break;
    case MotionID::ENABLE_FOLLOW_MODE:
      handle_enable_follow_mode();
      break;
    case MotionID::DISABLE_FOLLOW_MODE:
      handle_disable_follow_mode();
      break;
    case MotionID::ENABLE_GESTURE_RECOGNITION:
      handle_enable_gesture_recognition();
      break;
    case MotionID::DISABLE_GESTURE_RECOGNITION:
      handle_disable_gesture_recognition();
      break;
    default:
      RCLCPP_INFO_STREAM(this->get_logger(), "Unknown motion_id: '" << msg.motion_id << "'");
  }
}

void CommunicationServer::handle_simple_move() const {
  RCLCPP_INFO(this->get_logger(), "Handling simple move.");
}

void CommunicationServer::handle_publish_nav_point() const {
  RCLCPP_INFO(this->get_logger(), "Handling publish navigation point.");
}

void CommunicationServer::handle_start_charging() const {
  RCLCPP_INFO(this->get_logger(), "Handling start charging.");
}

void CommunicationServer::handle_spin_in_place() const {
  RCLCPP_INFO(this->get_logger(), "Handling spin in place.");
}

void CommunicationServer::handle_stop_speaking_or_moving() const {
  RCLCPP_INFO(this->get_logger(), "Handling stop speaking or moving.");
}

void CommunicationServer::handle_start_mapping() const {
  RCLCPP_INFO(this->get_logger(), "Handling start mapping.");
}

void CommunicationServer::handle_stop_mapping() const {
  RCLCPP_INFO(this->get_logger(), "Handling stop mapping.");
}

void CommunicationServer::handle_enable_follow_mode() const {
  RCLCPP_INFO(this->get_logger(), "Handling enable follow mode.");
}

void CommunicationServer::handle_disable_follow_mode() const {
  RCLCPP_INFO(this->get_logger(), "Handling disable follow mode.");
}

void CommunicationServer::handle_enable_gesture_recognition() const {
  RCLCPP_INFO(this->get_logger(), "Handling enable gesture recognition.");
}

void CommunicationServer::handle_disable_gesture_recognition() const {
  RCLCPP_INFO(this->get_logger(), "Handling disable gesture recognition.");
}

void CommunicationServer::handle_get_current_position() const{
  RCLCPP_INFO(this->get_logger(), "Handling get_current_position.");
}
void CommunicationServer::handle_start_recognizing_speech() const{
  RCLCPP_INFO(this->get_logger(), "Handling start_recognizing_speech.");
}
void CommunicationServer::handle_face_enrollment() const{
  RCLCPP_INFO(this->get_logger(), "Handling face_enrollment.");
}