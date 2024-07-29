#ifndef COMMUNICATION_SERVER_H
#define COMMUNICATION_SERVER_H

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor_config.hpp"       
#include "interfaces/msg/command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "lifecycle_manager.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class CommunicationServer : public rclcpp::Node
{
public:
  CommunicationServer();
  void init();

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

  //改变状态
  // void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client);

  //回调函数
  void sorcfg_callback(const interfaces::msg::SensorConfig & msg) ;
  void command_callback(const interfaces::msg::Command & msg) ;

  //事件处理函数
  void handle_get_current_position() const;
  void handle_start_recognizing_speech() const;
  void handle_face_enrollment() const;
  void handle_simple_move(double distance) const;
  void handle_publish_nav_point(double nav_x,double nav_y) const;
  void handle_start_charging() const;
  void handle_spin_in_place();
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
  
  //本地发送 直接执行
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_publisher_;

  //回调组
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  size_t count_;

  //前进后退的速度
  double speed_ = 0.5;

  //生命周期管理
  std::string lifecycle_node_ = "/sensor_node";
  std::shared_ptr<LifecycleServiceClient> lc_manager_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
};

#endif // COMMUNICATION_SERVER_H
