#ifndef COMMUNICATION_CLIENT_HPP
#define COMMUNICATION_CLIENT_HPP

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/image.hpp"   
#include "interfaces/msg/map.hpp"   
#include "interfaces/msg/pos.hpp"   
#include "interfaces/msg/sensor.hpp"   

using namespace std::chrono_literals;

class CommunicationClient : public rclcpp::Node
{
public:
  CommunicationClient();

private:
  void maptimer_callback();
  void sensortimer_callback();

  rclcpp::Publisher<interfaces::msg::Image>::SharedPtr image_pub_;     
  rclcpp::Publisher<interfaces::msg::Map>::SharedPtr map_pub_;     
  rclcpp::Publisher<interfaces::msg::Pos>::SharedPtr pos_pub_;     
  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr sensor_pub_;     

  rclcpp::TimerBase::SharedPtr maptimer_;   
  rclcpp::TimerBase::SharedPtr sensortimer_;           

  size_t count_;
};

#endif // COMMUNICATION_CLIENT_HPP
