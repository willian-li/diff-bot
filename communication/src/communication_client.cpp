#include "communication_client.hpp"

CommunicationClient::CommunicationClient()
: Node("communication_client"), count_(0)
{
  image_pub_ = this->create_publisher<interfaces::msg::Image>("~/image", 10);  
  map_pub_ = this->create_publisher<interfaces::msg::Map>("~/map", 10);  
  pos_pub_ = this->create_publisher<interfaces::msg::Pos>("~/pos", 10);  
  sensor_pub_ = this->create_publisher<interfaces::msg::Sensor>("~/sensor", 10);  

  maptimer_ = this->create_wall_timer(
      10s, std::bind(&CommunicationClient::maptimer_callback, this));
  sensortimer_ = this->create_wall_timer(
      3s, std::bind(&CommunicationClient::sensortimer_callback, this));
}

void CommunicationClient::maptimer_callback()
{
  auto map_message = interfaces::msg::Map();                                  
  //填充地图                                             
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: 'map' ");   
  map_pub_->publish(map_message);
}

void CommunicationClient::sensortimer_callback()
{
  auto sensor_message = interfaces::msg::Sensor();                                  
  //填充传感器                                      
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: 'sensor' ");   
  sensor_pub_->publish(sensor_message);
}
