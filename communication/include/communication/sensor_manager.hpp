#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <chrono>
// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"

//传感器
#include "micro_sensor/air_sensor.hpp"
#include "micro_sensor/i2c_sensor.hpp"

#include "interfaces/msg/sensor.hpp"

using CallbackReturn_T = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SensorManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SensorManager(const std::string & node_name, bool intra_process_comms = false);
  ~SensorManager();
  

protected:
  void publish();
  void readi2c_st();
  void readi2c_nd();
  void read_air();
  CallbackReturn_T on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_error(const rclcpp_lifecycle::State &) override;

private:
  // 传感器对象
  AirSensor air_;
  I2CSensor i2cst_;
  I2CSensor i2cnd_;
  
  //数据发布循环
  std::shared_ptr<rclcpp::TimerBase> publish_timer_;

  //数据读取循环
  std::shared_ptr<rclcpp::TimerBase> read_i2cst_timer_;
  std::shared_ptr<rclcpp::TimerBase> read_i2cnd_timer_;
  std::shared_ptr<rclcpp::TimerBase> read_air_timer_;

  //发布话题
  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr sensor_publisher_;
  //发送数据
  interfaces::msg::Sensor message = interfaces::msg::Sensor();

  //通信变量
  std::string serial_device;
  std::string i2c_device;
  int baud_rate;
  int address_st;
  int address_nd;


};



#endif // SENSOR_MANAGER_H
