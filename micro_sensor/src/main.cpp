#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"                                
#include "air_sensor.hpp"
#include "i2c_sensor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

    // 创建 I2CSensor 实例，假设 I2C 设备在 "/dev/i2c-1"，地址为 0x48
  auto sensor = std::make_shared<I2CSensor>("i2c_sensor_node", "/dev/i2c-3", 0x48);
  rclcpp::spin(sensor);
  rclcpp::shutdown();
  return 0;
}
