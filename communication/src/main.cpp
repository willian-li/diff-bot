#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"                                
#include "communication_server.hpp"
#include "communication_client.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommunicationClient>());
  rclcpp::shutdown();
  return 0;
}