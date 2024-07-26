#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"


#include "communication/communication_server.hpp"

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor  exe;

//   std::shared_ptr<SensorManager> sensor_node =
//     std::make_shared<SensorManager>("sensor_node");

  std::shared_ptr<CommunicationServer> communication =
    std::make_shared<CommunicationServer>();
//   exe.add_node(sensor_node->get_node_base_interface());
    
  exe.add_node(communication->get_node_base_interface());

    //   // 使用 lambda 表达式启动异步任务
    // auto future = std::async(std::launch::async, [&communication]() {
        communication->init();
    // });

  exe.spin();

  rclcpp::shutdown();

  return 0;
}