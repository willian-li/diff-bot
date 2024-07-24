#include "my_robot_actions/action_manager.hpp"
#include <cstdlib>

ActionManager::ActionManager()
: Node("action_manager")
{
    pos_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/pos", 10, std::bind(&ActionManager::pos_callback, this, std::placeholders::_1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void ActionManager::get_current_position()
{
    RCLCPP_INFO(this->get_logger(), "Subscribed to /pos topic to get current position.");
}

void ActionManager::start_navigation()
{
    RCLCPP_INFO(this->get_logger(), "Starting navigation...");
    std::system("ros2 launch my_navigation navigation.launch");
}

void ActionManager::move_forward_one_meter()
{
    RCLCPP_INFO(this->get_logger(), "Moving forward one meter...");
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 1.0;
    cmd_vel_publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel topic to move forward.");
}

void ActionManager::pos_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Current position: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionManager>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
