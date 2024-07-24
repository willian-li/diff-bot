#include "my_robot_actions/action_manager.hpp"
#include <cstdlib>

ActionManager::ActionManager()
: Node("action_manager")
{
    pos_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/pos", 10, std::bind(&ActionManager::pos_callback, this, std::placeholders::_1));
    
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);
}

void ActionManager::publish_goal(double x, double y, double z = 0, double qx = 0 , double qy = 0, double qz = 0, double qw = 0)
{
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.header.frame_id = "map";

    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = z;

    goal_msg.pose.orientation.x = qx;
    goal_msg.pose.orientation.y = qy;
    goal_msg.pose.orientation.z = qz;
    goal_msg.pose.orientation.w = qw;

    RCLCPP_INFO(this->get_logger(), "Publishing goal: [%.2f, %.2f, %.2f] with orientation [%.2f, %.2f, %.2f, %.2f]", 
                x, y, z, qx, qy, qz, qw);

    goal_publisher_->publish(goal_msg);
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

void ActionManager::move_meter(double meter)
{
    RCLCPP_INFO(this->get_logger(), "Moving ...");
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = meter;
    cmd_vel_publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel topic to move forward.");
}

void ActionManager::rotate_yaw(double yaw)
{
    RCLCPP_INFO(this->get_logger(), "rotating yaw...");
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = yaw;
    cmd_vel_publisher_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel topic to rotate.");
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
