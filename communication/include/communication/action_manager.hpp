#ifndef MY_ROBOT_ACTIONS__ACTION_MANAGER_HPP_
#define MY_ROBOT_ACTIONS__ACTION_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

class ActionManager : public rclcpp::Node
{
public:
    ActionManager();
    void get_current_position();
    void start_navigation();
    void move_meter(double meter);

    void publish_goal(double x, double y, double z = 0, double qx = 0 , double qy = 0, double qz = 0, double qw = 0);

private:
    void pos_callback(const std_msgs::msg::String::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pos_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
};

#endif  // MY_ROBOT_ACTIONS__ACTION_MANAGER_HPP_
