#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

// 一个基础的传感器类，其他具体的传感器类可以继承自这个类
class SensorBase : public rclcpp::Node {
public:
    SensorBase(const std::string& node_name) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "SensorBase initialized: %s", node_name.c_str());
    }

    virtual std::string getData() const = 0;

protected:
    std::string data;
};

