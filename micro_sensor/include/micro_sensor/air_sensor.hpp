#pragma once

#include "sensor_common.hpp"
#include <libserial/SerialPort.h>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class AirSensor : public SensorBase {
public:
    AirSensor();

    bool connect(const std::string &serial_device, int32_t baud_rate);
    bool disconnect();
    bool readData();

    std::string getData() const override;
    ~AirSensor();
private:
    LibSerial::SerialPort serial_;
    rclcpp::TimerBase::SharedPtr sensortimer_;
    int timeout_ms_;
};