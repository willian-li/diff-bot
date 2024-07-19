#ifndef AIR_SENSOR_HPP
#define AIR_SENSOR_HPP

#include <libserial/SerialPort.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class AirSensor {
public:
    AirSensor();

    bool connect(const std::string &serial_device, int32_t baud_rate);
    bool disconnect();
    bool readData();

    std::string getData() const;
    ~AirSensor();

    struct SensorData {
        uint16_t PMS1;
        uint16_t PMS2_5;
        uint16_t PMS10;
        double TPS;
        double HDS;
    };

    SensorData getSensorData() const; 

private:
    LibSerial::SerialPort serial_;
    int timeout_ms_;

    SensorData sensor_data_;  
};

#endif // AIR_SENSOR_HPP