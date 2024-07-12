#include "i2c_sensor.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <i2c/smbus.h>
}
#include <cstring>
#include <rclcpp/rclcpp.hpp>

I2CSensor::I2CSensor(const std::string& node_name, const std::string& i2c_device, int device_address)
    : SensorBase(node_name), i2cDevice(i2c_device), address(device_address), fileDescriptor(-1) {
    // 打开设备
    fileDescriptor = open(i2cDevice.c_str(), O_RDWR);
    if (fileDescriptor < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus");
        return;
    }
    // 指定地址
    if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave.");
        close(fileDescriptor);
        fileDescriptor = -1;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "I2CSensor initialized on %s with address %x", i2c_device.c_str(), device_address);
    sensortimer_ = this->create_wall_timer(
        500ms, std::bind(&I2CSensor::readData, this));
    RCLCPP_INFO(this->get_logger(), "read data...");
}

I2CSensor::~I2CSensor() {
    if (fileDescriptor >= 0) {
        close(fileDescriptor);
    }
}

bool I2CSensor::readData() {
    float results[4] = {0.0};

    for (int channel = 0; channel < 4; ++channel) {
        if (!writeConfig(channel, ADS1115_REG_CONFIG_PGA_6_144V)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write config for channel %d.", channel);
            return false;
        }
        // Wait for conversion to complete
        usleep(100000);  // 100 milliseconds

        // Set pointer to conversion register
        uint8_t reg = ADS1115_REG_POINTER_CONVERT;
        if (write(fileDescriptor, &reg, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set pointer to conversion register for channel %d.", channel);
            return false;
        }

        // Read conversion data
        uint8_t data[2];
        if (read(fileDescriptor, data, 2) != 2) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data for channel %d.", channel);
            return false;
        }
        
        int16_t raw_value = (data[0] << 8) | data[1];
        results[channel] = raw_value * 0.1875;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Channel 0 data: " << results[0] << " mV, Channel 1 data: " << results[1]
                        << " mV, Channel 2 data: " << results[2] << " mV, Channel 3 data: " << results[3] << " mV");

    return true;
}

std::string I2CSensor::getData() const {
    return data;
}

bool I2CSensor::set_single(const uint8_t *config, uint8_t len) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "config " << std::hex << static_cast<int>(config[0]) << static_cast<int>(config[1]));
    uint8_t buf[3];
    buf[0] = ADS1115_REG_POINTER_CONFIG;
    buf[1] = config[0];
    buf[2] = config[1];
    ioctl(fileDescriptor,I2C_RETRIES,5);
    if (write(fileDescriptor, buf, 3) != 3) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write.");
        return false;
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "write successfully");
    return true;
}

bool I2CSensor::writeConfig(int channel, uint16_t mygain) {
    uint8_t configData[2];
    configData[1] = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CQUE_NONE;
    switch (channel) {
        case 0:
            configData[0] = ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_0 | mygain | ADS1115_REG_CONFIG_MODE_CONTIN;
            break;
        case 1:
            configData[0] = ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_1 | mygain | ADS1115_REG_CONFIG_MODE_CONTIN;
            break;
        case 2:
            configData[0] = ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_2 | mygain | ADS1115_REG_CONFIG_MODE_CONTIN;
            break;
        case 3:
            configData[0] = ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_3 | mygain | ADS1115_REG_CONFIG_MODE_CONTIN;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid ADS1115 channel: %d", channel);
            return false;  // Invalid channel
    }
    return set_single(configData, 2);
}