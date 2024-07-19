#include "micro_sensor/i2c_sensor.hpp"

I2CSensor::I2CSensor(){
    fileDescriptor = -1;
    std::cout << "i2c class"<< std::endl;
}


I2CSensor::~I2CSensor() {
    if (fileDescriptor >= 0) {
        close(fileDescriptor);
    }
}


bool I2CSensor::connect(const std::string& i2c_device, int device_address)
{
    // 打开设备
    fileDescriptor = open(i2c_device.c_str(), O_RDWR);
    if (fileDescriptor < 0) {
        return false;
    }
    // 指定地址
    if (ioctl(fileDescriptor, I2C_SLAVE, device_address) < 0) {
        close(fileDescriptor);
        fileDescriptor = -1;
        return false;
    }
    return true;
}


bool I2CSensor::readData() {
    double results[4] = {0.0};

    for (int channel = 0; channel < 4; ++channel) {
        if (!writeConfig(channel, ADS1115_REG_CONFIG_PGA_6_144V)) {
            return false;
        }
        // Wait for conversion to complete
        usleep(100000);  // 100 milliseconds

        // Set pointer to conversion register
        uint8_t reg = ADS1115_REG_POINTER_CONVERT;
        if (write(fileDescriptor, &reg, 1) != 1) {
            return false;
        }

        // Read conversion data
        uint8_t data[2];
        if (read(fileDescriptor, data, 2) != 2) {
            return false;
        }
        
        int16_t raw_value = (data[0] << 8) | data[1];
        results[channel] = raw_value * 0.1875;
    }
    sensor_data_.data0 = results[0];
    sensor_data_.data1 = results[1];
    sensor_data_.data2 = results[2];
    sensor_data_.data3 = results[3];
    return true;
}

I2CSensor::SensorData I2CSensor::getSensorData() const {
    return sensor_data_;
}


bool I2CSensor::set_single(const uint8_t *config) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "config " << std::hex << static_cast<int>(config[0]) << static_cast<int>(config[1]));
    uint8_t buf[3];
    buf[0] = ADS1115_REG_POINTER_CONFIG;
    buf[1] = config[0];
    buf[2] = config[1];
    ioctl(fileDescriptor,I2C_RETRIES,5);
    if (write(fileDescriptor, buf, 3) != 3) {
        return false;
    }
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
            return false;  // Invalid channel
    }
    return set_single(configData);
}