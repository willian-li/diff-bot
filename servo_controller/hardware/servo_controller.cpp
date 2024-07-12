#include "servo_control/servo_controller.h"

ServoController::ServoController()
  :i2cBus(nullptr), deviceAddress(0), file(-1){}

ServoController::~ServoController() {
    if (file >= 0) {
        close(file);
    }
}

bool ServoController::init(const char* i2cBus, int deviceAddress) {
    this->i2cBus = i2cBus;
    this->deviceAddress = deviceAddress;

    file = open(i2cBus, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open the i2c bus: " << strerror(errno) << std::endl;
        return false;
    }

    if (ioctl(file, I2C_SLAVE, deviceAddress) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave: " << strerror(errno) << std::endl;
        close(file);
        file = -1;
        return false;
    }
    std::cout << "init succeessfully!"<< std::endl;
    return true;
}

bool ServoController::setVerticalAngle(uint8_t angle) {
    return sendData(9, angle);
}

bool ServoController::setHorizontalAngle(uint8_t angle) {
    return sendData(16, angle);
}

bool ServoController::sendData(uint8_t command, uint8_t value) {
    uint8_t buffer[2] = {command, value};
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Failed to write to the i2c bus: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}
