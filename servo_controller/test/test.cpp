#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

class ServoController {
public:
    ServoController(const char* i2cBus, int deviceAddress)
        : i2cBus(i2cBus), deviceAddress(deviceAddress), file(-1) {}

    ~ServoController() {
        if (file >= 0) {
            close(file);
        }
    }

    bool init() {
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

        return true;
    }

    bool setVerticalAngle(uint8_t angle) {
        return sendData(9, angle);
    }

    bool setHorizontalAngle(uint8_t angle) {
        return sendData(16, angle);
    }

private:
    bool sendData(uint8_t command, uint8_t value) {
        uint8_t buffer[2] = {command, value};
        if (write(file, buffer, 2) != 2) {
            std::cerr << "Failed to write to the i2c bus: " << strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    const char* i2cBus;
    int deviceAddress;
    int file;
};

int main() {
    ServoController servo("/dev/i2c-7", 0x2d);

    if (!servo.init()) {
        return 1;
    }

    if (!servo.setVerticalAngle(60)) {
        std::cerr << "Failed to set vertical angle" << std::endl;
    }

    if (!servo.setHorizontalAngle(90)) {
        std::cerr << "Failed to set horizontal angle" << std::endl;
    }

    return 0;
}
