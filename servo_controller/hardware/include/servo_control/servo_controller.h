#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

class ServoController {
public:
    ServoController();
    ~ServoController();

    bool init(const char* i2cBus, int deviceAddress);
    bool setVerticalAngle(uint8_t angle);
    bool setHorizontalAngle(uint8_t angle);

private:
    bool sendData(uint8_t command, uint8_t value);

    const char* i2cBus;
    int deviceAddress;
    int file;
};

#endif // SERVO_CONTROLLER_H
