#pragma once

#include "sensor_common.hpp"
#include <string>
using namespace std::chrono_literals;

class I2CSensor : public SensorBase {
public:
    I2CSensor(const std::string& node_name, const std::string& i2c_device, int device_address);
    virtual ~I2CSensor();

    bool readData();
    virtual std::string getData() const override;
    bool set_single(const uint8_t *config,uint8_t  len);
    bool writeConfig(int channel, uint16_t mygain);
    bool sendRecBytes(
    const int addr,
    uint8_t *sbuf, const int slen,
    uint8_t *rbuf, const int rlen);

private:
    // ADS1115 Register Map
    #define ADS1115_REG_POINTER_CONVERT		0x00 
    #define ADS1115_REG_POINTER_CONFIG		0x01 
    #define ADS1115_REG_POINTER_LOWTHRESH	0x02 
    #define ADS1115_REG_POINTER_HITHRESH	0x03 

    // ADS1115 Configuration Register
    #define ADS1115_REG_CONFIG_OS_NOEFFECT	0x00 
    #define ADS1115_REG_CONFIG_OS_SINGLE		0x80 

    #define ADS1115_REG_CONFIG_MUX_DIFF_0_1	0x00 
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_3	0x10 
    #define ADS1115_REG_CONFIG_MUX_DIFF_1_3	0x20 
    #define ADS1115_REG_CONFIG_MUX_DIFF_2_3	0x30 
    #define ADS1115_REG_CONFIG_MUX_SINGLE_0	0x40 
    #define ADS1115_REG_CONFIG_MUX_SINGLE_1	0x50 
    #define ADS1115_REG_CONFIG_MUX_SINGLE_2	0x60 
    #define ADS1115_REG_CONFIG_MUX_SINGLE_3	0x70 

    #define ADS1115_REG_CONFIG_PGA_6_144V	0x00 
    #define ADS1115_REG_CONFIG_PGA_4_096V	0x02 
    #define ADS1115_REG_CONFIG_PGA_2_048V	0x04 
    #define ADS1115_REG_CONFIG_PGA_1_024V	0x06 
    #define ADS1115_REG_CONFIG_PGA_0_512V	0x08 
    #define ADS1115_REG_CONFIG_PGA_0_256V	0x0A

    #define ADS1115_REG_CONFIG_MODE_CONTIN	0x00 
    #define ADS1115_REG_CONFIG_MODE_SINGLE	0x01 

    #define ADS1115_REG_CONFIG_DR_8SPS		0x00 
    #define ADS1115_REG_CONFIG_DR_16SPS		0x20 
    #define ADS1115_REG_CONFIG_DR_32SPS		0x40 
    #define ADS1115_REG_CONFIG_DR_64SPS		0x60 
    #define ADS1115_REG_CONFIG_DR_128SPS	0x80
    #define ADS1115_REG_CONFIG_DR_250SPS	0xA0 
    #define ADS1115_REG_CONFIG_DR_475SPS	0xC0 
    #define ADS1115_REG_CONFIG_DR_860SPS	0xE0 

    #define ADS1115_REG_CONFIG_CMODE_TRAD	0x00 
    #define ADS1115_REG_CONFIG_CMODE_WINDOW	0x10 

    #define ADS1115_REG_CONFIG_CPOL_ACTVLOW	0x00 
    #define ADS1115_REG_CONFIG_CPOL_ACTVHI	0x08 

    #define ADS1115_REG_CONFIG_CLAT_NONLAT	0x00 
    #define ADS1115_REG_CONFIG_CLAT_LATCH	0x04 

    #define ADS1115_REG_CONFIG_CQUE_1CONV	0x00 
    #define ADS1115_REG_CONFIG_CQUE_2CONV	0x01 
    #define ADS1115_REG_CONFIG_CQUE_4CONV	0x02 
    #define ADS1115_REG_CONFIG_CQUE_NONE	0x03 

    std::string i2cDevice;
    int address;
    int fileDescriptor;
    std::string data;
    rclcpp::TimerBase::SharedPtr sensortimer_;
};
