#include "micro_sensor/air_sensor.hpp"

AirSensor::AirSensor(){
    std::cout <<"init air sensor"<<std::endl;
}
AirSensor::~AirSensor()
{
    disconnect();
}

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

bool AirSensor::connect(const std::string &serial_device, int32_t baud_rate) {
    // 初始化
    serial_.Open(serial_device);
    serial_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    return true;
}

bool AirSensor::disconnect()
{
    serial_.Close();
    return true;
}

bool AirSensor::readData() {
    std::vector<unsigned char> data_vector;
    serial_.Read(data_vector, 32);

    uint16_t test_data = (static_cast<uint16_t>(data_vector[30]) << 8) + static_cast<uint16_t>(data_vector[31]);
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
        sum += static_cast<uint16_t>(data_vector[i]);
    }

    if (test_data == sum)
    {
        sensor_data_.PMS1 = (static_cast<uint16_t>(data_vector[10]) << 8) + static_cast<uint16_t>(data_vector[11]);
        sensor_data_.PMS2_5 = (static_cast<uint16_t>(data_vector[12]) << 8) + static_cast<uint16_t>(data_vector[13]);
        sensor_data_.PMS10 = (static_cast<uint16_t>(data_vector[14]) << 8) + static_cast<uint16_t>(data_vector[15]);
        sensor_data_.TPS = (static_cast<double>((static_cast<uint16_t>(data_vector[24]) << 8) + static_cast<uint16_t>(data_vector[25])))/10;
        sensor_data_.HDS = (static_cast<double>((static_cast<uint16_t>(data_vector[26]) << 8) + static_cast<uint16_t>(data_vector[27])))/10;
        return true;
    }
    else
    {
        return false;
    }

    
}

AirSensor::SensorData AirSensor::getSensorData() const {
    return sensor_data_;
}
