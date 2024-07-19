#include "sensor_manager.hpp"



SensorManager::SensorManager(const std::string & node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  //设置参数
  this->declare_parameter("serial_device", "/dev/ttyS4");
  this->declare_parameter("baud_rate", 9600);
  this->declare_parameter("i2c_device", "/dev/i2c-3");
  this->declare_parameter("address_st", 0x48);
  this->declare_parameter("address_nd", 0x49);

  RCLCPP_INFO(get_logger(), "SensorManager node has been created.");
}

SensorManager::~SensorManager()
{
  RCLCPP_INFO(get_logger(), "SensorManager node has been destroyed.");
}

void SensorManager::publish()
{
  sensor_publisher_->publish(message);
}

void SensorManager::readi2c_st()
{
  if (i2cst_.readData()) {
      I2CSensor::SensorData data = i2cst_.getSensorData();
      message.light.value = data.data0;
      message.co.value = data.data1;
      message.formaldehyde.value  = data.data2;
      message.smoke.value = data.data3;
  } else {
      std::cerr << "Failed to read data 1 from sensor." << std::endl;
  }

}

void SensorManager::readi2c_nd()
{
  if (i2cnd_.readData()) {
    I2CSensor::SensorData data = i2cnd_.getSensorData();
    message.voltage.value = data.data0;
    message.fire.value = data.data1;
    message.coal_gas.value = data.data2;
  } else {
      std::cerr << "Failed to read data 2 from sensor." << std::endl;
  }
}

void SensorManager::read_air()
{
  if (air_.readData()) {
    AirSensor::SensorData data = air_.getSensorData();
    message.pm1.value = data.PMS1;
    message.pm2.value = data.PMS2_5;
    message.pm10.value = data.PMS10;
    message.temperature.value = data.TPS;
    message.humidity.value = data.HDS;
  } else {
      std::cerr << "Failed to read air from sensor." << std::endl;
  }
}


CallbackReturn_T SensorManager::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SensorManager...");
  // 配置相关的初始化操作
  // 例如，初始化传感器，设置参数等
  serial_device = this->get_parameter("serial_device").as_string();
  baud_rate = this->get_parameter("baud_rate").as_int();
  i2c_device = this->get_parameter("i2c_device").as_string();
  address_st = this->get_parameter("address_st").as_int();
  address_nd = this->get_parameter("address_nd").as_int();


  if(air_.connect(serial_device,baud_rate))
  {
    RCLCPP_INFO(get_logger(), "serial connect  successfully!!");
  }

  if(i2cst_.connect(i2c_device,address_st))
  {
    RCLCPP_INFO(get_logger(), "i2c 1 connect  successfully!!");
  }
  if(i2cnd_.connect(i2c_device,address_nd))
  {
    RCLCPP_INFO(get_logger(), "i2c 2 connect  successfully!!");
  }

  //初始化发布者
  sensor_publisher_ = this->create_publisher<interfaces::msg::Sensor>("sensor_data", 10);
  publish_timer_ = this->create_wall_timer(1s, [this]() {return this->publish();});
  read_i2cst_timer_ = this->create_wall_timer(1s, [this]() {return this->readi2c_st();});
  read_i2cnd_timer_ = this->create_wall_timer(1s, [this]() {return this->readi2c_nd();});
  read_air_timer_ = this->create_wall_timer(1s, [this]() {return this->read_air();});
  publish_timer_->cancel();
  read_i2cst_timer_->cancel();
  read_i2cnd_timer_->cancel();

  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T SensorManager::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating SensorManager...");
  // 激活相关的操作
  // 例如，启动传感器数据采集，启动定时器等
  // air_.readData();
  publish_timer_->reset();  
  read_i2cst_timer_->reset();
  read_i2cnd_timer_->reset();
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T SensorManager::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating SensorManager...");
  // 停用相关的操作
  // 例如，停止传感器数据采集，停止定时器等
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T SensorManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SensorManager...");
  // 清理相关的操作
  // 例如，释放资源，重置变量等
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T SensorManager::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down SensorManager...");
  // 关闭相关的操作
  // 例如，安全地关闭传感器，保存状态等
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T SensorManager::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Error occurred in SensorManager...");
  // 错误处理相关的操作
  // 例如，记录错误，尝试恢复等
  return CallbackReturn_T::SUCCESS;
}

