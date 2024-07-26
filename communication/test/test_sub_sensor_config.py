#!/usr/bin/python
import rclpy
from rclpy.node import Node
from interfaces.msg import SensorConfig

class SensorConfigSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_config_subscriber')
        self.subscription = self.create_subscription(
            SensorConfig,
            '/sensor_config',
            self.listener_callback,
            10)
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg):
        self.get_logger().info(f'Received SensorConfig message:'
                               f'\n  fire: {msg.fire}'
                               f'\n  smoke: {msg.smoke}'
                               f'\n  coal_gas: {msg.coal_gas}'
                               f'\n  co: {msg.co}'
                               f'\n  formaldehyde: {msg.formaldehyde}'
                               f'\n  voltage: {msg.voltage}'
                               f'\n  light: {msg.light}'
                               f'\n  temperature: {msg.temperature}'
                               f'\n  humidity: {msg.humidity}'
                               f'\n  pm1: {msg.pm1}'
                               f'\n  pm2: {msg.pm2}'
                               f'\n  pm10: {msg.pm10}')

def main(args=None):
    rclpy.init(args=args)
    sensor_config_subscriber = SensorConfigSubscriber()
    rclpy.spin(sensor_config_subscriber)
    sensor_config_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
