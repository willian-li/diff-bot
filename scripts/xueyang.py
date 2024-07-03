#!/usr/bin/python
import serial
import rospy2 as rospy

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int8


class Publisher:
    def __init__(self):
        rate = rospy.Rate(1)  # 1 Hz
        self.switch_flag = 0
        self.ser = serial.Serial('/dev/ttyS3', 38400, timeout=0.5)
        self.sub = rospy.Subscriber('/pressure_sensor_switch', Int8, self.switch_callback)
        self.pub = rospy.Publisher('/pressure_sensor', Int32MultiArray, queue_size=1)
        if self.ser.is_open:
            print("串口已打开")
            # 打印串口的配置信息
            print(f"串口名称：{self.ser.name}")
            print(f"波特率：{self.ser.baudrate}")
            print(f"数据位：{self.ser.bytesize}")
            print(f"停止位：{self.ser.stopbits}")
            print(f"校验位：{self.ser.parity}")
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                # 读取
                data = self.ser.read(88)
                print(data)
                # 将字节串拆分成每个字节
                byte_array = [data[i:i+1] for i in range(0, len(data), 1)]

                # 存储转换后的数字的数组
                decimal_numbers = []

                # 将每个字节转换为十进制数字并存储在数组中
                for byte in byte_array:
                    num = int.from_bytes(byte, byteorder='big')
                    decimal_numbers.append(num)

                # 打印转换后的数字数组
                if(len(decimal_numbers) == 88): 
                    msg = Int32MultiArray()
                    msg.data = decimal_numbers
                    self.pub.publish(msg)
            rate.sleep()

    def switch_callback(self, msg):
        if (msg.data == 1):
            self.switch_flag = 1
            self.open_detection()
        else:
            self.switch_flag = 0
            self.close_detection()


    def send_command(self, command):
        data_to_send = bytes([command])
        self.ser.write(data_to_send)
        print(f'Sent command: 0x{command:02X}')

    def open_detection(self):
        self.send_command(0x8A)

    def close_detection(self):
        self.send_command(0x88)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("pressure_sensor_publisher")
        rospy.loginfo("Starting pressure_sensor_publisher node")
        sensorpub = Publisher()
        rospy.spin()
    except KeyboardInterrupt:
        sensorpub.ser.close()
        print ("Serial communication closed. Shutting down pressure_sensor_publisher node.")

