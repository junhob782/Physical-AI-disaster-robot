//
서버에서 
      pip install pyserial
      cd ~/ros2_ws/src
      ros2 pkg create --build-type ament_python sensor_pkg
      이거는 파일 ~/ros2_ws/src/sensor_pkg/sensor_pkg/sensor_node.py

빌드
sudo chmod 666 /dev/ttyACM0
cd ~/ros2_ws
colcon build
source install/setup.bash

실행
ros2 run sensor_pkg sensor_node

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial
import json


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        self.publisher_ = self.create_publisher(
            String,
            '/sensor_data',
            10
        )

        self.serial_port = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=1
        )

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info('Sensor node started')

    def read_serial(self):

        if self.serial_port.in_waiting > 0:

            try:
                line = self.serial_port.readline().decode().strip()

                data = json.loads(line)

                msg = String()
                msg.data = json.dumps(data)

                self.publisher_.publish(msg)

                self.get_logger().info(f'{msg.data}')

            except Exception as e:
                self.get_logger().error(f'Error: {e}')


def main(args=None):

    rclpy.init(args=args)

    node = SensorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
