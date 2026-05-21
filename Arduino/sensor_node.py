import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUDRATE = int(os.environ.get("SERIAL_BAUDRATE", "115200"))
SERIAL_TIMEOUT = float(os.environ.get("SERIAL_TIMEOUT", "1"))

SENSOR_TOPIC = os.environ.get("SENSOR_TOPIC", "/sensor_data")
READ_INTERVAL_SEC = float(os.environ.get("READ_INTERVAL_SEC", "0.1"))


class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")

        self.publisher_ = self.create_publisher(String, SENSOR_TOPIC, 10)
        self.serial_port = serial.Serial(
            SERIAL_PORT,
            SERIAL_BAUDRATE,
            timeout=SERIAL_TIMEOUT,
        )
        self.timer = self.create_timer(READ_INTERVAL_SEC, self.read_serial)

        self.get_logger().info(
            f"[sensor_node] started: port={SERIAL_PORT}, "
            f"baudrate={SERIAL_BAUDRATE}, topic={SENSOR_TOPIC}"
        )

    def read_serial(self):
        if self.serial_port.in_waiting <= 0:
            return

        try:
            line = self.serial_port.readline().decode("utf-8").strip()
            if not line:
                return

            data = json.loads(line)
            msg = String()
            msg.data = json.dumps(data, ensure_ascii=False)

            self.publisher_.publish(msg)
            self.get_logger().info(f"[sensor_node] publish: {msg.data}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[sensor_node] invalid JSON: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"[sensor_node] decode error: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"[sensor_node] serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"[sensor_node] error: {e}")

    def destroy_node(self):
        if getattr(self, "serial_port", None) and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[sensor_node] stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
