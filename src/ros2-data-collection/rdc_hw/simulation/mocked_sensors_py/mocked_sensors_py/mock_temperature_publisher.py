import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class MockTemperaturePublisher(Node):
    def __init__(self):
        super().__init__('mock_temperature_publisher')
        self.pub = self.create_publisher(Temperature, '/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish)  # 1 Hz
        self.get_logger().info('Mock Temperature publisher started at 1 Hz on /temperature')

    def publish(self):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'
        msg.temperature = random.gauss(25.0, 0.3)  # simulated room temperature in Celsius
        msg.variance = 0.09
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockTemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
