import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_camera_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz
        self.width = 320
        self.height = 240
        self.frame = 0
        self.get_logger().info('Mock Camera publisher started at 10 Hz on /camera/image_raw')

    def publish(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3

        # Solid-color frame that shifts hue each tick; avoids per-pixel Python loops
        r = (self.frame * 3) % 256
        g = (self.frame * 7 + 80) % 256
        b = 150
        msg.data = bytes([r, g, b]) * (self.width * self.height)

        self.pub.publish(msg)
        self.frame += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
