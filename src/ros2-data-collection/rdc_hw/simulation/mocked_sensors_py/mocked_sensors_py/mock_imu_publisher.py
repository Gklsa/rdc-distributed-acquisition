import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MockImuPublisher(Node):
    def __init__(self):
        super().__init__('mock_imu_publisher')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz
        self.get_logger().info('Mock IMU publisher started at 10 Hz on /imu/data')

    def publish(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated orientation (near-identity quaternion with small noise)
        msg.orientation.x = random.gauss(0.0, 0.001)
        msg.orientation.y = random.gauss(0.0, 0.001)
        msg.orientation.z = random.gauss(0.0, 0.001)
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        # Simulated angular velocity (rad/s)
        msg.angular_velocity.x = random.gauss(0.0, 0.01)
        msg.angular_velocity.y = random.gauss(0.0, 0.01)
        msg.angular_velocity.z = random.gauss(0.0, 0.01)
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        # Simulated linear acceleration (m/s^2) -- Z axis ~9.81 gravity
        msg.linear_acceleration.x = random.gauss(0.0, 0.05)
        msg.linear_acceleration.y = random.gauss(0.0, 0.05)
        msg.linear_acceleration.z = random.gauss(9.81, 0.05)
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
