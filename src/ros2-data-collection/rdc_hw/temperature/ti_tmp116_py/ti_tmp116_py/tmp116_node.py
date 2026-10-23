import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

from ti_tmp116_py.tmp116_driver import TMP116Driver


class TMP116Node(Node):
    def __init__(self):
        super().__init__('tmp116_node_py')
        self.declare_parameter('i2c_bus',         1)
        self.declare_parameter('i2c_address',     0x48)
        self.declare_parameter('publish_rate_hz', 1.0)

        bus     = self.get_parameter('i2c_bus').value
        addr    = self.get_parameter('i2c_address').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self._driver = TMP116Driver(bus_id=bus, address=addr)
        self._pub    = self.create_publisher(Temperature, '/temperature', 10)
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(f'TMP116 node started (bus={bus}, addr=0x{addr:02X}, {rate_hz}Hz)')

    def _publish(self):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature  = self._driver.read_temperature()
        msg.variance     = 0.0
        self._pub.publish(msg)
        self.get_logger().info(f'Temp: {msg.temperature:.3f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TMP116Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
