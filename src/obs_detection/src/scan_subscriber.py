import rclpy
from rcly.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_porfile_sensor_data

class Sub_Scan(Node):
	def __init__(self):
		super().__init__('scan_subscriber')
		self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

	def scan_callback(self, msg):
		self.get_logger().info('Received LaserScan message:')
		self.get_logger().info(f' Ranges: {msg.ranges}')


def main(args=None):
	rclpy.init(args=args)
	subscriber = Sub_Scan()

	try:
		rclpy.spin(subscriber)
	except KeyboardInterrupt:
		pass
	finally:
		subscriber.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
