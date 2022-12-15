"""
Node to receive GPS location data from an iOS device, process it, and publish to ROS
Adapted from: https://github.com/occamLab/ARKit-Ros-Bridge
"""

import rclpy
from rclpy.node import Node
from .handle_udp import extractUDP
from std_msgs.msg import Float64
from gps_interfaces.msg import CoordinateStamped

class GPSServerNode(Node):
    def __init__(self):
        super().__init__('gps_server')
        
        # Get port from ROS params
        self.declare_parameter('port')
        self.port = self.get_parameter('port').value

        # Declare variables to store GPS coordinates
        self.gps_data = None
        self.gps_vals = None
        self.msg = CoordinateStamped()

        self.ios_clock_offset = 0

        # Create subscriptions and publishers
        self.create_subscription(Float64, '/ios_clock', self.handle_ios_clock, 10)
        self.gps_pub = self.create_publisher(CoordinateStamped, '/gps_coords', 10)

        # Create timer to publish GPS coordinates every 0.1s
        self.create_timer(0.1, self.run)

    def run(self):
        """ Run function for node to publish GPS data """
        self.get_data()
        self.process_gps()
        self.gps_pub.publish(self.msg)

    def handle_ios_clock(self, msg):
        """ Handle difference between iOS clock and ROS clock """
        self.ios_clock_offset = msg.data

    def get_data(self):
        """ Get GPS data from UDP packet sent by app """
        self.gps_data = extractUDP(udp_port=self.port)
        self.gps_vals = self.gps_data.split(b",")
    
    def process_gps(self):
        """ Convert received GPS data to Coordinate custom message type """
        self.gps_coords_stamped = [float(val) for val in self.gps_vals]
        self.msg.timestamp = self.gps_coords_stamped[2] + self.ios_clock_offset
        self.msg.latitude = self.gps_coords_stamped[0]
        self.msg.longitude = self.gps_coords_stamped[1]
def main():
    rclpy.init()
    n = GPSServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
