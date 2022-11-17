import rclpy
from rclpy.node import Node
from .handle_udp import extractUDP
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class PoseServerNode(Node):
    def __init__(self):
        super().__init__('iOS_pose')
        self.declare_parameter('port')
        self.port = self.get_parameter('port').value

        self.pose_data = None
        self.pose_vals = None

        self.pose_pub = self.create_publisher(String, 'camera_pose', 10)

        self.create_timer(0.1, self.get_data)

    def get_data(self):
        self.pose_data = extractUDP(udp_port=self.port)
        self.pose_vals = self.pose_data.split(',')
        msg = String()
        msg.data = str(self.pose_data)
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    n = PoseServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
