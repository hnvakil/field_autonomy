import rclpy
from rclpy.node import Node
from .handle_udp import extractUDP

class PoseServerNode(Node):
    def __init__(self):
        super().__init__('iOS_pose')
        self.declare_parameter('port')
        self.port = self.get_parameter('port').value

        self.pose_data = None

        self.create_timer(0.1, self.get_data())
    
    def get_data(self):
        self.pose_data = extractUDP(udp_port=self.port)
        print(self.pose_data)

def main():
    rclpy.init()
    n = PoseServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
