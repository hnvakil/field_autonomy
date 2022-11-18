import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from .handle_udp import extractUDP
from .helper_functions import TFHelper, convert_matrix_to_frame
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

class PoseServerNode(Node):
    def __init__(self):
        super().__init__('pose_server')
        self.declare_parameter('port')
        self.port = self.get_parameter('port').value

        self.pose_data = None
        self.pose_vals = None
        self.msg = PoseStamped()
        self.msg.header.frame_id = "odom"
        
        self.transform_helper = TFHelper(self)

        self.ios_timestamp = None
        self.ios_clock_valid = False
        self.ios_clock_offset = -1.0
        self.last_timestamp = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, '/device_pose', 10)
        self.clock_pub = self.create_publisher(Float64, '/ios_clock', 10)

        self.create_timer(0.1, self.run)

    def run(self):
        self.get_data()
        self.handle_ios_clock()
        self.process_pose()
        self.pose_pub.publish(self.msg)
        self.transform_helper.send_transform(parent_frame="odom", child_frame="device", pose=self.msg.pose, timestamp=self.msg.header.stamp)

    def get_data(self):
        """Get pose data from UDP packet sent by app"""
        self.pose_data = extractUDP(udp_port=self.port)
        self.pose_vals = self.pose_data.split(b",")
    
    def handle_ios_clock(self):
        self.ios_timestamp = self.pose_vals[16]
        ros_timestamp = self.get_clock().now()
        if not self.ios_clock_valid:
            self.ios_clock_offset = ros_timestamp.nanoseconds/10e9 - float(self.ios_timestamp)
            self.ios_clock_valid = True
        corrected_time = self.ios_clock_offset + float(self.ios_timestamp)
        self.msg.header.stamp = Time(seconds=corrected_time).to_msg()
        clock_offset = Float64(data=self.ios_clock_offset)
        self.clock_pub.publish(clock_offset)
    
    def process_pose(self):
        self.pose_vals = [float(val) for val in self.pose_vals[:16]]
        #Get the transformation matrix from the server and transpose it to row-major order
        self.rotation_matrix = np.matrix([self.pose_vals[0:4], self.pose_vals[4:8], self.pose_vals[8:12], self.pose_vals[12:16]]).T
        #Changing from the iOS coordinate space to the ROS coordinate space.
        change_basis = np.matrix([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        change_basis2 = np.matrix([[0, 0, -1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
        #Left multiplying swaps rows, right multiplying swaps columns
        camera_transform = change_basis*self.rotation_matrix*change_basis2
        camera_transform = camera_transform.A

        #Get the position and orientation from the transformed matrix.
        device_frame = convert_matrix_to_frame(camera_transform)
        trans = device_frame.p
        quat = device_frame.M.GetQuaternion()

        self.msg.pose.position.x = trans.x()
        self.msg.pose.position.y = trans.y()
        self.msg.pose.position.z = trans.z()

        self.msg.pose.orientation.x = quat[0]
        self.msg.pose.orientation.y = quat[1]
        self.msg.pose.orientation.z = quat[2]
        self.msg.pose.orientation.w = quat[3]

def main():
    rclpy.init()
    n = PoseServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
