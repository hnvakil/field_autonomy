import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class OdometryRecorderNode(Node):
    def __init__(self):
        super().__init__('odometry_recorder')

        self.marker_num = 0

        self.odom_file = open("robot_path.csv", "w+")
        self.odom_file.write("x,y,z\n")
        self.odom_file.flush()

        self.create_subscription(PoseStamped, '/device_pose', self.record_point, 10)

        self.odom_marker_pub = self.create_publisher(Marker, '/odom_markers', 10)
    
    def record_point(self, pose):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = pose.header.stamp
        marker.ns = "my_namespace"
        marker.id = self.marker_num
        marker.lifetime = Duration(seconds=30).to_msg()

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_num += 1
        self.odom_marker_pub.publish(marker)

        self.odom_file.write(f"{pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z}\n")
        self.odom_file.flush()

def main():
    rclpy.init()
    n = OdometryRecorderNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()