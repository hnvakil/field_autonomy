from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.duration import Duration
import PyKDL

def convert_matrix_to_frame(transform_matrix):
    trans = PyKDL.Vector(transform_matrix[0,3], transform_matrix[1,3], transform_matrix[2,3])
    rot = PyKDL.Rotation(transform_matrix[0,0], transform_matrix[0,1], transform_matrix[0,2],
                         transform_matrix[1,0], transform_matrix[1,1], transform_matrix[1,2],
                         transform_matrix[2,0], transform_matrix[2,1], transform_matrix[2,2])
    return PyKDL.Frame(rot, trans)

class TFHelper(object):
    """ TFHelper Provides functionality to convert poses between various
        forms, compare angles in a suitable way, and publish needed
        transforms to ROS """
    def __init__(self, node):
        self.logger = node.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.node = node        # hold onto this for logging
        self.transform_tolerance = Duration(seconds=0.08)    # tolerance for mismatch between scan and odom timestamp

    def send_transform(self, parent_frame, child_frame, pose, timestamp):
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)