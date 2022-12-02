'''
Ros Node which follows a wall on the left side of the neato
'''
from geometry_msgs.msg import Twist
from direction_interfaces.msg import Direction
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

LINMULT = .5
ANGMULT = .5
MAXSPEED = .75

class CalculateVel(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        self.left_weight = None
        self.right_weight = None 
        self.center_weight = None
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Direction, 'dir_msg', self.get_dir, 10)


    def run_loop(self):
        '''
        Function which gets called repeatedly until node is ended
        Determines direction based on ML trail finding input
        '''
        linear_vel = min(LINMULT * self.center_weight, MAXSPEED)
        angular_vel = min(ANGMULT(self.right_weight - self.left_weight), MAXSPEED)

        linear = Vector3(x=linear_vel,y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=angular_vel)

        #publish velocity
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_dir(self,msg):
        #get direction weights
        self.center_weight = msg.center_weight
        self.left_weight = msg.left_weight
        self.right_weight = msg.right_weight
        


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = CalculateVel()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()