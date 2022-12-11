'''
Ros Node which follows takes direction input from trail finder and outputs twist
'''
from geometry_msgs.msg import Twist
from direction_interfaces.msg import Direction
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3

LINMULT = .025
LINADD = 10
ANGMULT = .025

LINDEF = .2
ANGDEF = .2
MAXSPEED = .5
CENTERTHRESH = .5
LEFTMULT = 1
RIGHTMULT = 1
CENTERMULT = 1

class CalculateVel(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        self.left_weight = None
        self.right_weight = None 
        self.center_weight = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Direction, '/dir_msg', self.get_dir, 10)

        self.center_thresh = CENTERTHRESH
        self.lin_def = LINDEF
        self.ang_def = ANGDEF
        self.left_mult = LEFTMULT

        self.image = np.ndarray(shape=[300,600]) # cv2.imread("cyberkat.jpg", cv2.IMREAD_COLOR)

        cv2.namedWindow("Slider Window")
        cv2.createTrackbar('Center Thresh', 'Slider Window', int(CENTERTHRESH * 100), 100, self.set_center_thresh)
        cv2.setTrackbarMin('Center Thresh', 'Slider Window', -100)
        cv2.setTrackbarMax('Center Thresh', 'Slider Window', 100)
        cv2.createTrackbar('Lin Def', 'Slider Window', int(LINDEF * 100), 100, self.set_lin_def)
        cv2.setTrackbarMin('Lin Def', 'Slider Window', 0)
        cv2.setTrackbarMax('Lin Def', 'Slider Window', 50)
        cv2.createTrackbar('Ang Def', 'Slider Window', int(ANGDEF * 100), 100, self.set_ang_def)
        cv2.setTrackbarMin('Ang Def', 'Slider Window', 0)
        cv2.setTrackbarMax('Ang Def', 'Slider Window', 50)
        cv2.createTrackbar('Left Mult', 'Slider Window', int(LEFTMULT * 100), 100, self.set_left_mult)
        cv2.setTrackbarMin('Left Mult', 'Slider Window', -200)
        cv2.setTrackbarMax('Left Mult', 'Slider Window', 200)

        self.create_timer(0.1, self.run_loop)

    def set_center_thresh(self, val):    
        self.center_thresh = val/100.0
    
    def set_lin_def(self, val):    
        self.lin_def = val/100.0

    def set_ang_def(self, val):    
        self.ang_def = val/100.0

    def set_left_mult(self, val):    
        self.left_mult = val/100.0

    def run_loop(self):
        '''
        Function which gets called repeatedly until node is ended
        Determines direction based on ML trail finding input
        '''
        cv2.waitKey(5)
        cv2.imshow("Slider Window", self.image)
        linear_vel = 0.0
        angular_vel = 0.0
        if not(self.center_weight == None):
            if self.center_weight > self.center_thresh:
                linear_vel = self.lin_def
            elif self.left_weight > self.right_weight:
                angular_vel = self.ang_def
            else:
                angular_vel = -1*self.ang_def

            # angular_vel = min(ANGMULT*(self.right_weight - self.left_weight), MAXSPEED)
            # angular_vel = max(angular_vel, -1*MAXSPEED)
            # # linear_vel = min(LINMULT * (self.center_weight + LINADD), MAXSPEED)
            # linear_vel = min(LINMULT * (self.center_weight + LINADD)/angular_vel, MAXSPEED)
            # linear_vel = max(linear_vel, 0)

        linear = Vector3(x=float(linear_vel),y=0.0,z=0.0)
        angular = Vector3(x=0.0,y=0.0,z=float(angular_vel))

        #publish velocity
        cmd_vel = Twist(linear=linear,angular=angular)
        self.publisher.publish(cmd_vel)

    def get_dir(self,msg):
        #get direction weights
        self.center_weight = msg.center_weight
        self.left_weight = msg.left_weight * self.left_mult
        self.right_weight = msg.right_weight
        


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = CalculateVel()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()