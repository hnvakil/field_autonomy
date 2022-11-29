import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image
from direction_interfaces.msg import Direction
from trail_model.model_paper import Net
import torch
from PIL import Image
import torchvision.transforms as transforms

model_path = "/home/kat/ros2_ws/src/field_autonomy/trail_model/trained_models/paper_10e.pth"
directions = ["LEFT", "CENTER", "RIGHT"]


class FindTrail(Node):
    """ Find Trail is a ROS node which takes and image and identifies
    if the trail in it is forward, left, or right """

    def __init__(self, image_topic):
        """ Initialize the node """
        super().__init__('find_trail')
        self.PIL_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Direction, 'dir_msg', 10)
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.transforms = transforms.Compose([transforms.Resize((101,101)),
                                     transforms.ToTensor(),
                                     transforms.Normalize(mean=[0.4914, 0.4822, 0.4465],
                                                          std=[0.2023, 0.1994, 0.2010])
                                     ])

        self.model = Net(3)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        thread = Thread(target=self.loop_wrapper)
        thread.start()


    def process_image(self, msg):
        """ Process image messages and store them in
            PIL_image as a for subsequent processing """
        self.PIL_image = Image.fromarray(msg)
        

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def calculate_direction(self):
        """
        Runs model to determine which direction to move
        """
        image_tensor = self.transforms(self.PIL_image).float()
        image_tensor = image_tensor.unsqueeze_(0)
        # input = Variable(image_tensor)
        input = image_tensor.to(self.device)
        output = self.model(input)
        index = output.data.cpu().numpy().argmax()
        return Direction(direction = directions[index])

    def run_loop(self):
        """
        Run loop which determines direction to move in
        """
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.PIL_image is None:
            dir = self.calculate_direction()
            self.pub.publish(dir)

if __name__ == '__main__':
    node = FindTrail("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = FindTrail("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()