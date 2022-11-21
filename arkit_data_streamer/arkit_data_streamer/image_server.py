import rclpy
import numpy as np
import cv2
import socket
import struct
import time
from threading import Thread
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ImageServerNode(Node):
    def __init__(self):
        super().__init__('image_server')
        # self.declare_parameter('port')
        # self.port = self.get_parameter('port').value
        self.port = 35602

        self.ios_clock_offset = 0

        self.clock_sub = self.create_subscription(Float64, '/ios_clock', self.handle_ios_clock, 10)
        self.camera_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.bridge = CvBridge()
        self.cv_image = None
        self.cvmsg = CompressedImage()
        self.image_data = {}
        self.intrinsic_msg = CameraInfo()
        self.updated_intrinsics = None
        self.last_packet_timestamp = Time(nanoseconds=0.0)

        UDP_IP = "0.0.0.0"
        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, self.port))        

        # self.create_timer(0.05, self.run)
        thread = Thread(target=self.loop_wrapper)
        thread.start()
    
    def loop_wrapper(self):
        while True:
            self.run()
            time.sleep(0.1)
    
    def run(self):
        data, addr = self.sock.recvfrom(1600)
        packet_offset = 0
        image_number, packet_number = struct.unpack('<BB', bytes(data[packet_offset:packet_offset+2]))
        packet_offset += 2

        # print(f"{image_number=} {packet_number=}")
        if packet_number == 0:
            print(f"--------- NEW IMAGE: {image_number} -----------")
            total_packets = struct.unpack('<B', bytes([data[packet_offset]]))[0]
            packet_offset += 1
            self.image_data[image_number] = {}
            self.image_data[image_number]['packets_expected'] = total_packets
            self.image_data[image_number]['packets_received'] = 1

            time_bytes = struct.unpack('<B', bytes([data[packet_offset]]))[0]
            packet_offset += 1
            intrinsic_bytes = struct.unpack('<B', bytes([data[packet_offset]]))[0]
            packet_offset += 1
            stampedTime = data[packet_offset:packet_offset+time_bytes]
            intrinsics_data = data[packet_offset+time_bytes:packet_offset+time_bytes+intrinsic_bytes]
            intrinsics_vals = [float(x) for x in intrinsics_data.split(b",")]

            self.image_data[image_number]['timestamp'] = stampedTime
            self.image_data[image_number]['payload'] = [(packet_number, data[packet_offset+time_bytes+intrinsic_bytes:])]

            # Set camera intrinsics
            self.intrinsic_msg.header.stamp = Time(seconds=float(self.ios_clock_offset) + float(stampedTime)).to_msg()
            self.intrinsic_msg.width = int(intrinsics_vals[6])
            self.intrinsic_msg.height = int(intrinsics_vals[5])
            self.intrinsic_msg.distortion_model = 'plumb_bob'
            self.intrinsic_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.intrinsic_msg.k = [intrinsics_vals[0], 0.0, intrinsics_vals[3], # Preemptively switch the principal point offsets since the image will be transposed and flipped
             		 	            0.0, intrinsics_vals[1], intrinsics_vals[2], 0.0, 0.0, 1.0]
            self.intrinsic_msg.p = [intrinsics_vals[0], 0.0, intrinsics_vals[3], 0.0,
             		 	            0.0, intrinsics_vals[1], intrinsics_vals[2], 0.0, 0.0, 0.0, 1.0, 0.0]

            self.image_data[image_number]['intrinsics_message'] = self.intrinsic_msg

        elif image_number in self.image_data.keys():
            self.image_data[image_number]['packets_received'] += 1
            self.image_data[image_number]['payload'] += [(packet_number, data[packet_offset:])]
            print(f"Packets received:\t{self.image_data[image_number]['packets_received']}")
            print(f"Packets expected:\t{self.image_data[image_number]['packets_expected']}")
            # Check if all of the packets for an image have been received
            if self.image_data[image_number]['packets_received'] == self.image_data[image_number]['packets_expected']:
                self.complete_packet_assembly(image_number)
                # Ensure images are not published if an image with a later timestamp has already been published
                # if self.last_packet_timestamp == Time(seconds=0.0) or self.last_packet_timestamp.nanosec < self.cvmsg.header.stamp.nanosec:
                print("---------- PUBLISHING IMAGE -----------")
                self.camera_pub.publish(self.cvmsg)
                self.camera_info_pub.publish(self.image_data[image_number]['intrinsics_message'])
                self.last_packet_timestamp = self.cvmsg.header.stamp
    
    def handle_ios_clock(self, msg):
        self.ios_clock_offset = msg.data

    def complete_packet_assembly(self, image_number):
        print("---------- ATTEMPTING TO PUBLISH IMAGE ----------")
        self.image_data[image_number]['payload'].sort()
        image = b''
        for packet in self.image_data[image_number]['payload']:
            image += packet[1]

        self.cvmsg.header.stamp  = Time(seconds=float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp'])).to_msg()
        self.cvmsg.header.frame_id = 'camera'
        self.cvmsg.data = image
        self.cvmsg.format = 'jpeg'

        # Convert the iOS image to a cv2 image and lower the resolution
        resize_factor = 1/3.
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(self.cvmsg)
        flipped_image = cv2.transpose(self.cv_image) # Transpose and flip the image so it is aligned with the correct camera axes
        flipped_image = cv2.flip(flipped_image, 1)
        full_res = self.bridge.cv2_to_compressed_imgmsg(flipped_image)
        self.cvmsg.data = full_res.data

        # Update the camera intrinsics for the flipped images
        self.updated_intrinsics = CameraInfo()
        self.updated_intrinsics.k = [v * resize_factor if v != 1.0 else v for v in self.image_data[image_number]['intrinsics_message'].k]
        self.updated_intrinsics.p = [v * resize_factor if v != 1.0 else v for v in self.image_data[image_number]['intrinsics_message'].p]
        self.updated_intrinsics.width = int(self.image_data[image_number]['intrinsics_message'].width * resize_factor)
        self.updated_intrinsics.height = int(self.image_data[image_number]['intrinsics_message'].height * resize_factor)
        self.updated_intrinsics.header.stamp  = Time(seconds=float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp'])).to_msg()
        self.updated_intrinsics.header.frame_id = 'camera'

def main():
    rclpy.init()
    n = ImageServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()