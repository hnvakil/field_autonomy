"""
Node to receive live camera feed from iOS device, process it, and publish it to ROS
Adapted from: https://github.com/occamLab/ARKit-Ros-Bridge
"""

import rclpy
import socket
import struct
from threading import Thread
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, CameraInfo

class ImageServerNode(Node):
    def __init__(self):
        super().__init__('image_server')

        # Get port from ROS params
        self.declare_parameter('port')
        self.port = self.get_parameter('port').value

        # Declare variables
        self.ios_clock_offset = 0

        # Create publishers and subscribers
        self.create_subscription(Float64, '/ios_clock', self.handle_ios_clock, 10)
        self.camera_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Declare variables for images
        self.cvmsg = CompressedImage()      # CompressedImage msg
        self.image_data = {}                # Dictionary for image data
        self.intrinsic_msg = CameraInfo()   # Camera intrinsics
        self.last_packet_timestamp = None

        # Set up UDP connection
        UDP_IP = "0.0.0.0"
        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, self.port))        

        # Start thread to receive and process images
        thread = Thread(target=self.loop_wrapper)
        thread.start()
    
    def loop_wrapper(self):
        """ Wrap run loop for flexibility and debugging """
        while True:
            self.run()
    
    def run(self):
        """ Run function for node to receive, process, and publish camera images and intrinsics """
        # Receive data
        data, addr = self.sock.recvfrom(1600)
        packet_offset = 0
        # Image and packet num from first two bytes
        image_number, packet_number = struct.unpack('<BB', bytes(data[packet_offset:packet_offset+2]))
        packet_offset += 2

        # Process a new image
        if packet_number == 0:
            total_packets = struct.unpack('<B', bytes([data[packet_offset]]))[0]    # Total expected packets
            packet_offset += 1
            # Start constructing image dictionary
            self.image_data[image_number] = {}
            self.image_data[image_number]['packets_expected'] = total_packets
            self.image_data[image_number]['packets_received'] = 1

            time_bytes = struct.unpack('<B', bytes([data[packet_offset]]))[0]       # Length of timestamp
            packet_offset += 1
            intrinsic_bytes = struct.unpack('<B', bytes([data[packet_offset]]))[0]  # Length of intrinsics
            packet_offset += 1
            stampedTime = data[packet_offset:packet_offset+time_bytes]              # Timestamp
            intrinsics_data = data[packet_offset+time_bytes:packet_offset+time_bytes+intrinsic_bytes]   # Camera instrinsics
            intrinsics_vals = [float(x) for x in intrinsics_data.split(b",")]

            self.image_data[image_number]['timestamp'] = stampedTime
            self.image_data[image_number]['payload'] = [(packet_number, data[packet_offset+time_bytes+intrinsic_bytes:])] # Retreive image data

            # Set camera intrinsics
            self.intrinsic_msg.header.stamp = Time(seconds=float(self.ios_clock_offset) + float(stampedTime)).to_msg()
            self.intrinsic_msg.width = int(intrinsics_vals[5])
            self.intrinsic_msg.height = int(intrinsics_vals[6])
            self.intrinsic_msg.distortion_model = 'plumb_bob'
            self.intrinsic_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.intrinsic_msg.k = [intrinsics_vals[0], 0.0, intrinsics_vals[2],
             		 	            0.0, intrinsics_vals[1], intrinsics_vals[3], 0.0, 0.0, 1.0]
            self.intrinsic_msg.p = [intrinsics_vals[0], 0.0, intrinsics_vals[2], 0.0,
             		 	            0.0, intrinsics_vals[1], intrinsics_vals[3], 0.0, 0.0, 0.0, 1.0, 0.0]

            self.image_data[image_number]['intrinsics_message'] = self.intrinsic_msg

        # Add to an existing image
        elif image_number in self.image_data.keys():
            self.image_data[image_number]['packets_received'] += 1
            self.image_data[image_number]['payload'] += [(packet_number, data[packet_offset:])]
            # Check if all of the packets for an image have been received
            if self.image_data[image_number]['packets_received'] == self.image_data[image_number]['packets_expected']:
                self.complete_packet_assembly(image_number)
                # Ensure images are not published if an image with a later timestamp has already been published
                if self.last_packet_timestamp is None or Time().from_msg(self.last_packet_timestamp) < Time().from_msg(self.cvmsg.header.stamp):
                    self.camera_pub.publish(self.cvmsg)
                    self.camera_info_pub.publish(self.image_data[image_number]['intrinsics_message'])
                    self.last_packet_timestamp = self.cvmsg.header.stamp
        
    def handle_ios_clock(self, msg):
        """ Handle difference between iOS clock and ROS clock """
        self.ios_clock_offset = msg.data

    def complete_packet_assembly(self, image_number):
        """ Convert compiled packets to a CompressedImage """
        # Sort image data
        self.image_data[image_number]['payload'].sort()
        image = b''
        # Assemble packets into image
        for packet in self.image_data[image_number]['payload']:
            image += packet[1]

        # Create CompressedImage message
        self.cvmsg.header.stamp  = Time(seconds=float(self.ios_clock_offset) + float(self.image_data[image_number]['timestamp'])).to_msg()
        self.cvmsg.header.frame_id = 'camera'
        self.cvmsg.data = image
        self.cvmsg.format = 'jpeg'

def main():
    rclpy.init()
    n = ImageServerNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()