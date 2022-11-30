import serial
import serial.tools.list_ports as list_ports
import time,rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

"""
writer = serialWrite.Serial_cmd()
time.sleep(2)
writer.write_data_to_arduino("0.2,0.0\n")
print("past first write")
time.sleep(2)
writer.write_data_to_arduino("0.0,0.0\n")
print("past second write")
"""

class Serial_cmd:
    Arduino_IDs = ((0x2341, 0x0043), (0x2341, 0x0001), 
                   (0x2A03, 0x0043), (0x2341, 0x0243), 
                   (0x0403, 0x6001), (0x1A86, 0x7523),
                   (0x9025, 0x0067))
    
    def __init__(self, port=''):
        print("test abc")
        if port == '':
            self.dev = None
            self.connected = False
            devices = list_ports.comports()
            for device in devices:
                print(device)
                print(type(device))
                print(device.vid)
                print(type(device.vid))
                if device.vid == 9025:
                    print("before try")
                    try:
                        self.dev = serial.Serial(device.device, 9600)
                        self.connected = True
                        print('Connected to {!s}...'.format(device.device))
                    except:
                        print("an exception occurred :(")
                if self.connected:
                    break
        else:
            try:
                self.dev = serial.Serial(port, 9600)
                self.connected = True
            except:
                self.edev = None
                self.connected = False
    def write_data_to_arduino(self, string_to_write):
        if self.connected:
            self.dev.write(string_to_write.encode())
        else:
            raise Exception("self not connected!")
    def read_data(self):
        if self.connected:
            try:
                print(self.dev.readline().decode().rstrip())
            except:
                pass

class TreadWriterNode(Node):
    def __init__(self):
        super().__init__('tread_writer')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(Twist, 'cmd_vel', self.process_twist, 10)

        self.linear = 0.0
        self.angular = 0.0
        self.writer = Serial_cmd()
        print("initializing")
        if not self.writer.connected:
            raise Exception("error connecting")

    def run_loop(self):
        print("")

        to_print = ""
        #print("start python prints:")
        to_print += "start python prints: "
        #print(f"self.linear: {self.linear} of type {type(self.linear)}")
        #print(f"self.angular: {self.angular} of type {type(self.angular)}")
        to_print += f"self.linear: {self.linear} of type {type(self.linear)}"
        to_print += f"self.angular: {self.angular}"
        
        string_to_write = f"<{self.linear},{self.angular}\n"
        #print(string_to_write)
        to_print += string_to_write
        self.writer.write_data_to_arduino(string_to_write)
        start = time.time()
        while self.writer.dev.in_waiting:
            self.writer.read_data()
        #print("elapsed " + str(time.time() - start))
        #print("end python prints")
        print(to_print)
        print("")

    def process_twist(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
    

def main(args=None):
    rclpy.init(args=args)
    node = TreadWriterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()