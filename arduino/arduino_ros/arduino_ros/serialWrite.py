import serial
import serial.tools.list_ports as list_ports
import time

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
                        time.sleep(1)
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
            print(self.dev.readline().decode().rstrip())