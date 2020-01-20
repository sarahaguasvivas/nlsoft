from pyax12.connection import Connection
import time

class DynamixelActor:
    def __init__(self, serial_port = '/dev/ttyUSB0', baudrate = 57600):
        self.dynamixel1 = 1
        self.dynamixel2 = 2
        self.baudrate = baudrate
        self.serial_port = serial_port
        self.serial_connection = Connection(port=serial_port, \
                                    baudrate = self.baudrate)
        self.zero1= 0.
        self.zero2= -50.

    def step(self, action=[0.0, 0.0]):
        pass

    def reset(self):
        self.serial_connection.goto(self.dynamixel1, \
                            self.zero1, speed = 512, degrees = True)
        self.serial_connection.goto(self.dynamixel2, \
                            self.zero2, speed = 512, degrees = True)

    def close_connection(self):
        self.serial_connection.close()
        return True
