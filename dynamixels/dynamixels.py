from pyax12.connection import Connection
import time

class DynamixelActor:
    def __init__(self, serial_port = '/dev/ttyUSB0', baudrate = 57600, verbose = True):
        self._dynamixel1 = 1
        self._dynamixel2 = 2
        self.baudrate = baudrate
        self.serial_port = serial_port
        self._serial_connection = Connection(port=serial_port, \
                                    baudrate = self.baudrate)
        self._zero1= 0.
        self._zero2= -50.

        self._angle1 = self.zero1
        self._angle2 = self.zero2

        self.__info = ""

        self.verbose = verbose


    def step(self, action=[0.0, 0.0]):
        self._angle1 , self._angle2 = action
        self._serial_connection.goto(self._dynamixel1, \
                            self._angle1, speed = 512, degrees = True)
        self._serial_connection.goto(self._dynamixel2, \
                            self._angle2, speed = 512, degrees = True)

    def reset(self):
        self._angle1= self._zero1
        self._angle2= self._zero2
        self._serial_connection.goto(self._dynamixel1, \
                            self._zero1, speed = 512, degrees = True)
        self._serial_connection.goto(self._dynamixel2, \
                            self._zero2, speed = 512, degrees = True)


    def get_info(self):
        self.__info = "Dynamixels: ID1= " + str(self._angle1) + \
                                " degrees;ID2= " + str(self._angle2) + " degrees"
        return self.__info

    def close_connection(self):
        self._serial_connection.close()
        return True
