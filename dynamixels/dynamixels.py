import pypot.dynamixel as pydxl

class DynamixelActor:
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 57600, verbose = True):

        self.baudrate = baudrate
        self.serial_port = port

        self._serial_connection = pydxl.DxlIO(self.serial_port,  baudrate = self.baudrate)
        print "Scanned motor ids:", self._serial_connection.scan()
        self._dynamixel1, self._dynamixel2 =  self._serial_connection.scan([1, 2])

        self._zero1= 0
        self._zero2= -50

        self._angle1 = self._zero1
        self._angle2 = self._zero2

        self.__info = ""
        self.verbose = verbose
        print "Dynamixels started!"

    def get_present_position(self):
        return list(self._serial_connection.get_present_position((1,2)))

    def step(self, action=[0, 0]):
        self._angle1 , self._angle2 = action
        goal_pos = {self._dynamixel1: self._angle1, self._dynamixel2 : self._angle2}
        self._serial_connection.set_goal_position(goal_pos)

    def reset(self):
        self.step(action=[self._zero1, self._zero2])

    def get_info(self):
        self._angle1 , self._angle2 = self.get_present_position()

        self.__info = "Dynamixels: ID1= " + str(self._angle1) + \
                                " degrees;ID2= " + str(self._angle2) + " degrees"
        return self.__info, self._angle1, self._angle2

    def close_connection(self):
        self._serial_connection.close()
        return True

if __name__=="__main__":
    import time
    motors = DynamixelActor()
    motors.get_present_position()
    motors.step(action=[150, -50])
    time.sleep(1)
    motors.reset()
    print motors.get_info()
    motors.close_connection()
