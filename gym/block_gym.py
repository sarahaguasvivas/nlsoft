from optitrack.optitrack import *
from signals.serial_client import *
from dynamixels.dynamixels import *

class BlockGym():
    def __init__(self, sensor_port= "/dev/ttyACM0",  \
                            motors_port = "/dev/ttyUSB0", \
                                vrpn_ip = "192.168.50.33:3883"):
        self.observation = []
        self.verbose = 1
        self.terminal = False
        self.reward = 0
        self.info = ""

        self.motors_port = motors_port
        self.motors = DynamixelActor(port = self.motors_port)

        self.sensor_signals = OpticalSignal()

        self.state = None

        self.vrpn_ip = vrpn_ip

        self.block_or = BlockOrientation(ip = self.vrpn_ip)

        self.target = self.block_or.get_target()

    def step(self, action = [0, 0]):
        # return observation, reward, done, info
        self.motors.step(action = action)

    def reset(self):
        # return to zero position for both servos
        self.motors.reset()

    def get_info(self):
        # get info wrt the types of verbose
        info, a1, a2 = self.motors.get_info()
        print info
        return a1, a2

    def is_terminal(self):
        pass

    def get_real_position(self):
        return  self.block_or.get_observation()

    def get_observation(self):
        # use signal reader from Arduino
        signals = self.sensor_signals.collect_signal_sample()
        return  signals

    def get_target(self):
        return self.block_or.get_target()

    def reward(self):
        # Calculate cost
        pass

    def get_signal_calibration(self):
        calibration_max = np.array([0]*self.sensor_signals.num_sensors)
        import time
        # calibrate signals to find out
        # where each channel maxes
        calibration_positions = [[300, 150], [-300, 150], [-150, -150], [150, -150], [0, -50], [150, 0], [-150, 0], [0, 150], [0, -150], [0, 300], [0, -300], [300, 0], [150, 150], [-150, 150], [0, 0], [0, -10], [150, -300], [200, 300], [-200, 300], [-200, -300]]

        for count, pos in enumerate(calibration_positions):
            self.step(pos)
            for i in range(5):
                time.sleep(1)
                obs = self.get_observation()
                calibration_max = np.maximum(calibration_max,obs)
        time.sleep(1)
        self.reset()
        print calibration_max
        return calibration_max

    def stretch(self):
        import numpy as np
        # Stretch out the block before running any
        # experiment
        array1 = list(np.arange(-300, 300, 0.3))
        array2 = list(np.arange(-300, 120, 0.3))

        a1, a2= self.get_info()

        for i in array2:
            self.step([a1, i])

        for i in reversed(array2):
            self.step([a1, i])

        for i in array1:
            self.step([i, a2])

        a1 = array1[-1]
        for i in array2:
            self.step([a1, i])

        for i in array1:
            self.step([i, a2])

        self.reset()

    def done(self):
        # reset
        self.sensor_signals.close_network()
        self.motors.close_connection()

if __name__ == "__main__":
    B = BlockGym()

    while True:
        print "Observation: ", B.get_observation()
        print "Target: ", B.get_target()

