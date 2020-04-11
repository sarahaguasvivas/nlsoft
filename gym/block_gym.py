from optitrack.optitrack import *
from signals.serial_client import *
from dynamixels.dynamixels import *

class BlockGym():
    def __init__(self, sensor_port= "/dev/ttyACM0",  \
                            motors_port = "/dev/ttyUSB0", \
                                vrpn_ip = "192.168.50.24:3883"):
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
        #self.target = self.block_or.get_target() # TODO: Uncomment when we find the stick
        self.calibration_max = np.array([0]* \
                    self.sensor_signals.num_sensors)

        print "Gym Environment created!"

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

    def get_state(self):
        position = self.get_real_position()
        return position[:3]

    def is_terminal(self):
        pass

    def get_real_position(self):
        return  self.block_or.get_observation()

    def get_observation(self):
        # use signal reader from Arduino
        signals = self.sensor_signals.collect_signal_sample()
        return  signals

    def get_target(self):
        return self.block_or.get_target()[:3]

    def reward(self):
        # Calculate cost
        pass

    def get_signal_calibration(self):
        import time
        # calibrate signals to find out
        # where each channel maxes
        calibration_positions = [[300, 150], [-300, 150], \
                                [-150, -150], [150, -150], \
                                [0, -50], [150, 0], [-150, 0], \
                                [0, 150], [0, -150], [0, 300], [0, -300], \
                                [300, 0], [150, 150], [-150, 150], \
                                [0, 0], [0, -10], [150, -300], \
                                [200, 300], [-200, 300],\
                                [-200, -300], [0, 150], \
                                [0, -200], [-150, -200],\
                                [150, -200]]

        for count, pos in enumerate(calibration_positions):
            self.step(pos)
            time.sleep(1)
            for i in range(200):
                obs = self.get_observation()
                self.calibration_max = np.maximum(self.calibration_max,obs)
            time.sleep(1)
        self.reset()
        for i in range(len(self.calibration_max)):
            if self.calibration_max[i] == 0:
                self.calibration_max[i] = 1
        print "BLOCK: calibration vector : ", self.calibration_max
        return self.calibration_max

    def stretch(self):
        import numpy as np
        # Stretch out the block before running any experiment
        calibration_positions = [[300, 300], [-300, 300],\
                                [-300, -300], [300, -300]]
        for count, pos in enumerate(calibration_positions):
            self.step(pos)
            time.sleep(1)
        self.reset()

    def done(self):
        self.sensor_signals.close_network()
        self.motors.close_connection()

if __name__ == "__main__":
    B = BlockGym()

    while True:
        print "Observation: ", B.get_observation()
        print "Target: ", B.get_target()

