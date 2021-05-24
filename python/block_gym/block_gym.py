from python.optitrack.vrpn_client import *
from python.signals.serial_client import *
from python.dynamixels.dynamixels import *

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
        self.block_or = BlockState(ip = self.vrpn_ip)
        #self.target = self.block_or.get_target() # TODO: Uncomment when we find the stick
        self.calibration_max = np.array([0]* \
                    self.sensor_signals.num_sensors)

        print("Gym Environment created!")

    def step(self, action = [0, 0]):
        print("here")
        obs = self.get_observation()
        print("here1")
        self.calibration_max = np.maximum(self.calibration_max,obs)
        # return observation, reward, done, info
        self.motors.step(action = action)

    def reset(self):
        # return to zero position for both servos
        self.motors.reset()

    def get_info(self):
        # get info wrt the types of verbose
        info, a1, a2 = self.motors.get_info()
        print(info)
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
        self.calibration_max = np.maximum(signals, self.calibration_max)
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
        zero = [self.motors._zero1, self.motors._zero2]
        calibration_positions = [[50, 20],zero, [-100, -100], \
                           zero, [-100, 20], zero, [50, -100], zero,
                           [-20, 20], zero, [-20, -100], zero, [-100, -90], zero, \
                                   [50, -90], zero, [50, -100], zero, \
                                   [50, 10], zero, [50, -100], zero, [-90, -90], zero, \
                                   [50, -90], zero, [50, -90]]

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
        print("BLOCK: calibration vector : ", self.calibration_max)
        return self.calibration_max

    def stretch(self):
        import numpy as np
        # Stretch out the block before running any experiment
        zero = [self.motors._zero1, self.motors._zero2]
        calibration_positions = [[-100, 100], zero, [-100, 100], zero, \
                                [100, -100], zero, [100, 100]]
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
        print("Observation: ", B.get_observation())
        print("Target: ", B.get_target())

