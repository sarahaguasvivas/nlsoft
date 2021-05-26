#from python.optitrack.vrpn_client import *
#from python.dynamixels.dynamixels import *

from python.signals.magnetic_sensor_signals import *

class BlockGymVani():
    def __init__(self, signal_simulator_model : str = '../models/model_signals_may_25_2021.hdf5'):
        self.observation = []
        self.verbose = 1
        self.terminal = False
        self.reward = 0
        self.info = ""
        self.sensor_signals = MagneticSignal(model = signal_simulator_model)
        self.state = None

        print("Gym Environment created!")

    def step(self, action = [0, 0]):
        pass
    def reset(self):
        pass
    def get_info(self):
        pass
    def get_state(self):
        pass
    def is_terminal(self):
        pass

    def get_observation(self, u):
        # use signal reader from Arduino
        signals = self.sensor_signals.collect_signal_sample(u)
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
    B = BlockGymVani()

