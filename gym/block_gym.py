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
        from_motors = self.motors.get_info()

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

    def calibrate_signals(self):
        # calibrate signals to find out
        # where each channel maxes
        pass

    def stretch(self):
        # Stretch out the block before running any
        # experiment
        pass

    def done(self):
        # reset
        self.sensor_signals.close_network()
        self.motors.close_connection()

if __name__ == "__main__":
    B = BlockGym()

    while True:
        print "Observation: ", B.get_observation()
        print "Target: ", B.get_target()

