from optitrack.optitrack import *
from signals.spi_read import *
from dynamixels.dynamixels import *

class BlockGym():
    def __init__(self):
        self.observation = []
        self.verbose = 1
        self.terminal = False
        self.reward = 0
        self.info = ""
        pass

    def step(self, action):
        # return observation, reward, done, info
        pass

    def reset(self):
        # return to zero position for both servos
        pass

    def get_info(self):
        # get info wrt the types of verbose
        pass

    def is_terminal(self):
        pass

    def get_observation(self):
        # use signal reader from Arduino
        pass

    def reward(self):
        # Calculate cost
        pass

    def done(self):
        # reset
        pass




