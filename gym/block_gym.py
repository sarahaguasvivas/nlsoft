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
        self.dynamixels = DynamixelActor()

        self.block_or = BlockOrientation()

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
        end_eff = self.block_or.get_observation()
        print(end_eff)
        pass

    def reward(self):
        # Calculate cost
        pass

    def done(self):
        # reset
        pass


if __name__ == "__main__":
    B = BlockGym()

    while True:
        B.get_observation()

