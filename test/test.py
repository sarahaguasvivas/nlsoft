#!/usr/bin/env python2.7
from gym.block_gym import *
B = BlockGym()

B.step([0.0, -40.])
print B.get_state()
print "Getting motors positions: "
print B.motors.get_present_position()

