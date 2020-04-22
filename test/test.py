#!/usr/bin/env python2.7
from gym.block_gym import *
B = BlockGym()
#B.reset()

print B.get_target()

print "stretching..."
B.step([0, -100])
#B.reset()
print "Getting motors positions: "
print B.motors.get_present_position()

