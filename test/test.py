#!/usr/bin/env python2.7
from gym.block_gym import *
B = BlockGym()

print B.get_target()

print "stretching..."
B.step([0.0, -50])
print "Getting motors positions: "
print B.motors.get_present_position()

