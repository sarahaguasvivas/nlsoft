#!/usr/bin/env python2.7
from gym.block_gym import *
B = BlockGym()
B.reset()

print B.get_target()

B.step([-115, -90])
print "stretching..."
#B.stretch()
print "Getting motors positions: "
print B.motors.get_present_position()

