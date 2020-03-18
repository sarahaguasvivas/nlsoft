#!/usr/bin/env python2.7
from gym.block_gym import *
B = BlockGym()
B.reset()
print B.get_target()
B.step([150, 150])
print "stretching..."
B.stretch()
print "calibrating signal..."
