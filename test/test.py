#!/usr/bin/env python2.7
from gym.block_gym import *
try:
    B = BlockGym()
    B.reset()
    print "stretching..."
    B.stretch()
    print "calibrating signal..."
    B.get_signal_calibration()
except:
    B.reset()
