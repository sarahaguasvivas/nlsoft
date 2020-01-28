#!/usr/bin/env python2.7
from gym.block_gym import *
try:
    B = BlockGym()
    B.reset()
    print B.get_target()
    print "stretching..."
    B.stretch()
    print "calibrating signal..."
    cal = B.get_signal_calibration()
    print cal
except:
    B.reset()
