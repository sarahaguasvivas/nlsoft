#!/usr/bin/env python2.7
from gym.block_gym import *
import time
from datetime import date
import traceback, sys

B = BlockGym()

B.reset()

today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")
# my_string = ','.join(map(str, my_list))

B.step([-40., -50.])
