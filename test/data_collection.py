#!/usr/bin/env python2.7
from gym.block_gym import *
import time
from datetime import date
import traceback, sys

B = BlockGym()

B.reset()

actions1 = list(np.arange(150, -150, -0.3))
actions2 = list(np.arange(130, -150, -0.3)) # need to change this
actions3 = list(np.arange(-150, 130, 0.3))  # need to change this

flag = True

today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")
# my_string = ','.join(map(str, my_list))

try:
    for i in actions1:
        if flag:
            aactions = actions2
        if not flag:
            aactions = actions3

        for j in aactions:
            B.step(action=[i, j])
            print i, j
            data = B.get_observation() + B.get_real_position()
            data = data + [i, j]
            print >> f, data
        flag = not flag
    B.reset()
except Exception as e:
    print str(e)
    f.close()
    B.reset()
