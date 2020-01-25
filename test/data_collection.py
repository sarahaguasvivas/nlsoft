#!/usr/bin/env python2.7
from gym.block_gym import *
import time
from datetime import date
import traceback, sys

B = BlockGym()

B.reset()
# 102.7 -42.8

actions1 = list(np.arange(150, -150, -0.3))
actions2 = list(np.arange(130, -150, -0.3))
actions3 = list(np.arange(-150, 130, 0.3))

flag = True

today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")

try:
    for i in actions1:
        if flag:
            for j in actions2:
                B.step(action=[i, j])
                #time.sleep(0.5)
                print i, j
                data = B.get_observation() + B.get_real_position()
                data = data + [i, j]
                print >> f, data
        if not flag:
            for j in actions3:
                print i, j
                B.step(action = [i, j])
                data = B.get_observation() + B.get_real_position()
                data = data + [i, j]
                print >> f, data
        flag = not flag

    B.reset()
except Exception as e:
    print str(e)
    f.close()
    B.reset()
