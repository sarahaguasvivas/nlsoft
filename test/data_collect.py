#!/usr/bin/env python2.7
from gym.block_gym import *
import time
from datetime import date
import traceback, sys
import time

B = BlockGym()

B.reset()

actions1 = list(np.arange(100, -100, -5.))
actions2 = list(np.arange(60, -100, -1.)) # need to change this
actions3 = list(np.arange(-100, 60, 1.))  # need to change this

N = 150
updown = np.random.randint(-100, 100, (N, 1))
sides = np.random.randint(-100, 60, (N, 1))
random_points = np.concatenate((updown, sides), axis = 1)

flag = True
today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")
# my_string = ','.join(map(str, my_list))

try:
    for exp in range(100):
        print "_________"
        print " Round #", exp
        print "_________"
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

        for ii in range(N):
            aaction = random_points[ii, :].tolist()
            B.step(action = aaction)
            time.sleep(0.3)
            print ii, aaction
            data = B.get_observation() + B.get_real_position()
            data = data + aaction
            print >> f, data
        B.reset()
except Exception as e:
    B.reset()
    print str(e)
    B.reset()
