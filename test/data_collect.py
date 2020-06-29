#!/usr/bin/env python2.7
from gym.block_gym import *
import time
from datetime import date
import traceback, sys
import time

B = BlockGym()

B.reset()

actions1 = list(np.arange(80, -100, -5.))
actions2 = list(np.arange(40, -100, -1.)) # need to change this
actions3 = list(np.arange(-100, 40, 1.))  # need to change this

actions4 = list(np.arange(40, -100, -5.))
actions5 = list(np.arange(80, -100, -1.)) # need to change this
actions6 = list(np.arange(-100, 80, 1.))  # need to change this

actions7 = list((1.+np.cos(2.* np.pi / 1000. * (np.arange(10000))))/2. * 180. - 100.)
actions8 = list((1.+np.sin(2.* np.pi / 1000. * (np.arange(10000))))/2. * 150. - 100.)

N = 200

today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".csv", "w+")
# my_string = ','.join(map(str, my_list))
try:
    for exp in range(N):
        flag = True
        print "_________"
        print " Round #", exp
        print "_________"

        print "circles"
        for j, i in enumerate(actions7):
            B.step(action = [i, actions8[j]])
            print i, actions8[j]
            data = B.get_observation() + B.get_real_position()
            data = data + [i, actions8[j]]
            print >> f, ", ".join(map(str, data))
        B.reset()
        print "sides"
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
                print >> f, ", ".join(map(str, data))
            flag = not flag
        B.reset()
        print "updown"
        flag = True
        for i in actions4:
            if flag:
                aactions = actions5
            if not flag:
                aactions = actions6

            for j in aactions:
                B.step(action=[j, i])
                print j, i
                data = B.get_observation() + B.get_real_position()
                data = data + [j, i]
                print >> f, ", ".join(map(str, data))
            flag = not flag
        B.reset()
    B.reset()
except Exception as e:
    B.reset()
    print str(e)
    B.reset()
