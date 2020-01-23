from gym.block_gym import *
import time
from datetime import date

B = BlockGym()

B.reset()

actions1 = list(range(-150, 150, 10))
actions2 = list(range(-150, 150, 20))

today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y")+".txt", "w+")

try:
    for i in actions1:
        for j in actions2:
            B.step(action=[i, j])
            print i, j
            data = B.get_observation() + B.get_real_position()
            print data
            data = data + [i, j]
            print >> f, data
            time.sleep(1)
except:
    f.close()
    B.reset()
