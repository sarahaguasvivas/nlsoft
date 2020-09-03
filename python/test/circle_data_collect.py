from gym.block_gym import *
import time
from datetime import date
import traceback, sys
import time

B = BlockGym()

B.reset()
wavelengths = np.arange(100, 1000, 100).tolist()
actions1 = []
actions2 = []
actions3 = []
times = 1000
for wave in wavelengths:
    actions1 += np.clip(100*np.cos(2.*np.pi /wave * np.arange(0, times, 1)), -100, 60).tolist() #list(np.arange(60, -100, -5.))
    actions2 += np.clip(100*np.sin(2.*np.pi /wave * np.arange(times, 0, -1)), -100, 80).tolist()#list(np.arange(80, -100, -1.)) # need to change this
    actions3 += np.clip(100*np.sin(2.*np.pi / wave *np.arange(0, times, 1)), -100, 80).tolist() #list(np.arange(-100, 80, 1.))  # need to change this

N = 0
updown = np.random.randint(-100, 100, (N, 1))
sides = np.random.randint(-100, 60, (N, 1))
random_points = np.concatenate((updown, sides), axis = 1)

flag = True
today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".csv", "w+")
# my_string = ','.join(map(str, my_list))

try:
    for exp in range(100):
        print "_________"
        print " Round #", exp
        print "_________"
        for enum, i in enumerate(actions1):
            B.step(action=[i, actions2[enum]])
            print i, actions2[enum]
            data = B.get_observation() + B.get_real_position()
            data = data + [i, actions2[enum]]
            print >> f, ", ".join(map(str, data))

        for enum, i in enumerate(actions1):
            B.step(action=[i, actions3[enum]])
            print i, actions2[enum]
            data = B.get_observation() + B.get_real_position()
            data = data + [i, actions3[enum]]
            print >> f, ", ".join(map(str, data))
        B.reset()
except Exception as e:
    B.reset()
    print str(e)
    B.reset()
