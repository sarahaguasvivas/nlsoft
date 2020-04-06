#!/usr/bin/env python2.7
from gym.block_gym import *
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import datetime

B = BlockGym(vrpn_ip = "192.168.50.24:3883")
B.reset()
calibration=[]
time = []

fig = plt.subplot(1, 2, 1)
for i in range(100):
    try:
        B.calibration_max = np.array([0.0]*11)
        B.get_signal_calibration()
        calibration += [B.calibration_max.tolist()]
        sns.distplot(B.calibration_max.tolist())
        time+=[datetime.datetime.now()]
    except:
        continue
plt.subplot(1, 2, 2)

calibration = np.array(calibration)
time = np.array(time)

sns.distplot(calibration)
print "mean: ", np.mean(calibration, axis = 0), \
                "std dev: ", np.std(calibration, axis = 0)

plt.show()

data = np.concatenate((time, calibration), axis = 1)
np.savetxt("calibration_data.csv", data, delimiter=',')





