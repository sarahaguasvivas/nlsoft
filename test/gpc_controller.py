#!/usr/bin/env python2.7
#from gym.block_gym import * # vrpn not installed in mac
from controller.GPC.python.dyno_model.neural_network_predictor import *
from controller.GPC.python.optimizer.newton_raphson import *
import numpy as np

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
plt.style.use('dark_background')
filename = "../data/model_data.csv"


def prepare_data_file(filename = '../data/model_data.csv', nd = 3, dd = 3):
    data_array = np.genfromtxt('model_data.csv', delimiter=',')
    signals = data_array[:, :11]
    position = data_array[:, 11:17]
    inputs = data_array[:, 17:]
    print signals, position, inputs





if __name__ == "__main__":
    prepare_data_file(filename, nd=3, dd=3)



