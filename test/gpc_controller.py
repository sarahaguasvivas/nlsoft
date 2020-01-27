#!/usr/bin/env python2.7
#from gym.block_gym import * # vrpn not installed in mac
from controller.dynamic_model import *
from controller.newton_raphson import *
import numpy as np
import os

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"

#plt.style.use('dark_background')
filename = "/Users/sarahaguasvivas/gpc_controller/data/model_data.csv"

def prepare_data_file(filename = '../data/model_data.csv', nd = 3, dd = 3):
    data_array = np.genfromtxt(filename, delimiter=',')
    signals = data_array[:, :11]
    position = data_array[:, 11:17]
    inputs = data_array[:, 17:]
    N = max(nd, dd) # data sample where we will start first
    U = np.empty((signals.shape[0] - N + 1, 2))
    Y = np.empty((signals.shape[0] - N + 1, 3))
    L = signals.shape[0]

    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[ nd - i - 1 + (N-nd):L-i, :]), axis = 1)

    for i in range(dd):
        Y = np.concatenate((Y, position[dd - i - 1 + (N - dd) : L-i, :]), axis = 1)

    U = U[:, 2:]
    S = signals[N - 1:, :]
    Y = Y[:, 3:] # Y

    X = np.concatenate((U, S), axis = 1)
    return X, Y




if __name__ == "__main__":
    prepare_data_file(filename, nd=3, dd=3)



