#!/usr/bin/env python2.7
from gym.block_gym import * # vrpn not installed in mac
from controller.dynamic_model import *
from controller.newton_raphson import *
import numpy as np
import os

#plt.style.use('dark_background')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data.csv"





def prepare_data_file(filename = '../data/model_data.csv', nd = 3, dd = 3):
    data_array = np.genfromtxt(filename, delimiter=',')
    signals = data_array[:, :11]
    position = data_array[:, 11:14] # not using Euler angles
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
    Y = Y[:, 3:-3] # Y

    X = np.concatenate((U, S), axis = 1)
    X = np.concatenate((X, Y[:, 3:]), axis = 1)

    y = Y[:, :3] # we are using all of this to estimate current

    return X, y


if __name__ == "__main__":
    X, y = prepare_data_file(filename, nd=3, dd=3)



