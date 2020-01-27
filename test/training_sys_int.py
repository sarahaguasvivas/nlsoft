#!/usr/bin/env python2.7

from gym.block_gym import * # vrpn not installed in mac
from controller.dynamic_model import *
from controller.newton_raphson import *
import numpy as np
import os

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"

import keras
from keras.models import Sequential, load_model
from keras.layers import Dense
from sklearn.model_selection import train_test_split

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

plt.style.use('dark_background')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data.csv"


def neural_network_training(X, y):
    y = 1000*y
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.1)

    model = Sequential()
    model.add(Dense(100, activation =  'linear'))
    model.add(Dense(3,  activation = 'linear'))

    model.compile(optimizer= 'rmsprop', loss ='mse', metrics=['mse'])
    model.fit(X_train, y_train, epochs = 10000, batch_size = 10, validation_split=0.33)
    print model.predict(X_test)
    model.save('sys_id.hdf5')
    return 'sys_id.hdf5'

def plot_sys_id(X, y, modelfile= 'sys_id.hdf5'):

    model = load_model(modelfile)

    yn = model.predict(X)/ 1000
    plt.figure()

    lab = ['x', 'y', 'z']
    L = X.shape[0]

    for i in range(3):
        plt.subplot(3, 2, 2*i+1)
        plt.plot(yn[:L, i], 'r', label = str(lab[i]) + " est")
        plt.plot(y[:L, i], '-w', label = str(lab[i]) + "true", alpha = 0.7)
        plt.title("Estimation vs. Truth for " + str(lab[i]) + " [m]")
        plt.legend()

        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[:L, i] - yn[:L, i], 'cyan', linewidth = 0.5, label = str(lab[i]) + " [m]")
        plt.title("Error in estimation for " + str(lab[i]) + " [m]")
        plt.legend()

    plt.show()

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(yn[:L, 0], yn[:L, 1], yn[:L, 2], 'red', linewidth = 3, alpha = 0.9, label = "estimated position")
    ax.plot3D(y[:L, 0], y[:L, 1], y[:L, 2], 'white', alpha = 1, linewidth = 3, label = "ground truth")
    plt.legend()
    plt.title('Estimated position vs. Ground Truth')
    plt.show()


def prepare_data_file(filename = '../data/model_data.csv', nd = 3, dd = 3):
    data_array = np.genfromtxt(filename, delimiter=',')
    signals = data_array[:, :11]
    max_signals = np.max(signals, axis = 0)
    for i in range(len(max_signals)):
        if max_signals[i] == 0:
            max_signals[i] = 1

        signals[:, i]/= max_signals[i]

    position = data_array[:, 11:14] # not using Euler angles
    inputs = data_array[:, 17:]

    N = max(nd, dd) # data sample where we will start first

    U = np.empty((signals.shape[0] - N + 1, 2))
    Y = np.empty((signals.shape[0] - N + 1, 3))
    L = signals.shape[0]

    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[nd - i - 1 + (N-nd):L-i, :]), axis = 1)

    for i in range(dd ):
        Y = np.concatenate((Y, position[dd - i - 1 + (N - dd) : L-i, :]), axis = 1)

    print Y.shape
    U = U[:, 2:]
    S = signals[N - 1:, :]
    Y = Y[:, 3:-3] # Y

    X = np.concatenate((U, S), axis = 1)
    X = np.concatenate((X, Y[:, 3:]), axis = 1)

    y = Y[:, :3] # we are using all of this to estimate current

    return X, y


if __name__ == "__main__":
    X, y = prepare_data_file(filename, nd=3, dd=4)
    modelfile = neural_network_training(X, y)
    plot_sys_id(X, y)

