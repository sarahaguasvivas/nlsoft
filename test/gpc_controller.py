#!/usr/bin/env python2.7
from gym.block_gym import * # vrpn not installed in mac
from controller.dynamic_model import *
from controller.newton_raphson import *
import numpy as np
import os

import keras
from keras.models import Sequential, load_model
from keras.layers import Dense
from sklearn.model_selection import train_test_split

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D


plt.style.use('dark_background')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data.csv"


def neural_network_training(X, y):
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2)
    model = Sequential()
    model.add(Dense(64, activation =  'linear'))
    model.add(Dense(3,  activation = 'linear'))

    model.compile(optimizer= 'rmsprop', loss ='mse', metrics=['mse'])
    model.fit(X_train, y_train, epochs = 50, batch_size = 32)
    print model.predict(X_test)
    print y_test
    model.save('sys_id.hdf5')
    return 'sys_id.hdf5'

def plot_sys_id(X, y, modelfile= 'sys_id.hdf5'):

    model = load_model(modelfile)

    yn = model.predict(X)
    plt.figure()

    lab = ['x', 'y', 'z']

    for i in range(3):
        plt.subplot(3, 2, 2*i+1)
        plt.plot(yn[:, i], 'r', label = str(lab[i]) + " est")
        plt.plot(y[:, i], '-w', label = str(lab[i]) + "true", alpha = 0.7)
        plt.title("Estimation vs. Truth for " + str(lab[i]) + " [m]")
        plt.legend()

        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[:, i] - yn[:, i], 'b', label = str(lab[i]) + " [m]")
        plt.title("Error in estimation for " + str(lab[i]) + " [m]")
        plt.legend()

    plt.show()

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(yn[:, 0], yn[:, 1], yn[:, 2], 'red')
    ax.plot3D(y[:, 0], y[:, 1], y[:, 2], 'white', alpha = 0.7)
    plt.show()


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
    #modelfile = neural_network_training(X, y)
    plot_sys_id(X, y)

