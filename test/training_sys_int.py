#!/usr/bin/env python2.7

#from gym.block_gym import * # vrpn not installed in mac
#from controller.dynamic_model import *
#from controller.newton_raphson import *
import numpy as np
import os

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"

import keras
from keras.models import Sequential, load_model
from keras.layers import Dense, GaussianNoise, LSTM, BatchNormalization
from sklearn.model_selection import train_test_split
import keras.backend as K
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import random
NUM_DATA_RUNS = 100
TRAIN = True

#plt.style.use('dark_background')
plt.style.use('seaborn')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data9.csv"

def custom_loss(y_true, y_pred):
    return 1000*K.mean(K.square(y_pred - y_true), axis = -1)

keras.losses.custom_loss = custom_loss
def neural_network_training(X, y):
    #y = 1000*y
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.4)

    model = Sequential()
    #model.add(GaussianNoise(0.1))
    model.add(Dense(15, activation =  'tanh', kernel_initializer='random_normal'))
    #model.add(GaussianNoise(0.1))
    model.add(Dense(3,  activation = 'linear', kernel_initializer='random_normal'))

    model.compile(optimizer= 'adam', loss =custom_loss, metrics=['mse'])
    model.fit(X_train, y_train, epochs = 100, batch_size = 100, validation_split=0.2)
    print model.predict(X_test)
    model.save('sys_id.hdf5')
    return 'sys_id.hdf5'

def plot_sys_id(X, y, modelfile= 'sys_id.hdf5'):

    model = load_model(modelfile)
    #y *= 1000 # convert meters to mm
    yn = model.predict(X) #*1000
    plt.figure()

    lab = ['x', 'y', 'z']
    L = X.shape[0]//NUM_DATA_RUNS

    for i in range(3):
        plt.subplot(3, 2, 2*i+1)
        plt.plot(yn[1:L, i], color = '#E74C3C',   label = r"$" + str(lab[i]) + "_{est}$")
        plt.plot(y[1:L, i], color = '#5D6D7E',linestyle ='dashed', label = r"$" + str(lab[i]) + "_{true}$", alpha = 0.7)
        plt.ylabel(str(lab[i]) + " [m]")
#        plt.title("Estimation vs. Truth for " + str(lab[i]) + " [mm]")
        plt.ylim([-0.1, 0.1])
        plt.legend()
        if 2*i+1 == 1:
            plt.title("States with respect to Timesteps")
        if 2*i+1 == 5:
            plt.xlabel('timesteps')

        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[1:L, i] - yn[1:L, i], color = '#34495E', linewidth = 0.5, label = r"$\Delta " + str(lab[i]) + " [m]$")
#        plt.title("Error in estimation for " + str(lab[i]) + " [mm]")
        plt.ylabel(r"$\varepsilon_{" + str(lab[i]) + "}$ [m]")
        plt.ylim([-0.1, 0.1])
        plt.legend()
        if 2*i+2 == 2:
            plt.title("Errors in Testing Set Predictions")
        plt.xlabel('timesteps')
    plt.show()

    L = 300
    START = random.randint(0, yn.shape[0] - L)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(yn[START:START+L, 0], yn[START:START+L, 1], yn[START:START+L, 2],  \
                            linewidth = 3, alpha = 0.9, label = "estimated position")
    ax.plot3D(y[START:START + L, 0], y[START:START+L, 1], y[START:START+L, 2], \
                            alpha = 1, linewidth = 3, label = "ground truth")
    ax.set_aspect('equal')
    #ax.set_xlim(-.04, .05)
    #ax.set_ylim(-.1, .05)
    #ax.set_zlim(-.05, .05)
    plt.legend()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Estimated position vs. Ground Truth')
    plt.show()

    # Testing set loss
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.5)
    y_pred= model.predict(X_test)

    diff = np.mean(np.linalg.norm(y_pred - y_test, axis = 1))
    print "Testing set avg l2 norm:", diff


def prepare_data_file(filename = '../data/model_data.csv', nd = 3, dd = 3):
    data_array = np.genfromtxt(filename, delimiter=',')
    signals = data_array[:, :11]
    max_signals = np.max(signals, axis = 0)
    for i in range(len(max_signals)):
        if max_signals[i] == 0:
            max_signals[i] = 1

        signals[:, i]/= max_signals[i]

    position = data_array[:, 11:14] # not using Euler angles
    print position
    inputs = data_array[:, 17:]

    N = max(nd, dd) # data sample where we will start first

    U = np.empty((signals.shape[0] - N + 1, 2))
    Y = np.empty((signals.shape[0] - N + 1, 3))
    L = signals.shape[0]

    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[nd - i - 1 + (N-nd):L-i, :]), axis = 1)

    for i in range(dd):
        Y = np.concatenate((Y, position[dd - i - 1 + (N - dd) : L-i, :]), axis = 1)

    print Y.shape
    U = U[:, 2:]/100
    S = signals[N - 1:, :]
    Y = Y[:, 3:-3] # Y

    X = np.concatenate((U, Y[:, 3:]), axis = 1)
    X = np.concatenate((X, S), axis = 1)

    y = Y[:, :3] # we are using all of this to estimate current

    return X, y


if __name__ == "__main__":
    # dd is dd+2
    # nd is nd
    X, y = prepare_data_file(filename, nd=5, dd=3)
    if TRAIN:
        modelfile = neural_network_training(X, y)
    plot_sys_id(X, y)

