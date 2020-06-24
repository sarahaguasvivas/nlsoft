#!/usr/bin/env python2.7

#from gym.block_gym import * # vrpn not installed in mac
#from controller.dynamic_model import *
#from controller.newton_raphson import *
import numpy as np
import os
from tensorflow.keras.constraints import max_norm

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
from tensorflow.keras import regularizers
import keras
from keras.models import Sequential, load_model
from keras.layers import Dense, GaussianNoise, LSTM, BatchNormalization
from sklearn.model_selection import train_test_split
import keras.backend as K
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import random
NUM_DATA_RUNS = 100
TRAIN = False

#plt.style.use('dark_background')
plt.style.use('seaborn')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data17.csv"
filename1 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data18.csv"
#filename2 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data15.csv"
#filename3 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data16.csv"

def custom_loss(y_true, y_pred):
    loss = 1000.*K.sqrt(K.sum(K.square(y_pred- y_true), axis = -1))
    return loss

keras.losses.custom_loss = custom_loss
def neural_network_training(X, y):
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2)

    model = Sequential()
    model.add(Dense(10, activation='relu', kernel_initializer='random_normal',
                        kernel_regularizer = regularizers.l1(0.01),
                        activity_regularizer= regularizers.l2(0.02)))
    model.add(Dense(3,  activation = 'tanh', kernel_initializer='random_normal'))

    #optimizer = keras.optimizers.Adam(lr = 0.01)
    model.compile(optimizer= "adam", loss = custom_loss, metrics=['mse'])

    model.fit(X_train, y_train, epochs = 1000, batch_size = 1000, validation_split = 0.3)
    print model.predict(X_test)
    model.save('sys_id.hdf5')
    return 'sys_id.hdf5'

def plot_sys_id(X, y, modelfile= 'sys_id.hdf5'):

    model = load_model(modelfile)
    y *= 1000 # convert meters to mm
    yn = 1000.*model.predict(X) #*1000
    plt.figure()

    lab = ['x', 'y', 'z']
    L = 5000

    max_target = np.max(y, axis=0)
    min_target = np.min(y, axis=0)
    shift = (max_target + min_target) / 2.

    for i in range(3):
        plt.subplot(3, 2, 2*i+1)
        plt.plot(yn[2:L, i] - shift[i], color = '#E74C3C',   label = r"$" + str(lab[i]) + "_{est}$")
        plt.plot(y[2:L, i] - shift[i], color = '#5D6D7E',linestyle ='dashed', label = r"$" + str(lab[i]) + "_{true}$", alpha = 0.7)
        plt.ylabel(str(lab[i]) + " [mm]")
#        plt.title("Estimation vs. Truth for " + str(lab[i]) + " [mm]")

        plt.legend()
        if 2*i+1 == 1:
            plt.title("Estimated vs. Ground Truth States")
        if 2*i+1 == 5:
            plt.xlabel('timesteps')

        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[1:L, i] - yn[1:L, i], color = '#34495E', linewidth = 0.5, label = r"$\Delta " + str(lab[i]) + " [mm]$")
#        plt.title("Error in estimation for " + str(lab[i]) + " [mm]")
        plt.ylabel(r"$\varepsilon_{" + str(lab[i]) + "}$ [mm]")
        plt.ylim([-1., 1.])
        plt.legend()
        if 2*i+2 == 2:
            plt.title("Errors in Testing Set Predictions")
        plt.xlabel('timesteps')
    plt.show()
    plt.savefig("sysint", format ='svg', dpi = 1000)

    L = 100
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
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Estimated position vs. Ground Truth')
    plt.show()

    # Testing set loss

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.5)
    y_pred= 1000.*model.predict(X_test)

    print y_pred, y_test, y_pred.shape
    diff = np.linalg.norm(y_test-y_pred)

    print "Testing set avg l2 norm:", diff


def prepare_data_file(filename = '../data/model_data.csv', nd = 5, dd = 5):
    data_array = np.genfromtxt(filename[0], delimiter=',')
    for i in range(1, len(filename)):
        data_array = np.concatenate((data_array, np.genfromtxt(filename[i], delimiter = ',')), axis = 0)
    signals = data_array[:, :11]
    max_signals = np.max(signals, axis = 0)
    for i in range(len(max_signals)):
        if max_signals[i] == 0:
            max_signals[i] = 1

        signals[:, i]/= max_signals[i]

    position = data_array[:, 11:14] # not using Euler angles
    inputs = data_array[:, 14:]

    N = max(nd, dd) # data sample where we will start first

    U = np.empty((signals.shape[0] - N + 1, 2))
    Y = np.empty((signals.shape[0] - N + 1, 3))
    L = signals.shape[0]

    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[nd - i - 1 + (N-nd):L-i, :]), axis = 1)

    for i in range(dd):
        Y = np.concatenate((Y, position[dd - i - 1 + (N - dd) : L-i, :]), axis = 1)

    U = np.deg2rad(U[:, 2:])
    S = signals[N - 1:, :]

    Y = Y[:, 3:-3] # Y
    min_y = np.min(Y, axis =0)
    max_y = np.max(Y, axis =0)
    shift = (min_y + max_y)/2.
    Y -= shift
    print shift
    X = np.concatenate((U, Y[:, 3:]), axis = 1)
    X = np.concatenate((X, S), axis = 1)
    y = Y[:, :3] # we are using all of this to estimate current
    print y.shape
    print np.cov(y.T, bias = True)
    return X, y


if __name__ == "__main__":
    # dd is dd+2
    # nd is nd
    X, y = prepare_data_file([filename, filename1], nd=3, dd=3+2)
    if TRAIN:
        modelfile = neural_network_training(X, y)
    plot_sys_id(X, y)

