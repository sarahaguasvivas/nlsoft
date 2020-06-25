#!/usr/bin/env python2.7
import numpy as np
import os
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
from tensorflow.keras import regularizers
import keras
from keras.models import Sequential, load_model
from keras.layers import Dense, GaussianNoise, LSTM, BatchNormalization
from keras.losses import Huber
from sklearn.model_selection import train_test_split
import keras.backend as K
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from mpl_toolkits.mplot3d import axes3d, Axes3D
import random
NUM_DATA_RUNS = 200
TRAIN = True

plt.style.use('seaborn')
filename = str(os.environ["HOME"]) + "/gpc_controller/data/model_data19.csv"
#filename1 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data18.csv"
#filename2 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data15.csv"
#filename3 = str(os.environ["HOME"]) + "/gpc_controller/data/model_data16.csv"

font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(14)
font = {'family' : 'serif',
        'size': 14}

plt.rcParams["font.family"] = "Times New Roman"

def custom_loss(y_true, y_pred):
    loss = K.sqrt(K.sum(K.square(y_pred- y_true), axis = -1))
    return loss

h = Huber()
def huber_loss(y_true, y_pred):
    return 1000.*h(y_true, y_pred)

keras.losses.custom_loss = custom_loss
def neural_network_training(X, y):
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.4)

    model = Sequential()
    model.add(Dense(5, activation='relu', kernel_initializer='random_normal',
                        kernel_regularizer = regularizers.l1(0.01),
                        activity_regularizer= regularizers.l2(0.02)))
    model.add(Dense(3,  activation = 'tanh', kernel_initializer='random_normal'))

    model.compile(optimizer= "adam", loss = custom_loss, metrics=['mse'])

    model.fit(X_train, y_train, epochs = 50, batch_size = 64, validation_split = 0.4)
    print model.predict(X_test)
    model.save('sys_id.hdf5')
    return 'sys_id.hdf5'

def plot_sys_id(X, y, modelfile= 'sys_id.hdf5'):
    color_palette1 = ['#272838', '#F3DE8A', '#F3DE8A', '#F3DE8A']
    color_palette = ['#1446A0', '#DB3069', '#F5D547', '#F5D547', '#3C3C3B']
    model = load_model(modelfile)
    y *= 1000 # convert meters to mm
    yn = 1000.*model.predict(X) #*1000
    plt.figure()

    lab = ['x', 'y', 'z']
    L = yn.shape[0] // NUM_DATA_RUNS

    max_target = np.max(y, axis=0)
    min_target = np.min(y, axis=0)
    shift = (max_target + min_target) / 2.

    for i in range(3):
        plt.subplot(3, 2, 2*i+1)
        plt.plot(yn[2:L, i] - shift[i], color = color_palette[0],   label = r"$" + str(lab[i]) + "_{est}$")
        plt.plot(y[2:L, i] - shift[i], color = color_palette[1], label = r"$" + str(lab[i]) + "_{true}$", alpha = 0.7)
        plt.ylabel(str(lab[i]) + " [mm]", **font)
#        plt.title("Estimation vs. Truth for " + str(lab[i]) + " [mm]")
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.legend(prop={"family":"Times New Roman", "size": 14})
        if 2*i+1 == 1:
            plt.title("Estimated vs. Ground Truth States", **font)
        if 2*i+1 == 5:
            plt.xlabel('timesteps', **font)
        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[2:L, i] - yn[2:L, i], color = '#34495E', linewidth = 0.5, label = r"$\Delta " + str(lab[i]) + " [mm]$")
#        plt.title("Error in estimation for " + str(lab[i]) + " [mm]")
        plt.ylabel(r"$\varepsilon_{" + str(lab[i]) + "}$ [mm]", **font)
        plt.ylim([-10, 10.])
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.legend(prop={"family": "Times New Roman", "size": 14})
        if 2*i+2 == 2:
            plt.title("Errors in Testing Set Predictions", **font)
        plt.xlabel('timesteps', **font)
    plt.xticks(fontsize = 14, **font)
    plt.yticks(fontsize = 14, **font)
    plt.show()
    plt.savefig("sysint", format ='svg', dpi = 1000)

    L = 100
    START = random.randint(0, yn.shape[0] - L)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(yn[START:START+L, 0], yn[START:START+L, 1], yn[START:START+L, 2],
                            linewidth = 3, alpha = 0.9, label = "estimated position")
    ax.plot3D(y[START:START + L, 0], y[START:START+L, 1], y[START:START+L, 2],
                            alpha = 1, linewidth = 3, label = "ground truth")
    ax.set_aspect('equal')
    #ax.set_xlim(-.04, .05)
    #ax.set_ylim(-.1, .05)
    #ax.set_zlim(-.05, .05)
    plt.legend(prop={"family": "Times New Roman", "size": 14})
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Estimated position vs. Ground Truth')
    plt.show()

    # Testing set loss

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.5)
    y_pred= 1000.*model.predict(X_test)

    print y_pred, y_test, y_pred.shape
    diff = np.linalg.norm(y_test-y_pred, axis = 1)
    print "Testing set avg l2 norm:", np.mean(diff)


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

    print "Y", Y
    Y = Y[:, 3:-3] # Y

    X = np.concatenate((U, Y[:, 3:]), axis = 1)
    X = np.concatenate((X, S), axis = 1)
    y = Y[:, :3] # we are using all of this to estimate current

    print np.cov(y.T, bias = True)
    return X, y


if __name__ == "__main__":
    # dd is dd+2
    # nd is nd
    X, y = prepare_data_file([filename], nd=5, dd=5+2)
    if TRAIN:
        modelfile = neural_network_training(X, y)
    plot_sys_id(X, y)

