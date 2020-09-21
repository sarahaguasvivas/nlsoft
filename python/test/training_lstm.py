import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'gpc_controller/python'))
import numpy as np
import os
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "1"
from tensorflow.keras import regularizers
import keras
from keras.models import Sequential, load_model
from keras.layers import Dense, LSTM, BatchNormalization, Dropout, Flatten
from tensorflow.keras.losses import Huber
from sklearn.model_selection import train_test_split, KFold
import keras.backend as K
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from mpl_toolkits.mplot3d import axes3d, Axes3D
import random
NUM_DATA_RUNS = 200
TRAIN = True
from typing import List, Tuple

plt.style.use('seaborn')
filename = str(os.environ["HOME"]) + "/gpc_controller/python/data/data_Sep_08_2020.csv"

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

def huber_loss(y_true, y_pred):
    h = Huber()
    return 1000.*h(y_true, y_pred)

keras.losses.custom_loss = custom_loss

def create_network(x_train_shape : Tuple[int]):
    model = Sequential()
    model.add(LSTM(units = 15, return_sequences = True, input_shape = (x_train_shape[1],
                                                                    1)))
    model.add(Dropout(0.2))
    model.add(Flatten())
    model.add(Dense(3, activation='tanh', kernel_initializer='random_normal'))
    model.compile(optimizer="adam", loss=huber_loss, metrics=['mse'])
    return model

def neural_network_training(X, y):
    percentage = 0.60
    X = X.reshape(X.shape[0], X.shape[1], 1)

    train = range(int((X.shape[0]) * percentage))
    test = range(int((X.shape[0]) * percentage), X.shape[0])

    X_train = X[train]
    y_train = y[train]

    model = create_network(X_train.shape)
    model.fit(X_train, y_train, epochs = 100, batch_size = 1000)

    model.save('sys_id_LSTM.hdf5')
    return 'sys_id_LSTM.hdf5'

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
        plt.xticks(fontsize=14, **font)
        plt.yticks(fontsize=14, **font)
        plt.legend(prop={"family":"Times New Roman", "size": 14})
        if 2*i+1 == 1:
            plt.title("Estimated vs. Ground Truth States", **font)
        if 2*i+1 == 5:
            plt.xlabel('timesteps', **font)
        plt.subplot(3, 2, 2*i+2)
        plt.plot(y[2:L, i] - yn[2:L, i], color = '#34495E', linewidth = 0.5, label = r"$\Delta " + str(lab[i]) + " [mm]$")
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
    plt.legend(prop={"family": "Times New Roman", "size": 14})
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.title('Estimated position vs. Ground Truth')
    plt.show()

    # Testing set loss

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.5)
    y_pred= 1000.*model.predict(X_test)

    print(y_pred, y_test, y_pred.shape)
    diff = np.linalg.norm(y_test-y_pred, axis = 1)
    print("Testing set avg l2 norm:", np.mean(diff))


def prepare_data_file(filename = '../data/model_data.csv', look_back_window : int = 5):
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

    U = np.array(inputs[:-look_back_window, :])
    Y_pred = np.array(position[look_back_window:, :])
    Y_past = np.array(position[:-look_back_window, :])
    S = np.array(signals[:-look_back_window])

    L = S.shape[0]
    U = np.deg2rad(U)
    S = signals[look_back_window - 1:, :]

    for i in range(look_back_window):
        U = np.concatenate((U, inputs[look_back_window - i - 1 : L - i, :]), axis=1)

    for i in range(look_back_window):
        Y = np.concatenate((Y, position[look_back_window - i - 1  : L-i, :]), axis = 1)

    X = np.concatenate((U, Y_past), axis = 1)
    X = np.concatenate((X, S), axis = 1)
    y = Y_pred

    return X, y

if __name__ == "__main__":
    # nd is nd
    X, y = prepare_data_file([filename], look_back_window = 1)
    if TRAIN:
        modelfile, k_fold_summary = neural_network_training(X, y)
        print(k_fold_summary)
    plot_sys_id(X, y)

