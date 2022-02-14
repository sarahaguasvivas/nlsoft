import numpy as np
from os import listdir
from os.path import isfile, join
from keras.models import Sequential, load_model
from keras.layers import Dense, LSTM, BatchNormalization, Dropout, Flatten, GRU, Conv1D, MaxPool1D
from tensorflow.keras.losses import Huber
from sklearn.model_selection import train_test_split, KFold, TimeSeriesSplit
import keras.backend as K
import keras
from typing import List, Tuple
import copy
import tensorflow as tf
import matplotlib.pyplot as plt
from collections import deque

#  Channel description:
#  Channel 0: Timestamp
#  Channel 1--6: Mag_x data
#  Channel 7--12: Mag_y data
#  Channel 13--18: Mag_z data
#  Channel 19--25: PWM inputs
#  Channel 26--39: Other positions markers
#  Channel 40: Timestamp marker
#  Channel 41--43: {x, y, z} markers
#  Channel 44--48: Centroid position
#  Channel 49--52: {q0, q1, q2, q3}
#
NUM_SENSORS = 18
N_D = 2
D_D = 2
DATA_SIZE = N_D*6 + D_D*3 + NUM_SENSORS
TRAINING = True

data_files_location = "../../data/12_23_2021"
regions = ['SV_tilt_' + str(i) for i in range(1, 20)]

def prepare_data_file_vani(signals, position, inputs, nd=3, dd=3):
    dd = copy.copy(dd) + 2  # adjusting for shifted columns
    signals = signals.astype(np.float32)
    position = position.astype(np.float32)
    position = position - position[0, :]
    #max_signals = np.clip(np.max(signals, axis=0), 1, np.inf)
    #print(max_signals)
    signals = np.clip(signals / 1e4 - 0.5, 0, np.inf)
    inputs = inputs / np.max(inputs, axis = 0) - 0.5

    N = max(nd, dd)  # data sample where we will start first

    U = np.empty((signals.shape[0] - N + 1, 6))
    Y = np.empty((signals.shape[0] - N + 1, 3))
    L = signals.shape[0]
    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[nd - i - 1 + (N - nd):L - i, :]), axis=1)

    for i in range(dd):
        Y = np.concatenate((Y, position[dd - i - 1 + (N - dd): L - i, :]), axis=1)

    U = U[:, 6:]  # np.deg2rad(U[:, 2:])
    S = signals[N - 1:, :]

    Y = Y[:, 3:-3]  # Y

    X = np.concatenate((U, Y[:, 3:]), axis=1)
    X = np.concatenate((X, S), axis=1)
    y = Y[:, :3]  # we are using all of this to estimate current
    return X, y

def huber_loss(y_true, y_pred):
    h = Huber()
    return h(y_true, y_pred)

def create_gru_network(x_train_shape: Tuple[int]):
    model = Sequential()
    model.add(GRU(units = 15, input_shape = (1, x_train_shape[-1])))
    model.add(Dense(15, activation = 'relu'))
    model.add(Dense(3, activation = 'tanh', kernel_initializer='random_normal',
                            bias_constraint = tf.keras.constraints.max_norm(0.0)))
    model.compile(optimizer = "adam", loss = huber_loss, metrics=["mse"])
    return model

def create_data_labels(data):
    X = []
    y = []
    for i in range(len(data)):
        mag_x = data[i][:, 1:7]
        mag_y = data[i][:, 7:13]
        mag_z = data[i][:, 13:19]

        position = data[i][:, 41:44]
        signals = np.concatenate((mag_x, mag_y, mag_z), axis = 1)
        pwm_inputs = data[i][:, 19:25]

        dataset, labels = prepare_data_file_vani(signals = signals,
                                                position = position,
                                                inputs = pwm_inputs,
                                                nd = N_D, dd = D_D)
        X += [dataset]
        y += [labels]
    print(X[0].shape)
    return np.vstack(X), np.vstack(y)

def gru_training(data):
    data_shape = data[0].shape
    model = create_gru_network(x_train_shape = (None, 1, DATA_SIZE))
    X, y = create_data_labels(data)
    kfold = TimeSeriesSplit(n_splits = 10)
    for train, test in kfold.split(X, y):
        x_train = X[train].reshape(-1, 1, DATA_SIZE)
        y_train = y[train]
        model.fit(x_train, y_train, epochs = 20, batch_size = 50)
    model.save('forward_kinematics_jan_10_2022.hdf5')
    return X, y, model

if __name__=='__main__':
    #plt.style.use('seaborn')
    files = []
    for region in regions:
      files += [f for f in listdir(data_files_location)
                  if isfile(join(data_files_location, region)) and f[:4] == 'SV_t']
    print(files)

    data = []
    for i, region in enumerate(regions):
      data += [np.genfromtxt(join(data_files_location, region),
                             delimiter = ',',
                              invalid_raise = False)]

    if TRAINING:
        X, y, forward_kinematics_model = gru_training(data)
    else:
        X, y = create_data_labels(data)

    forward_kinematics_model = keras.models.load_model('forward_kinematics_jan_10_2022.hdf5', compile=False)
    samples = 10000
    X_sysint = X[:samples, :]
    y_true_sysint = y[:samples, :]
    y_pred_sysint = forward_kinematics_model.predict(
        X_sysint.reshape(X_sysint.shape[0], 1, X_sysint.shape[1])
    )  # our predictions!

    plt.figure(figsize=(7, 5))
    plt.subplot(3, 2, 1)
    plt.plot(1000 * y_true_sysint[:, 0], '--k', linewidth=2)
    plt.plot(1000 * y_pred_sysint[:, 0], 'r', linewidth=2)
    plt.tick_params(axis = 'x', which = 'both', bottom = False, top = False, labelbottom = False)
    plt.legend([r'$x_{true}$', r'$\hat{x}_{GRU}$'])
    plt.title("Predictions on Test Set Data")
    plt.ylabel('x [mm]')

    plt.subplot(3, 2, 3)
    plt.plot(1000 * y_true_sysint[:, 1], '--k', linewidth=2)
    plt.plot(1000 * y_pred_sysint[:, 1], 'r', linewidth=2)
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.legend([r'$y_{true}$', r'$\hat{y}_{GRU}$'])
    plt.ylabel('y [mm]')

    plt.subplot(3, 2, 5)
    plt.plot(1000 * y_true_sysint[:, 2], '--k', linewidth=2)
    plt.plot(1000 * y_pred_sysint[:, 2], 'r', linewidth=2)
    plt.ylim(-10, 10)
    plt.legend([r'$z_{true}$', r'$\hat{z}_{GRU}$'])
    plt.ylabel('z [mm]')

    plt.subplot(3, 2, 2)
    plt.plot(1000 * y_true_sysint[:, 0] - 1000 * y_pred_sysint[:, 0])
    plt.ylim([-2, 2])
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.title("Error in Prediction on Testing Set Data")
    plt.ylabel('error x [mm]')

    plt.subplot(3, 2, 4)
    plt.plot(1000 * y_true_sysint[:, 1] - 1000 * y_pred_sysint[:, 1])
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.ylim([-2, 2])
    plt.ylabel('error y [mm]')

    plt.subplot(3, 2, 6)
    plt.plot(1000 * y_true_sysint[:, 2] - 1000 * y_pred_sysint[:, 2])
    plt.ylim([-2, 2])
    plt.ylabel('error z [mm]')
    plt.savefig('sysint.png', dpi=300, format='png')

    plt.figure()
    plt.plot(X[:, -18:])
    plt.savefig('sensors.png', format = 'png')