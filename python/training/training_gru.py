import numpy as np
from os import listdir
from os.path import isfile, join
from keras.models import Sequential, load_model
from keras.layers import Dense, LSTM, BatchNormalization, Dropout, Flatten, GRU, Conv1D, MaxPool1D
from tensorflow.keras.losses import Huber
from sklearn.model_selection import train_test_split, KFold, TimeSeriesSplit
import keras
from typing import List, Tuple
import tensorflow as tf
import matplotlib.pyplot as plt

plt.style.use('seaborn-whitegrid')

#  Channel description:
#  Channel 0: Timestamp
#  Channel 1--6: Mag_x data
#  Channel 7--12: Mag_y data
#  Channel 13--18: Mag_z data
#  Channel 19--25: PWM inputs
#  Channel 26--39: Other positions markers
#  Channel 39: Timestamp marker
#  Channel 40--43: {x, y, z} markers
#  Channel 43--47: Centroid position
#  Channel 47--52: {q0, q1, q2, q3}
#
NUM_SENSORS = 18
N_D = 2
D_D = 2
DATA_SIZE = N_D*6 + D_D*3 + NUM_SENSORS
TRAINING = True
MODEL_NAME = "forward_kinermatics_mar_18_2022.hdf5"

data_files_location = "../data/12_23_2021/12_23_2021/"
regions = ['SV_tilt_' + str(i) for i in range(1, 20)]


def prepare_data_file_vani(signals, position, inputs, nd=3, dd=3):
    assert nd == dd, "nd and dd need to be the same"
    signals = signals.astype(np.float32)
    position = position.astype(np.float32)
    position = position - position[0, :]
    np.set_printoptions(precision=10)
    signals = np.clip(signals / 1e4 - 0.5, -0.5, 0.5)
    inputs = inputs / np.max(inputs, axis = 0) - 0.5

    N = max(nd, dd)  # data sample where we will start first

    U = np.empty((signals.shape[0] - N, 0))
    Y = np.empty((signals.shape[0] - N, 0))
    L = signals.shape[0]
    # TODO: Test for when nd neq dd
    for i in range(nd):
        U = np.concatenate((U, inputs[i:L-nd + i, :]), axis=1)

    for i in range(dd + 1):
        Y = np.concatenate((Y, position[i:L-dd + i, :]), axis=1)
    S = signals[N:, :]
    Y = Y[:, 3:]  # Y

    X = np.concatenate((U, Y), axis=1)
    X = np.concatenate((X, S), axis=1)
    y = Y[:, :3]  # we are using all of this to estimate current
    print(X.shape, y.shape)
    assert DATA_SIZE == X.shape[1], "data size not consistent"
    return X, y

def huber_loss(y_true, y_pred):
    h = Huber()
    return h(y_true, y_pred)
def create_gru_network(x_train_shape: Tuple[int]):
    model = Sequential()
    model.add(GRU(units = 15, input_shape = (1, x_train_shape[-1])))
    #model.add(Flatten())
    model.add(Dense(5, activation = 'relu'))
    #model.add(Dense(5, activation = 'relu'))
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
        position = data[i][:, 40:43]
        signals = np.concatenate((mag_x, mag_y, mag_z), axis = 1)
        pwm_inputs = data[i][:, 19:25]

        dataset, labels = prepare_data_file_vani(signals = signals,
                                                position = position,
                                                inputs = pwm_inputs,
                                                nd = N_D, dd = D_D)
        X += [dataset]
        y += [labels]
    X_data = np.vstack(X)
    y_data = np.vstack(y)
    #print(np.median(X_data[:, -NUM_SENSORS:], axis = 0))
    #X_data[:, -NUM_SENSORS:] = np.clip(X_data[:, -NUM_SENSORS:] - np.median(X_data[:, -NUM_SENSORS:], axis = 0), -0.5, 0.5)
    return X_data, y_data

def gru_training(data):
    data_shape = data[0].shape
    model = create_gru_network(x_train_shape = (None, 1, DATA_SIZE))
    X, y = create_data_labels(data)
    kfold = TimeSeriesSplit(n_splits = 10)
    test = None
    for train, test in kfold.split(X, y):
        x_train = X[train].reshape(-1, 1, DATA_SIZE)
        y_train = y[train]
        model.fit(x_train, y_train, epochs = 50, batch_size = 1000)
    model.save(MODEL_NAME)
    return X, y, model, test

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
        X, y, forward_kinematics_model, test = gru_training(data)
    else:
        X, y = create_data_labels(data)
        test = np.arange(X.shape[0] - 1000,X.shape[0])
        forward_kinematics_model = keras.models.load_model(MODEL_NAME, compile=False)
    X_sysint = X[test, :]
    y_true_sysint = y[test, :]
    y_pred_sysint = forward_kinematics_model.predict(
        X_sysint.reshape(X_sysint.shape[0], 1, X_sysint.shape[1])
    )  # our predictions!
    color = 'r' #'#00A36C'
    plt.figure(figsize = (4, 4))
    plt.figure()
    plt.subplot(4, 1, 1)
    plt.plot(test*1/240, 1000 * X[test, :6], linewidth=2)
    #plt.plot(1000 * y_true_sysint[:, 0], '--k', linewidth=2)
    plt.legend([r'$u_0$',r'$u_1$',r'$u_2$',r'$u_3$', r'$u_4$', r'$u_5$'],
               shadow=True, fontsize=10, bbox_to_anchor=(0.7, 0.4, 0.3, 0.2),
               frameon=True, loc='center left', ncol = 2)
    plt.locator_params(axis='x', nbins=15)
    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    plt.subplot(4, 1, 2)
    plt.plot(test * 1 / 240, 1000 * y_true_sysint[:, 0], 'k', linewidth=2)
    plt.plot(test * 1 / 240, 1000 * y_pred_sysint[:, 0], color, linestyle = 'dashed', alpha=0.8, linewidth=2)

    plt.tick_params(axis = 'x', which = 'both', bottom = False, top = False, labelbottom = False)
    #plt.legend([r'$y_{0, true}$', r'$\hat{y}_{0, GRU}$'],
    #           shadow=True, fontsize=10, bbox_to_anchor=(0.94, 0.4, 0.3, 0.2),
    #           frameon=True, loc='center left')
    #plt.title("Predictions on Test Set Data")
    #plt.ylabel('x [mm]')
    plt.locator_params(axis='x', nbins=15)
    plt.subplot(4, 1, 3)
    plt.plot(test * 1 / 240, 1000 * y_true_sysint[:, 1], 'k', linewidth=2)
    plt.plot(test * 1 / 240, 1000 * y_pred_sysint[:, 1], color, linestyle = 'dashed', alpha=0.8, linewidth=2)

    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    #plt.legend([r'$y_{1, true}$', r'$\hat{y}_{1, GRU}$'],
    #           shadow=True, fontsize=10, bbox_to_anchor=(0.94, 0.4, 0.3, 0.2),
    #           frameon=True, loc='center left')
    #plt.ylabel('y [mm]')
    plt.locator_params(axis='x', nbins=15)
    plt.subplot(4, 1, 4)
    plt.plot(test * 1/240,1000 * y_true_sysint[:, 2], 'k', linewidth=2)
    plt.plot(test * 1 / 240, 1000 * y_pred_sysint[:, 2], color, linestyle = 'dashed',
                        alpha = 0.8, linewidth=2)

    #plt.ylim(-15, 15)
    plt.legend([r'$y_{true}$', r'$\hat{y}_{pred}$'],
                shadow=True, fontsize=10, bbox_to_anchor=(0.31, -1, 0.3, 0.2),
                frameon=True, ncol = 2, loc='center left')

    plt.locator_params(axis='x', nbins=15)
    plt.savefig('sysint_hasel.pdf', format='pdf', bbox_inches = 'tight')

    plt.figure()
    plt.plot(X[:, -18:])
    plt.savefig('sensors.png', format = 'png')