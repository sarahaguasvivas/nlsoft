#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.newton_raphson import *
from gym.block_gym import *
from collections import deque

import time, os

model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NNP = NeuralNetworkPredictor(model_file = model_filename, N1= 0, N2 = 1, Nu = 2, \
                                    nd = 3, dd = 2, K = 5, lambd = [1., 2.])

NR_opt = NewtonRaphson(cost = NNP.Cost, d_model = NNP)

Block = BlockGym() # declare my block
Block.reset()
#Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array([ 30, 390,  54,   6,   7,   1, 247,   1,  28, 153,  16 ])

u_optimal_old = np.array([0., -50.])
new_state_new = Block.get_state()

du = [0.0]*2

u_deque = deque()
y_deque = deque()

for _ in range(NNP.nd):
    u_deque.append([0, -50])
for _ in range(NNP.dd):
    y_deque.append(Block.get_state())

elapsed = []
u_optimal_list = []
ym = []
state = []
yn = []

# Working in mm
for n in range(100):
    seconds = time.time()

    signal = np.divide(Block.get_observation(), Block.calibration_max, dtype = np.float64).tolist()

    neural_network_input = np.array(signal + np.array(list(u_deque)).flatten().tolist() +\
                                    np.array(list(y_deque)).flatten().tolist())

    neural_network_input = np.reshape(neural_network_input, (1, -1))

    predicted_states = NNP.predict(neural_network_input).flatten()

    NNP.yn = predicted_states

    NNP.ym = Block.get_target() * 1000

    new_state_old = new_state_new

    u_optimal = np.reshape(NR_opt.optimize(u = u_optimal_old, del_u = du, rtol = 1e-8, \
                    maxit = 6, verbose = False)[0], (-1, 1)).flatten()

    u_optimal_list += [u_optimal.flatten().tolist() ]

    du = np.array(u_optimal_old) - np.array(u_optimal)

    du = du.flatten()

    u_optimal_old = u_optimal

    Block.step(u_optimal.tolist())

    u_deque.pop()
    y_deque.pop()
    u_deque.appendleft(u_optimal.tolist())
    y_deque.appendleft(predicted_states.tolist())

    # need to keep adding stuff. Also need to have deque with previous signals and states
Block.reset()
