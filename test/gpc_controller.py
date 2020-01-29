#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.newton_raphson import *
from gym.block_gym import *
from collections import deque
import matplotlib.pyplot as plt
import time, os

plt.style.use('dark_background')
model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NNP = NeuralNetworkPredictor(model_file = model_filename, N1= 0, N2 = 3, Nu = 2, \
                                    nd = 3, dd = 2, K = 2, lambd = [1., 1.])

NR_opt = NewtonRaphson(cost = NNP.Cost, d_model = NNP)

Block = BlockGym() # declare my block
Block.reset()
#Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array([ 25, 373,  60,   4,   7,   1, 248,   1,  25, 148,  14 ])

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
yn = []
actual_states= []

try:
    # working in mm
    while True:
        seconds = time.time()

        signal = np.divide(Block.get_observation(), Block.calibration_max, dtype = np.float64).tolist()

        neural_network_input = np.array(signal + np.array(list(u_deque)).flatten().tolist() + \
                                        np.array(list(y_deque)).flatten().tolist())

        neural_network_input = np.reshape(neural_network_input, (1, -1))

        nn_old_in = neural_network_input

        predicted_states = NNP.predict(neural_network_input).flatten() / 1000

        NNP.yn = predicted_states * 1000

        NNP.ym = Block.get_target() * 1000

        print "GPC: Target: ", NNP.ym
        print "GPC: P. State: ", NNP.yn
        print "GPC: Ac State: ", Block.get_state()

        new_state_old = new_state_new

        u_optimal = np.reshape(NR_opt.optimize(u = u_optimal_old, del_u = du, rtol = 1e-8, \
                        maxit = 5, verbose = False)[0], (-1, 1)).flatten()

        #u_optimal[0] = np.clip(u_optimal[0], -300, 300)
        #u_optimal[0] = np.clip(u_optimal[0], -300, 300)

        u_optimal_list += [u_optimal.flatten().tolist() ]

        du = np.array(u_optimal_old) - np.array(u_optimal)

        du = du.flatten()

        u_optimal_old = u_optimal

        Block.step(action = u_optimal.tolist())

        print "GPC: u_optimal", u_optimal.tolist()

        u_deque.pop()
        y_deque.pop()

        u_deque.appendleft(u_optimal.tolist())
        y_deque.appendleft(predicted_states.tolist())

        ym += [NNP.ym]
        yn += [NNP.yn]

        elapsed += [time.time() - seconds]
        print elapsed[-1]
        u_optimal_list+= [u_optimal]
        actual_states += [Block.get_state()*1000]

except:
    Block.reset()
    ym = np.reshape(ym, (-1, 3))
    yn = np.reshape(yn, (-1, 3))
    actual_states = np.reshape(actual_states, (-1, 3))
    u_optimal_list = np.reshape(u_optimal_list, (-1, 2))

    labels = ['x', 'y', 'z']
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(ym[:, i], '--w', label = 'target')
        plt.plot(yn[:, i], 'cyan', label = 'predicted state')
        plt.plot(actual_states[:, i], label = 'actual state')
        plt.legend()
        plt.ylabel(str(labels[i]) + ' [mm]')
    plt.show()
