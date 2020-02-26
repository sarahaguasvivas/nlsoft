#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.soloway_nr import *
from gym.block_gym import *
from collections import deque
from scipy import signal as sig

import matplotlib.pyplot as plt
import time, os

plt.style.use('dark_background')
model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NNP = NeuralNetworkPredictor(model_file = model_filename, N1 = 1, N2 = 3, Nu = 2, \
                                    nd = 3, dd = 3, K = 0, lambd = [.1, 1e2], \
                                        y0 = [0.02,-0.05, 0.05], \
                                            u0 = [0.0, -50.], s = 1e-10, b = 1e-2, r = 1e-2)

NR_opt = SolowayNR(cost = NNP.Cost, d_model = NNP)

Block = BlockGym(vrpn_ip = "192.168.50.24:3883") # declare my block
Block.reset()

neutral_point = Block.get_state()

NNP.y0 = neutral_point

def custom_loss(y_true, y_pred):
    pass

#Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array([24, 219,  69,  13,   9,  16, 243, 1, 26, 102, 16])

u_optimal_old = np.reshape([0.0, -50.]*2, (-1, 2))

new_state_new = Block.get_state()

del_u = np.zeros(u_optimal_old.shape)

elapsed = []
u_optimal_list = []

ym = []
yn = []
actual_states= []

u_deque = deque()
y_deque = deque()

for _ in range(NNP.nd):
    u_deque.append([0.0, -50])

for _ in range(NNP.dd):
    y_deque.append(Block.get_state())

n = 0
try:
# working in m
    while True:
        seconds = time.time()

        signal = np.divide(Block.get_observation(), Block.calibration_max, dtype = np.float64).tolist()

        neural_network_input = np.array(np.array(list(u_deque)).flatten().tolist() + \
                                                np.array(list(y_deque)).flatten().tolist() + signal)

        neural_network_input = np.reshape(neural_network_input, (1, -1))

        predicted_states = NNP.predict(neural_network_input).flatten()

        NNP.yn = predicted_states

        NNP.ym = 1000*np.array([neutral_point[0], neutral_point[1] , \
                                            neutral_point[2] + 0.0325*sig.square(np.pi * n / 10.) - 0.0325/2.0 ])

        new_state_old = new_state_new

        u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, \
                                            maxit = 10, rtol = 1e-8, verbose = True)

        u_action = u_optimal[0, :].tolist()

        u_action[0] = np.clip(u_action[0], -300, 150)
        u_action[1] = np.clip(u_action[1], -300, 150)

        del_u_action = del_u[0, :].tolist()

        print "-----------------------------------------------------"
        print "GPC: Target: ", NNP.ym
        print "GPC: P. State: ", NNP.yn
        print "GPC: Act. State: ", 1000*np.array(Block.get_state())
        print "GPC: u_optimal", u_action
        print "GPC: Cost: ", NNP.compute_cost()

        NNP.update_dynamics(u_action, del_u_action, NNP.yn.tolist(), NNP.ym.tolist())

        u_optimal_old = u_optimal

        Block.step(action = u_action)

        u_deque.pop()
        y_deque.pop()

        u_deque.appendleft(u_action)
        y_deque.appendleft(predicted_states.tolist())

        ym += [NNP.ym]
        yn += [NNP.yn]

        elapsed += [time.time() - seconds]

        print "GPC: elapsed time: ", elapsed[-1]
        print ""

        u_optimal_list+= [u_action]
        actual_states += [(np.array(Block.get_state())*1000).tolist()]
        n += 1

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
