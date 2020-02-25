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

NNP = NeuralNetworkPredictor(model_file = model_filename, N1 = 0, N2 = 3, Nu = 2, \
                                    nd = 3, dd = 3, K = 1, lambd = [1e0]*2, \
                                    y0 = [0.03,-0.04, 0.06], \
                                            u0 = [0.0, -50.], s = 1e-10, b = 1., r = 1.)

NR_opt = SolowayNR(cost = NNP.Cost, d_model = NNP)

Block = BlockGym(vrpn_ip = "192.168.50.24:3883") # declare my block
Block.reset()

neutral_point = Block.get_state()

NNP.y0 = neutral_point

#Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array( [ 24, 219,  69,  13,   9,  16, 243,   1,  26, 102,  16 ])

u_optimal_old = [0.0, -50.]
new_state_new = Block.get_state()

del_u = [0.0, 0.0]

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

        predicted_states = NNP.predict(neural_network_input).flatten() / 1000.

        NNP.yn = predicted_states

        #NNP.ym = np.array([neutral_point[0]+ 0.1*sig.square(np.pi * n / 20.) - 0.1, neutral_point[1], \
        #                                    neutral_point[2]])

        NNP.ym = np.array([neutral_point[0] + 0.01, neutral_point[1], neutral_point[2]])
        new_state_old = new_state_new

        u_optimal, del_u = NR_opt.optimize(u = u_optimal_old, del_u = del_u, \
                                            maxit = 1, rtol = 1e-8, verbose = False)

        u_optimal = u_optimal[0, :].tolist()
        del_u = del_u[0, :].tolist()

        u_optimal[0] = np.clip(u_optimal[0], -300, 150)
        u_optimal[1] = np.clip(u_optimal[1], -300, 150)

        print "-----------------------------------------------------"
        print "GPC: Target: ", NNP.ym
        print "GPC: P. State: ", NNP.yn
        print "GPC: Act. State: ", Block.get_state()
        print "GPC: u_optimal", u_optimal
        print "GPC: Cost: ", NNP.compute_cost()

        NNP.update_dynamics(u_optimal_old, del_u, NNP.yn.tolist(), NNP.ym.tolist())

        u_optimal_old = u_optimal

        Block.step(action = u_optimal)

        u_deque.pop()
        y_deque.pop()

        u_deque.appendleft(u_optimal)
        y_deque.appendleft(predicted_states.tolist())

        ym += [NNP.ym]
        yn += [NNP.yn]

        elapsed += [time.time() - seconds]

        print "GPC: elapsed time: ", elapsed[-1]
        print ""

        u_optimal_list+= [u_optimal]
        actual_states += [Block.get_state()]
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
        plt.plot(ym[:, i]*1000, '--w', label = 'target')
        plt.plot(yn[:, i]*1000, 'cyan', label = 'predicted state')
        plt.plot(actual_states[:, i]*1000, label = 'actual state')
        plt.legend()
        plt.ylabel(str(labels[i]) + ' [mm]')
    plt.show()
