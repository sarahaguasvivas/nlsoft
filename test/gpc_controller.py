#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.soloway_nr import *
from gym.block_gym import *
from collections import deque
import matplotlib.pyplot as plt
import time, os

plt.style.use('dark_background')
model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NNP = NeuralNetworkPredictor(model_file = model_filename, \
                                    nd = 3, dd = 2, K = 1, lambd = [1., 1., 1.])

NR_opt = SolowayNR(cost = NNP.Cost, d_model = NNP)

Block = BlockGym(vrpn_ip = "192.168.50.24:3883") # declare my block
Block.reset()
#Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array([38, 393, 86, 10, 14, 1, 279, 2, 31, 179, 21 ])

u_optimal_old = [0., -50.]
new_state_new = Block.get_state()

du = np.array([0.0, 0.0])

elapsed = []
u_optimal_list = []
ym = []
yn = []
actual_states= []
u_deque = deque()
y_deque = deque()

for _ in range(NNP.nd):
    u_deque.append([0, -50])

for _ in range(NNP.dd):
    y_deque.append(Block.get_state())

try:
    # working in m
    while True:
        seconds = time.time()

        signal = np.divide(Block.get_observation(), Block.calibration_max, dtype = np.float64).tolist()

        neural_network_input = np.array(signal + np.array(list(u_deque)).flatten().tolist() + \
                                        np.array(list(y_deque)).flatten().tolist())

        neural_network_input = np.reshape(neural_network_input, (1, -1))

        nn_old_in = neural_network_input

        predicted_states = NNP.predict(neural_network_input).flatten()/1000

        NNP.yn = predicted_states
        NNP.ym = Block.get_target()

        print "GPC: Target: ", NNP.ym
        print "GPC: P. State: ", NNP.yn
        print "GPC: Ac State: ", Block.get_state()

        new_state_old = new_state_new

        u_optimal, del_u = NR_opt.optimize(u = u_optimal_old, del_u = [0., 0.], \
                         verbose = False)

        u_optimal = u_optimal[0, :].tolist()
        del_u = del_u[0, :].tolist()

        NNP.update_dynamics(u_optimal_old, del_u, NNP.yn.tolist(), NNP.ym.tolist())

        #u_optimal[0] = np.clip(u_optimal[0], -300, 300)
        #u_optimal[0] = np.clip(u_optimal[0], -300, 300)

        u_optimal_list += [u_optimal]

        du = np.array(u_optimal_old) - np.array(u_optimal)

        du = du.flatten()

        u_optimal_old = u_optimal

        Block.step(action = u_optimal)

        print "GPC: u_optimal", u_optimal

        u_deque.pop()
        y_deque.pop()

        u_deque.appendleft(u_optimal)
        y_deque.appendleft(predicted_states.tolist())

        ym += [NNP.ym]
        yn += [NNP.yn]

        elapsed += [time.time() - seconds]
        print elapsed[-1]
        u_optimal_list+= [u_optimal]
        actual_states += [Block.get_state()]

except Exception as e:
    print str(e)
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
