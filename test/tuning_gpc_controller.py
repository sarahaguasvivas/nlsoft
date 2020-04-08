#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.soloway_nr import *
from gym.block_gym import *
from collections import deque
from scipy import signal as sig

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time, os

plt.style.use('dark_background')
model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

#NNP = NeuralNetworkPredictor(model_file = model_filename, N1 = 0, N2 = 3, Nu = 1, \
#                                    nd = 3, dd = 3, K = 2, lambd = [1e-4], \
#                                        y0 = [0.02,-0.05, 0.05], \
#                                            u0 = [0.0, -50.], s = 1e-5, b = 5e2, r = 5.)

NNP = NeuralNetworkPredictor(model_file = model_filename, N1 = 0, N2 = 3, Nu = 1, \
                                    nd = 3, dd = 3, K = 2, lambd = [1e-4], \
                                        y0 = [0.02, 0.05, 0.05], \
                                            u0 = [0.0, -50.], s = 1e-5, b = 5e2, r = 5.)

NR_opt = SolowayNR(cost = NNP.Cost, d_model = NNP)
Block = BlockGym(vrpn_ip = "192.168.50.24:3883")

Block.reset()
neutral_point = Block.get_state()
NNP.y0 = neutral_point
verbose = 1

print NNP.model.summary()

def custom_loss(y_true, y_pred):
    pass

Block.stretch() # stretch block for better signal readings before calibrating
#Block.get_signal_calibration() # calibrate signal of the block

Block.calibration_max = np.array([  1, 404,  56,   1,   1,   1, 168,   1,   1,  74,   1 ])

u_optimal_old = np.reshape([0.0, -50.]*NNP.Nu, (-1, 2))

new_state_new = Block.get_state()

del_u = np.zeros(u_optimal_old.shape)

elapsed = []
u_optimal_list = []

ym = []
yn = []

predicted_ = []
actual_= []

u_deque = deque()
y_deque = deque()

for _ in range(NNP.nd):
    u_deque.append([0.0, -50])

for _ in range(NNP.dd):
    y_deque.append(Block.get_state())

n = 0

u_action = np.array([0.0, 0.0])
predicted_states= np.array(new_state_new)

try:
    # working in m
    while True:
        seconds = time.time()

        signal = np.divide(Block.get_observation(), Block.calibration_max, dtype = np.float64).tolist()

        neural_network_input = np.array((np.array(list(u_deque))/150.).flatten().tolist() + \
                                                np.array(list(y_deque)).flatten().tolist() + signal)

        neural_network_input = np.reshape(neural_network_input, (1, -1))

        for _ in range(NNP.K):
            u_deque.pop()
            y_deque.pop()

            u_deque.appendleft(u_action)
            y_deque.appendleft(predicted_states.tolist())

            predicted_states = NNP.predict(neural_network_input).flatten()

        predicted_ += [predicted_states]

        NNP.yn = predicted_states

        omega = 1000.       # frequency
        amplitude = 0.015  # [m]
        initial_angle = np.pi/2.0
        circle_center = neutral_point
        NNP.ym = np.array([circle_center[0] + amplitude * np.cos(2.*np.pi * n / omega + initial_angle), \
                           circle_center[1] + amplitude * np.sin(2.*np.pi * n / omega + initial_angle), \
                            circle_center[2] + amplitude * np.sin(2.*np.pi * n / omega + initial_angle)])

        new_state_old = new_state_new

        u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, \
                                    maxit = 3, rtol = 1e-8, verbose = False)

        u_action = u_optimal[0, :].tolist()

        SCALING1 = 1
        SCALING2 = 1

        u_action[0] = np.clip(u_action[0]*SCALING1, -150, 150)
        u_action[1] = np.clip((u_action[1]+50)*SCALING2 - 50., -200, 300)

    #        u_action[0] = 150*np.cos(omega*n)
    #        u_action[1] = 150*np.sin(omega*n)

        del_u_action = del_u[0, :].tolist()
        actual_st = Block.get_state()
        if verbose == 0:
            print "-----------------------------------------------------"
            print "GPC: Target: ", NNP.ym
            print "GPC: P. State: ", NNP.yn
            print "GPC: Act. State: ", np.array(actual_st)
            print "GPC: u_optimal", u_action
            print "GPC: Cost: ", NNP.compute_cost()

        if verbose == 1:
            print "GPC: u_optimal ", u_action

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
        if verbose == 0:
            print "GPC: elapsed time: ", elapsed[-1]
            print ""

        u_optimal_list+= [u_action]
        actual_ += [(np.array(Block.get_state())).tolist()]
        n += 1

except:
    Block.reset()
    ym = np.reshape(ym, (-1, 3))
    yn = np.reshape(yn, (-1, 3))

    predicted_ = np.reshape(predicted_, (-1, 3))
    actual_ = np.reshape(actual_, (-1, 3))

    u_optimal_list = np.reshape(u_optimal_list, (-1, 2))

    print "Block calibration vector: ", Block.calibration_max
    labels = ['x', 'y', 'z', 'u']
    plt.figure()

    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(ym[:, i], '--w', label = 'target')
        plt.plot(yn[:, i], 'cyan', label = 'predicted state')
        plt.plot(actual_[:, i], label = 'actual state') # only 0 and 2
        plt.ylim([-0.1, 0.09])
        plt.legend()
        plt.ylabel(str(labels[i]) + ' [mm]')
        plt.plot(neutral_point[i], marker = 'h')
    plt.show()

    plt.figure()
    for i in range(2):
        plt.subplot(2, 1, i+1)
        plt.plot(u_optimal_list[:, i], label = 'u' + str(i))
        plt.legend()
        plt.ylabel(str(labels[-1]) + ' [degrees]')
    plt.show()

    neutral_point = np.array(neutral_point).reshape(-1, 3)

    fig = plt.figure()

    ax = Axes3D(fig)
    ax.plot3D(predicted_[:, 0],predicted_[:, 1], predicted_[:, 2], linewidth = 1, alpha = 0.9, label = 'estimated position')
    ax.plot3D(ym[:, 0], ym[:, 1], ym[:, 2], '--w', linewidth = 1, alpha = 1, label = 'target')
    ax.plot3D(actual_[:, 0], actual_[:, 1], actual_[:, 2], \
                        linewidth = 1, alpha = 1, label = 'actual position')
    ax.plot3D(neutral_point[:, 0], neutral_point[:, 1], neutral_point[:, 2], "r" ,marker='h',  label="starting point")
    ax.set_xlim(-0.1, .1)
    ax.set_ylim(-.1, .1)
    ax.set_zlim(-.1, .1)
    plt.legend()
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.title('Target Position vs. Controlled Positions')
    plt.show()

