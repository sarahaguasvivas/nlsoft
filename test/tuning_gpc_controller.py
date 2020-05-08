#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.soloway_nr import *
from gym.block_gym import *
from collections import deque
from scipy import signal as sig
import time, os
import copy
from logger.logger import Logger
from utilities.util import *

model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NUM_EXPERIMENTS = 2
NUM_TIMESTEPS = 3

NNP = NeuralNetworkPredictor(model_file = model_filename, N1 = 0, \
                N2 = 3, Nu = 2, nd = 3, dd = 3, K = 5, \
                    lambd = np.array([[1e2, 1e3], [1e-5, 1e-4]]), \
                        y0 = [0.02, -0.05, 0.05], \
                            u0 = [0.0, -50.0], s = 1e-10, b= 1e-4, r = 5e-1)

NR_opt, Block = SolowayNR(cost = NNP.Cost, d_model = NNP), BlockGym(vrpn_ip = "192.168.50.24:3883")

log = Logger()

Block.reset()
neutral_point = Block.get_state()

#Block.get_signal_calibration() # calibrate signal of the block
Block.calibration_max = np.array([1, 335, 120,1,1, 1, 166,9,1,50,1])

u_optimal_old = np.reshape(NNP.u0*NNP.Nu, (-1, 2))

new_state_new = copy.copy(neutral_point)
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,\
         'num_experiments' : NUM_EXPERIMENTS, \
         'num_timesteps': NUM_TIMESTEPS}})

print log.log_dictionary


u_deque, y_deque = first_load_deques(neutral_point, NNP.u0, NNP.nd, NNP.dd)

u_action, predicted_states = np.array(NNP.u0), np.array(new_state_new)

for e in range(NUM_EXPERIMENTS):
    log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [], \
            'ym' : [], 'elapsed' : [], 'u' : []}})
    for n in range(NUM_TIMESTEPS):
        seconds = time.time()

        signal = np.divide(Block.get_observation(), Block.calibration_max, \
                            dtype = np.float64).tolist()

        NNP.yn = []
        for ii in range(NNP.K):
            neural_network_input = np.array((np.array(list(u_deque)) \
                                                /100.).flatten().tolist() + \
                                                np.array(list(y_deque)
                                                    ).flatten().tolist() + \
                                                    signal).reshape(1, -1)
            predicted_states = NNP.predict(neural_network_input).flatten()
            NNP.yn += [predicted_states]

            y_deque = roll_deque(y_deque, predicted_states.tolist())


        omega, amplitude, initial_angle, circle_center = 500,  0.025, 0.0, neutral_point

        NNP.ym = np.empty([NNP.N2 - NNP.N1, 3])
        for jjj in range(NNP.N1, NNP.N2):
            NNP.ym[jjj, :] = [circle_center[0],  \
                    circle_center[1] + amplitude * np.cos(2*np.pi * (n+jjj) / omega),
                    circle_center[2] + amplitude/2. * np.sin(2*np.pi * (n+jjj) / omega)]

        new_state_old = new_state_new

        u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, \
                                    maxit = 8, rtol = 1e-5, verbose = True)

        u_action = u_optimal[0, :].tolist()
        del_u_action = del_u[0, :].tolist()

        u_action[0] = np.clip(u_action[0], -100, 100)
        u_action[1] = np.clip(u_action[1], -100, 60)

        Block.step(action = u_action)

        NNP.update_dynamics(u_action, del_u_action, predicted_states.tolist(), \
                                    NNP.ym[0, :].tolist())
        u_optimal_old = u_optimal

        u_deque = roll_deque(u_deque, u_action)
        y_deque = roll_deque(y_deque, predicted_states.tolist())

        log.verbose(actual = np.array(Block.get_state()).tolist(),
                yn = predicted_states, ym = NNP.ym[0, :], \
                        elapsed = time.time()-seconds, u = u_action)

        log.log({str(e) : {'actual' : np.array(Block.get_state()).tolist(), \
                        'yn' : predicted_states, \
                        'ym' : NNP.ym[0, :],\
                        'elapsed' : time.time() - seconds,\
                        'u' : [u_action]}})

    u_optimal_old = np.reshape(NNP.u0*NNP.Nu, (-1, 2))
    Block.reset()

log.plot_log()

