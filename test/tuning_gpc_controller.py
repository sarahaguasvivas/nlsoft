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
from target.target import Circle, Pringle, SingleAxisSineWave, SingleAxisSquareWave, Square3D

model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NUM_EXPERIMENTS = 1
NUM_TIMESTEPS = 1000

SCALE0 = 100.
SCALE1 = 50.

verbose = 0

#neutral point
NNP = NeuralNetworkPredictor(model_file = model_filename,
                    N1 = 0, N2 = 2, Nu = 1, nd = 2, dd = 2, K = 5,
                    Q = 5e-1*np.array([[0.7, 0.],
                                       [0, 0.5e1]]),
                    Lambda = np.array([[5.5e-10]]),
                        y0 = [0.0, 0.0, 0.0],
                        u0 = [0.0, 0.0], s = 1e-5, b = 1., r = .4)

NR_opt, Block = SolowayNR(cost = NNP.cost, d_model = NNP), \
                        BlockGym(vrpn_ip = "192.168.50.24:3883")
log = Logger()
Block.reset()
Block.step([0.0, 0.])
neutral_point = Block.get_state()

#NNP.y0 = neutral_point

target = Pringle(wavelength = 100, amplitude = 15./1000., center = neutral_point)

#Block.get_signal_calibration() # calibrate signal of the block
Block.calibration_max = np.array([ 1, 288, 110,   1,   1,   1, 104,   1,   1,  38,   1 ])
#  1 288 110   1   1   1 104   1   1  38   1
u_optimal_old = np.reshape(NNP.u0*NNP.Nu, (-1, 2))
new_state_new = copy.copy(neutral_point)
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,\
         'num_experiments' : NUM_EXPERIMENTS, \
         'num_timesteps': NUM_TIMESTEPS}})

u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
u_action, predicted_states = np.array(NNP.u0), np.array(new_state_new)

try:
    for e in range(NUM_EXPERIMENTS):
        log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [], \
                'ym' : [], 'elapsed' : [], 'u' : []}})

        Block.reset()
        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(new_state_new)

        for n in range(NUM_TIMESTEPS):

            seconds = time.time()
            signal = np.divide(Block.get_observation(), Block.calibration_max, \
                                dtype = np.float64).tolist()
            NNP.yn = []
            for k in range(NNP.K):
                neural_network_input = np.array((np.array(list(u_deque)) \
                                                    /100.).flatten().tolist() + \
                                                    np.array(list(y_deque)
                                                        ).flatten().tolist() + \
                                                        signal).reshape(1, -1)
                predicted_states = NNP.predict(neural_network_input).flatten()
                NNP.yn += [predicted_states]
                y_deque = roll_deque(y_deque, predicted_states.tolist())

            NNP.last_model_input = neural_network_input
            NNP.ym = target.spin(n, 0, NNP.K, 3, predicted_states.tolist())
            new_state_old = new_state_new
            u_optimal, del_u,  _ = NR_opt.optimize(u = np.array(u_optimal_old), \
                                        maxit = 1, rtol = 1e-4, verbose = True)
            print 'del_u',  del_u
            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            # clipping for safety; with good tuning this is almost never needed:
            u_action[0] = np.clip(SCALE0*u_action[0],-100, 80)
            u_action[1] = np.clip(SCALE1*u_action[1],-100, 60)

            Block.step(action = u_action)
            NNP.update_dynamics(u_optimal[0, :].tolist(), del_u_action, \
                                predicted_states.tolist(), NNP.ym[0, :].tolist())
            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, u_action)
            y_deque = roll_deque(y_deque, predicted_states.tolist())

            if verbose == 0:
                log.verbose(actual = np.array(Block.get_state()).tolist(),
                        yn = predicted_states, ym = NNP.ym[0, :], \
                            elapsed = time.time()-seconds, u = u_action)
            #if verbose == 1:
            #    log.verbose(u = u_action, cost = NNP.cost.compute_cost())

            log.log({str(e) : {'actual' : np.array(Block.get_state()).tolist(), \
                            'yn' : predicted_states.tolist(), \
                            'ym' : NNP.ym[0, :].tolist(),\
                            'elapsed' : time.time() - seconds,\
                            'u' : [u_action], \
                            'signal' : signal}})
            #'cost : ': NNP.cost.compute_cost(),
        u_optimal_old = np.reshape(NNP.u0*NNP.Nu, (-1, 2))
        Block.reset()
    log.plot_log()
    log.save_log()
except Exception as e:
    import traceback, sys
    print str(e)
    print "Closing all connections!"
    Block.reset()
    Block.motors.close_connection()
    traceback.print_exc(file = sys.stdout)

