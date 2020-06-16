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
from target.target import Circle, Pringle, Pringle2, SingleAxisSineWave, SingleAxisSquareWave, Square3D
#from scipy.spatial.transform import Rotation as R

model_filename = str(os.environ['HOME']) + '/gpc_controller/test/sys_id.hdf5'

NUM_EXPERIMENTS = 1
NUM_TIMESTEPS = 1000

input_scale = [1., 1.]
verbose = 1

NNP = NeuralNetworkPredictor(model_file = model_filename,
                    N1 = 0, N2 = 4, Nu = 3, nd = 2, dd = 2, K = 5,
                    Q = 1e5*np.array([[1.,  0.],
                                 [0.,  1.]]),
                    Lambda = np.array([[1., 0.],
                                       [0., 1.]]),
                    states_to_control = [0, 1, 1],
                        x0 = [0.0, 0.0, 0.0],
                        u0 = [0.0, 0.0], s = 0., b = 1., r = 4.)

NR_opt, Block = SolowayNR(d_model = NNP), BlockGym(vrpn_ip = "192.168.50.24:3883")

log = Logger()
Block.reset()

Block.step([0., -50.])

neutral_point = Block.get_state()

#NNP.x0=neutral_point

print "neutral_point: ", neutral_point

target = Pringle2(wavelength = 100, amplitude = 10./1000., center = neutral_point)

Block.calibration_max = np.array([ 80, 309, 113,   1,   4,   1, 100,   1,   1,  33,   1])
#Block.get_signal_calibration()

u_optimal_old = np.reshape(NNP.u0*NNP.Nu, (-1, 2))
new_state_new = copy.copy(neutral_point)
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,
         'num_experiments' : NUM_EXPERIMENTS,
         'num_timesteps': NUM_TIMESTEPS}})

u_deque, y_deque = first_load_deques(NNP.x0, NNP.u0, NNP.nd, NNP.dd)
u_action, predicted_states = np.array(NNP.u0), np.array(new_state_new)

try:
    for e in range(NUM_EXPERIMENTS):
        log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [], \
                'ym' : [], 'elapsed' : [], 'u' : []}})

        Block.reset()
        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.x0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(new_state_new)

        for n in range(NUM_TIMESTEPS):
            seconds = time.time()
            signal = np.divide(Block.get_observation(), Block.calibration_max, \
                                dtype = np.float64).tolist()

            NNP.yn = []

            ydeq = copy.copy(y_deque)
            for k in range(NNP.K):
                neural_network_input = np.array((np.array(list(u_deque
                                                    ))).flatten().tolist() + \
                                                    np.array(list(ydeq)
                                                        ).flatten().tolist() + \
                                                        signal).reshape(1, -1)


                predicted_states = NNP.predict(neural_network_input).flatten()
                NNP.yn += [NNP.C.dot(predicted_states).tolist()]
                ydeq = roll_deque(ydeq, predicted_states.tolist())

            y_deque = roll_deque(y_deque, predicted_states.tolist())

            NNP.last_model_input = neural_network_input

            Target = target.spin(n, 0, NNP.K, 3, neutral_point)

            NNP.ym = NNP.C.dot(Target.T).reshape(NNP.ny, -1).T.tolist()

            new_state_old = new_state_new

            u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, delu = del_u, \
                                        maxit = 1, rtol = 1e-4, verbose = True)

            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            u_action[0] = np.clip(input_scale[0]*np.rad2deg(u_action[0]),-100, 80)
            u_action[1] = np.clip(input_scale[1]*np.rad2deg(u_action[1]),-100, 60)

            #u_action[0] = np.clip(100*np.cos(2.*np.pi/100.*n), -100., 80.)
            #u_action[1] = np.clip(100*np.sin(2.*np.pi/100.*n), -100., 60.)

            Block.step(action = u_action)

            NNP.update_dynamics(np.deg2rad(u_action).tolist(), del_u_action, \
                                predicted_states.tolist(), Target[0, :].tolist())

            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, np.deg2rad(u_action).tolist())
            y_deque = roll_deque(y_deque, predicted_states.tolist())

            if verbose == 0:
                log.verbose(actual = np.array(Block.get_state()).tolist(),
                        yn = predicted_states, ym = Target[0, :], \
                            elapsed = time.time()-seconds, u = u_action)
            if verbose == 1:
                log.verbose(u_action = u_action, elapsed = time.time() - seconds)

            log.log({str(e) : {'actual' : np.array(Block.get_state()).tolist(), \
                            'yn' : predicted_states.tolist(), \
                            'ym' : Target[0, :].tolist(),\
                            'elapsed' : time.time() - seconds,\
                            'cost' : NNP.cost.compute_cost(), \
                            'u' : [u_action], \
                            'signal' : signal}})

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

