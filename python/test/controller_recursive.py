import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'gpc_controller/python'))
from controller.soloway_nr import *
from block_gym.block_gym import *
import time, os
import copy
from logger.logger import Logger
from utilities.util import *
from target.target import Pringle2, FigureEight
import numpy as np

model_filename = str(os.environ['HOME']) + '/gpc_controller/python/test/sys_id.hdf5'

NUM_EXPERIMENTS = 1
NUM_TIMESTEPS = 500

input_scale = [1., 1.]
shift = [0., 0.]
verbose = 1

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 2, Nu = 1, nd = 5,
                                      dd = 5, K = 2, Q = np.array([[1., 0., 0.],
                                                                   [0., 100., 0.],
                                                                   [0., 0., 100.]]),
                                      Lambda = np.array([[1e-1, 0.],
                                                         [0., 1e-1]]),
                                      states_to_control = [1, 1, 1],
                                      x0 = [0.0, 0.0, 0.0],
                                      u0 = [np.deg2rad(-50.)]*2)
NR_opt, Block = SolowayNR(d_model = NNP), BlockGym(vrpn_ip = "192.168.50.24:3883")

log = Logger()
Block.reset()

neutral_point = Block.get_state()

NNP.x0 = neutral_point
NNP.u0 = [np.deg2rad(Block.motors._zero1),
                        np.deg2rad(Block.motors._zero2)]

target = FigureEight(a=20. / 1000., wavelength= 100.,
                     center = neutral_point)

#Block.get_signal_calibration()
Block.calibration_max = np.array([ 45., 1, 90.,   1,   1,   122., 173.,   1,   1,  1,  17.])

u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))

del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,
         'num_experiments' : NUM_EXPERIMENTS,
         'num_timesteps': NUM_TIMESTEPS}})

u_deque = deque()
y_deque = deque()

try:
    for e in range(NUM_EXPERIMENTS):
        log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [], \
                'ym' : [], 'elapsed' : [], 'u' : []}})

        Block.reset()

        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.x0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(neutral_point)

        for n in range(NUM_TIMESTEPS):
            seconds = time.time()
            signal = np.divide(Block.get_observation(), Block.calibration_max,
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

            Target = target.spin(n, 0, NNP.K, 3)

            NNP.ym = NNP.C.dot(Target.T).reshape(NNP.ny, -1).T.tolist()

            u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, delu = del_u,
                                        maxit = 1, rtol = 1e-4, verbose = True)

            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            u_action[0] = np.clip(input_scale[0]*(np.rad2deg(u_action[0]) + shift[0]),-100, 50)
            u_action[1] = np.clip(input_scale[1]*(np.rad2deg(u_action[1] + shift[1])),-100, 50)

            Block.step(action = u_action)

            NNP.update_dynamics(u_optimal[0, :].tolist(), del_u_action,
                                predicted_states.tolist(), Target[0, :].tolist())

            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, u_optimal[0, :].tolist())

            if verbose == 0:
                log.verbose(actual = np.array(Block.get_state()).tolist(),
                        yn = predicted_states, ym = Target[0, :],
                            elapsed = time.time()-seconds, u = u_action,
                           # cost = NNP.cost.compute_cost(del_u)
                            )
            if verbose == 1:
                log.verbose(u_action = u_action, elapsed = time.time() - seconds)

            log.log({str(e) : {'actual' : np.array(Block.get_state()).tolist(),
                            'yn' : predicted_states.tolist(),
                            'ym' : Target[0, :].tolist(),
                            'elapsed' : time.time() - seconds,
                       #     'cost' : NNP.cost.compute_cost(del_u),
                            'u' : [u_action],
                            'signal' : signal}})

        u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))
        Block.reset()
    log.plot_log()
    log.save_log()

except Exception as e:
    import traceback, sys
    print(str(e))
    print("Closing all connections!")
    Block.reset()
    Block.motors.close_connection()
    traceback.print_exc(file = sys.stdout)

