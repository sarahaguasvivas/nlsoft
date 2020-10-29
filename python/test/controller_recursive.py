import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'gpc_controller/python'))
from controller.soloway_nr import *
from block_gym.block_gym import *
import time, os
from logger.logger import Logger
from utilities.util import *
from test.training_recnn import thousand_mse
from target.target import FigureEight
import numpy as np

model_filename = str(os.environ['HOME']) + '/gpc_controller/python/test/sys_id.hdf5'

NUM_EXPERIMENTS = 1
NUM_TIMESTEPS = 3000

verbose = 1

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 1, Nu = 1,
                                      nd = 3, dd = 3, K = 5,
                                      Q = np.array([[1e-3, 0., 0],
                                                    [0., 1e3, 0],
                                                    [0., 0., 1e3]]),
                                      Lambda = np.array([[1., 0.],
                                                         [0., 1.]]),
                                      s = 1e-20, b = 1., r = 4.,
                                      states_to_control = [1, 1, 1],
                                      y0= [0.0, 0.0, 0.0],
                                      u0 = [np.deg2rad(-50.), np.deg2rad(-50.)],
                                      step_size = 1e-3)

NR_opt, Block = SolowayNR(d_model = NNP), BlockGym(vrpn_ip = "192.168.50.24:3883")

log = Logger()
Block.reset()
time.sleep(1)
neutral_point = Block.get_state()

NNP.y0 = neutral_point
NNP.u0 = [np.deg2rad(-50.), np.deg2rad(-50.)]

target = FigureEight(a = 20. / 1000., b = 10./1000., wavelength= 300.,
                     center = neutral_point)

#Block.get_signal_calibration()
Block.calibration_max = np.array([613., 134., 104., 174, 86., 146., 183., 1., 2., 1., 60.])

u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,
         'num_experiments' : NUM_EXPERIMENTS,
         'num_timesteps': NUM_TIMESTEPS}})

u_deque = deque()
y_deque = deque()

try:
    for e in range(NUM_EXPERIMENTS):
        log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [],
                'ym' : [], 'elapsed' : [], 'u' : []}})

        Block.reset()
        time.sleep(1)
        NNP.y0 = Block.get_state()

        log.log_dictionary['metadata']['neutral_point'] = NNP.y0

        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(NNP.y0)

        target.center = NNP.y0
        for n in range(NUM_TIMESTEPS):
            seconds = time.time()
            signal = np.divide(Block.get_observation(), Block.calibration_max,
                                dtype = np.float64).tolist()

            NNP.yn = []
            ydeq = y_deque.copy()
            for k in range(NNP.K):
                neural_network_input = np.array((np.array(list(u_deque))/np.pi).flatten().tolist() + \
                                np.array(list(ydeq)).flatten().tolist() + signal).reshape(1, -1)
                predicted_states = NNP.predict(neural_network_input).flatten()
                NNP.yn += [(NNP.C @ predicted_states).tolist()]
                ydeq = roll_deque(ydeq, predicted_states.tolist())

            y_deque = roll_deque(y_deque, predicted_states.tolist())

            NNP.last_model_input = neural_network_input

            target_path = target.spin(n, 0, NNP.K, 3)

            NNP.ym = (NNP.C @ target_path.T).reshape(NNP.ny, -1).T.tolist()

            u_optimal, del_u,  _ = NR_opt.optimize(u = u_optimal_old, delu = del_u,
                                        maxit = 1, rtol = 1e-4, verbose = True)

            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            u_action[0] = np.clip(np.rad2deg(u_action[0]), -100., 50.)
            u_action[1] = np.clip(np.rad2deg(u_action[1]), -100., 50.)

            #u_action[0] = (1.+np.cos(2.* np.pi / 1000. * n))/2. * 150. - 100.
            #u_action[1] = (1.+np.sin(2.* np.pi / 1000. * n))/2. * 150. - 100.

            Block.step(action = u_action)

            NNP.update_dynamics(u_optimal[0, :].tolist(), del_u_action,
                        predicted_states.tolist(), target_path[0, :].tolist())

            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, u_optimal[0, :].tolist())
            elapsed = time.time()-seconds
            actual_ = np.array(Block.get_state()).tolist()
            if verbose == 0:
                log.verbose( actual = actual_,
                            yn = predicted_states, ym = target_path[0, :],
                            elapsed = elapsed, u = u_action)
            if verbose == 1:
                log.verbose(u_action = u_action, elapsed = time.time() - seconds)

            log.log({str(e) : {'actual' : actual_,
                            'yn' : predicted_states.tolist(),
                            'ym' : target_path[0, :].tolist(),
                            'elapsed' : elapsed,
                            'u' : [u_action],
                            'signal' : signal}})

        u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))
        Block.reset()
    log.plot_log()
    log.save_log()

except Exception as e:
    print(str(e))
    print("Closing all connections!")
    Block.reset()
    Block.motors.close_connection()

