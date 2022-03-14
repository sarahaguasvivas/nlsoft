import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'nlsoft', 'python'))
from python.controller.soloway_nr import *
from python.block_gym.block_gym_vani import *
from python.controller.recursive_model import *
from python.logger.logger import Logger
from python.utilities.util import *
from python.target.vanis_target_swirl import VanisSwirl
import numpy as np

model_filename = str(os.environ['HOME']) + '/github_repos/nlsoft/python/training/forward_kinematics_jan_10_2022.hdf5'
sensor_signal_model_filename = str(os.environ['HOME']) + '/github_repos/nlsoft/python/models/model_signals_may_25_2021.hdf5'
target_filename = str(os.environ['HOME']) + '/github_repos/nlsoft/python/models/ref_data.mat'
NUM_EXPERIMENTS = 1
WAVELENGTH = 5
NUM_TIMESTEPS = (1130 - 5) * WAVELENGTH
FILENAME = 'gru_log_output_vani_swirl.json'
verbose = 1
savelog = True

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 3, Nu= 1,
                                      nd = 2, dd = 2, K = 3,
                                      Q = np.array([[1e2, 0., 0.],
                                                    [0., 3e2, 0.],
                                                    [0., 0., 5e2]]),
                                      Lambda = 5e-2*np.array([[1., 0, 0, 0, 0, 0],
                                                              [0, 1., 0, 0, 0, 0],
                                                              [0, 0, 1., 0, 0, 0],
                                                              [0, 0, 0, 1., 0, 0],
                                                              [0, 0, 0, 0, 1., 0],
                                                              [0, 0, 0, 0, 0, 1.]]),
                                      s = 1e-20, b = 1e5, r = 4e3,
                                      states_to_control = [1, 1, 1],
                                      y0= [0.0, 0.0, 0.0],
                                      u0 = [-0.5]*6,
                                      step_size = 1./240.)

NR_opt, Block = SolowayNR(d_model = NNP), BlockGymVani(signal_simulator_model=sensor_signal_model_filename)

log = Logger()
Block.step(NNP.u0)
#neutral_point = [ 0.01290038, -0.00257767, 0.05512653]
neutral_point = [0.]*3

NNP.y0 = neutral_point
motors_calibration = [800]*6
Block.calibration_max = [1e4]*18
target = VanisSwirl(source_filename = target_filename, neutral_point= [0.]*3,
                            wavelength = WAVELENGTH)

u_optimal_old = np.reshape([NNP.u0] * NNP.nu, (-1, 6))
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,
         'num_experiments' : NUM_EXPERIMENTS,
         'num_timesteps': NUM_TIMESTEPS,
         'ym' : []}})

u_deque = deque()
y_deque = deque()

try:
    for e in range(NUM_EXPERIMENTS):
        log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [],
                'elapsed' : [], 'u' : []}})
        print(e)
        NNP.y0 = [0.]*3

        log.log_dictionary['metadata']['neutral_point'] = NNP.y0

        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(NNP.y0)

        target.center = NNP.y0

        log.log({str(e) : {'predicted' : NNP.y0, 'actual' : NNP.y0,
                           'yn' : NNP.y0, 'elapsed' : 0.0, 'u' : [NNP.u0]}})
        if (e == 0):
            log.log({'metadata': {'ym': neutral_point, 'num_signals' : 18, 'm' : NNP.m,
                                                                    'n' : NNP.nx}})
        for n in range(NUM_TIMESTEPS):
            seconds = time.time()
            signal = [0.0] * 18 #np.array(Block.get_observation(u_optimal_old)).tolist()
            NNP.yn = []
            ydeq = y_deque.copy()
            for k in range(NNP.K):
                neural_network_input = np.array((np.array(list(u_deque))).flatten().tolist() + \
                                np.array(list(ydeq)).flatten().tolist() + signal).reshape(1, 1, 36)
                predicted_states = NNP.predict(neural_network_input).flatten()
                NNP.yn += [(NNP.C @ predicted_states).tolist()]
                ydeq = roll_deque(ydeq, predicted_states.tolist())
            y_deque = roll_deque(y_deque, predicted_states.tolist())
            NNP.last_model_input = neural_network_input
            target_path = target.spin(n, 0, NNP.K, NNP.nx)
            NNP.ym = (NNP.C @ target_path.T).reshape(NNP.ny, -1).T.tolist()

            u_optimal, del_u, _ = NR_opt.optimize(u = u_optimal_old, delu = del_u,
                                        maxit = 1, rtol = 1e-4, verbose = False)

            u_optimal = np.clip(u_optimal, -0.5, 0.5)

            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            NNP.update_dynamics(u_optimal[0, :].tolist(), del_u_action,
                        predicted_states.tolist(), target_path[0, :].tolist())

            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, u_optimal[0, :].tolist())
            elapsed = time.time()-seconds
            actual_ = np.array(predicted_states.tolist()).tolist()

            if verbose is not None:
                if verbose == 0:
                    log.verbose(yn = predicted_states, ym = target_path[0, :],
                                elapsed = elapsed, u = u_action)
                if verbose == 1:
                    log.verbose(u_action = u_action, elapsed = time.time() - seconds)

            log.log({str(e) : { 'actual' : actual_,
                            'yn' : predicted_states.tolist(),
                            'elapsed' : elapsed,
                            'u' : [u_action],
                            'signal' : signal}})
            if e==0:
                log.log({'metadata' : {'ym' : target_path[0, :].tolist()}})

        u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, NNP.m))
        Block.reset()

    if savelog:
        log.save_log(filename=FILENAME)
    log.plot_log()


except Exception as e1:
    print(str(e1))
    print("Closing all connections!")
    Block.reset()

