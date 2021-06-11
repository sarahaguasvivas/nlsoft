import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'nlsoft', 'python'))
from python.controller.soloway_nr import *
from python.block_gym.block_gym import *
import time
from python.controller.recursive_model import *
from python.logger.logger import Logger
from python.utilities.util import *
from python.test.training_recnn import thousand_mse
from python.target.target import FigureEight, FixedTarget, Pringle, Diagonal
import numpy as np

model_filename = str(os.environ['HOME']) + '/nlsoft/python/test/sys_id_GRU.hdf5'
NUM_EXPERIMENTS = 1
NUM_TIMESTEPS = 3000
FILENAME = 'gru_log_output_figure8_demo.json'
verbose = 1
savelog = True

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 1, Nu = 1,
                                      nd = 2, dd = 2, K = 3,
                                      Q = np.array([[1e3, 0., 0],
                                                    [0., 1e4, 0.],
                                                    [0., 0., 1e3]]),
                                      Lambda = np.array([[1., 0.],
                                                         [0., 1.]]),
                                      s = 1e-20, b = 1e-5, r = 4e3,
                                      states_to_control = [1, 1, 1],
                                      y0= [0.0, 0.0, 0.0],
                                      u0 = [np.deg2rad(-60.), np.deg2rad(-50.)],
                                      step_size = 8e-2)

#NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
#                                      N1 = 0, N2 = 1, Nu = 1,
#                                      nd = 2, dd = 2, K = 1,
#                                      Q = np.array([[1e3, 0., 0],
#                                                    [0., 8e3, 0.],
#                                                    [0., 0., 1e3]]),
#                                      Lambda = np.array([[1., 0.],
#                                                         [0., 1.]]),
#                                      s = 1e-20, b = 1e-5, r = 4e3,
#                                      states_to_control = [1, 1, 1],
#                                      y0= [0.0, 0.0, 0.0],
#                                      u0 = [np.deg2rad(-70.), np.deg2rad(-50.)],
#                                      step_size = 8e-2)

NR_opt, Block = SolowayNR(d_model = NNP), BlockGym(vrpn_ip = "192.168.50.24:3883")

log = Logger()
Block.step(NNP.u0)
time.sleep(10)
neutral_point = [0., 0., 0.] #Block.get_state()
print(neutral_point)
NNP.y0 = neutral_point

#target = FixedTarget(a = 0. / 1000., b = 0./1000.,
#                     center = neutral_point)

#target = Diagonal(wavelength = 15000, amplitude= 10./1000., center = neutral_point)

target = FigureEight(a = 15./1000., b = 25./1000., wavelength = 150., center= [-0.0, -0.0, -0.0])#neutral_point)
#target = Pringle(wavelength = 1000, amplitude = 5./1000.,
#                                center = neutral_point)

Block.calibration_max = np.array([24., 1., 11., 75., 29., 98., 168., 1., 1., 1., 4.])
#Block.calibration_max = np.array([615., 110., 103., 157., 99., 155., 170., 1., 1., 7., 6.])
#Block.calibration_max = np.array([613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.])

u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))
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
        Block.step(action = NNP.u0)
        time.sleep(5)
        NNP.y0 = neutral_point #Block.get_state()

        log.log_dictionary['metadata']['neutral_point'] = NNP.y0

        u_deque.clear()
        y_deque.clear()

        u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
        u_action, predicted_states = np.array(NNP.u0), np.array(NNP.y0)

        target.center = NNP.y0
        log.log({str(e): {'predicted': NNP.y0, 'actual': NNP.y0,
                          'yn': NNP.y0, 'elapsed': 0.0, 'u': [NNP.u0]}})
        if (e == 0):
            log.log({'metadata': {'ym': [0., 0., 0.], 'num_signals': 11, 'm': NNP.m,
                                  'n': NNP.nx}})

        for n in range(NUM_TIMESTEPS):
            seconds = time.time()
            signal = np.divide(Block.get_observation(), Block.calibration_max,
                                dtype = np.float64).tolist()

            NNP.yn = []
            ydeq = y_deque.copy()
            for k in range(NNP.K):
                neural_network_input = np.array((np.array(list(u_deque))).flatten().tolist() + \
                                np.array(list(ydeq)).flatten().tolist() + signal).reshape(1, 1, -1)
                predicted_states = NNP.predict(neural_network_input).flatten()
                NNP.yn += [(NNP.C @ predicted_states).tolist()]
                ydeq = roll_deque(ydeq, predicted_states.tolist())

            y_deque = roll_deque(y_deque, predicted_states.tolist())
            NNP.last_model_input = neural_network_input
            target_path = target.spin(n, 0, NNP.K, 3)
            NNP.ym = (NNP.C @ target_path.T).reshape(NNP.ny, -1).T.tolist()

            u_optimal, del_u, _ = NR_opt.optimize(u = u_optimal_old, delu = del_u,
                                        maxit = 1, rtol = 1e-4, verbose = False)

            u_action = u_optimal[0, :].tolist()
            del_u_action = del_u[0, :].tolist()

            u_action[0] = np.clip((np.rad2deg(u_action[0]) + 50.) - 50. + 20., -100., 50.)
            u_action[1] = np.clip((np.rad2deg(u_action[1]) + 50.) - 50. + 20., -100., 50.)

            #u_action[0] = ((1.+np.cos(2.* np.pi / 1000. * n))/2. * 150. - 100.)
            #u_action[1] = ((1.+np.sin(2.* np.pi / 1000. * n))/2. * 150. - 100.)

            Block.step(action = u_action)

            NNP.update_dynamics(u_optimal[0, :].tolist(), del_u_action,
                        predicted_states.tolist(), target_path[0, :].tolist())

            u_optimal_old = u_optimal
            u_deque = roll_deque(u_deque, u_optimal[0, :].tolist())
            elapsed = time.time()-seconds
            actual_ = np.array(Block.get_state()  - np.array([-0.09223248064517975, 0.00512850284576416, 0.0041762664914131165])).tolist()

            if verbose is not None:
                if verbose == 0:
                    log.verbose( actual = actual_,
                                yn = predicted_states, ym = target_path[0, :],
                                elapsed = elapsed, u = u_action)
                if verbose == 1:
                    log.verbose(u_action = u_action, elapsed = time.time() - seconds)

            log.log({str(e) : {'actual' : actual_,
                            'yn' : predicted_states.tolist(),
                            'elapsed' : elapsed,
                            'u' : [u_action],
                            'signal' : signal}})
            if e==0:
                log.log({'metadata' : {'ym' : target_path[0, :].tolist()}})

        u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, 2))
        Block.reset()

    Block.step([-80., -50.])
    if savelog:
        log.save_log(filename=FILENAME)
    log.plot_log()


except Exception as e1:
    print(str(e1))
    print("Closing all connections!")
    Block.reset()
    Block.motors.close_connection()

