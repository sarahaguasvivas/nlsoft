import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'nlsoft', 'python'))
from python.controller.soloway_nr import *
from python.block_gym.block_gym_vani import *
from python.controller.recursive_model import *
from python.logger.logger import Logger
from python.utilities.util import *
from python.target.target import SMDTarget
import numpy as np

model_filename = 'sys_id_spring_mass_GRU.hdf5'
NUM_EXPERIMENTS = 1
WAVELENGTH = 1
NUM_TIMESTEPS = 1000
FILENAME = 'gru_log_output_spring_mass_damper.json'
verbose = 1
savelog = True

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 3, Nu= 1,
                                      nd = 2, dd = 2, K = 3,
                                      Q = np.array([[1e6, 0.],
                                                    [0., 1e6]]),
                                      Lambda = np.array([[1.]]),
                                      s = 1e-20, b = 1e-5, r = 4e2,
                                      states_to_control = [1, 1],
                                      y0= [0.0, 0.0],
                                      u0 = [0.0],
                                      step_size = 1e-3)

NR_opt, Block = SolowayNR(d_model = NNP), BlockGymVani()

log = Logger()
Block.step(NNP.u0)
neutral_point = [0.]

NNP.y0 = neutral_point
Block.calibration_max = 1

target = SMDTarget(distance = 5)

u_optimal_old = np.reshape([NNP.u0] * NNP.nu, (-1, 1))
del_u = np.zeros(u_optimal_old.shape)

log.log({'metadata' : {'neutral_point' : neutral_point,
         'num_experiments' : NUM_EXPERIMENTS,
         'num_timesteps': NUM_TIMESTEPS,
         'ym' : []}})

u_deque = deque()
y_deque = deque()

#try:
for e in range(NUM_EXPERIMENTS):
    log.log({str(e) : {'predicted' : [], 'actual' : [], 'yn' : [],
            'elapsed' : [], 'u' : []}})
    print(e)
    NNP.y0 = [0.]*2

    log.log_dictionary['metadata']['neutral_point'] = NNP.y0

    u_deque.clear()
    y_deque.clear()

    u_deque, y_deque = first_load_deques(NNP.y0, NNP.u0, NNP.nd, NNP.dd)
    u_action, predicted_states = np.array(NNP.u0), np.array(NNP.y0)

    log.log({str(e) : {'predicted' : NNP.y0, 'actual' : NNP.y0,
                       'yn' : NNP.y0, 'elapsed' : 0.0, 'u' : [NNP.u0]}})
    if (e == 0):
        log.log({'metadata': {'ym': neutral_point, 'num_signals' : 18, 'm' : NNP.m,
                                                                'n' : 2}})
    for n in range(NUM_TIMESTEPS):
        seconds = time.time()
        NNP.yn = []
        ydeq = y_deque.copy()
        signal = [0.]*18
        for k in range(NNP.K):
            neural_network_input = np.array((np.array(list(u_deque))).flatten().tolist() + \
                            np.array(list(ydeq)).flatten().tolist()).reshape(1, 1, 6)
            predicted_states = NNP.predict(neural_network_input).flatten()
            NNP.yn += [(NNP.C @ predicted_states).tolist()]
            ydeq = roll_deque(ydeq, predicted_states.tolist())
        y_deque = roll_deque(y_deque, predicted_states.tolist())
        NNP.last_model_input = neural_network_input
        target_path = target.spin(n, 0, NNP.K, NNP.nx)
        NNP.ym = (NNP.C @ target_path.T).reshape(NNP.ny, -1).T.tolist()
        u_optimal, del_u, _ = NR_opt.optimize(u = u_optimal_old, delu = del_u,
                                    maxit = 1, rtol = 1e-4, verbose = False)

        u_optimal = np.clip(u_optimal, -1., 1.)

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
                        'signal': signal}})
        if e==0:
            log.log({'metadata' : {'ym' : target_path[0, :].tolist()}})
        print(np.linalg.norm(np.array(NNP.ym) - np.array(NNP.yn)))
    u_optimal_old = np.reshape(NNP.u0 * NNP.nu, (-1, NNP.m))
    Block.reset()

if savelog:
    log.save_log(filename=FILENAME)
log.plot_log()


#except Exception as e1:
#    print(str(e1))
#    print("Closing all connections!")
#    Block.reset()

