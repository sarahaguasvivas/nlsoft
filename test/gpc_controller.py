#!/usr/bin/env python2.7
from controller.dynamic_model import *
from controller.newton_raphson import *
from gym.block_gym import *
import time

plt.style.use('dark_background')
model_filename = 'test/sys_int.hdf5'

NNP = NeuralNetworkPredictor(model_file = model_filename, N1= 0, N2 = 2, Nu = 3, \
                                    nd = 3, dd = 2, K = 2, lambd = [1., 2.])

NR_opt = NewtonRaphson(cost = NNP.Cost, d_model = NNP)

B = BlockGym() # declare my block
B.reset() # if block is not neutral, bring back to neutral
B.stretch() # stretch block for better signal readings before calibrating
B.get_signal_calibration() # calibrate signal of the block

u_optimal_old = np.array([0., 0.])

new_state_new = B.get_state()
du = [0.0]*2

elapsed = []
u_optimal_list = []
ym = []
state = []
yn = []

for n in range(100):
    seconds = time.time()
    future_outputs = NNP.predict(new_state_new).flatten() # need to have a deque at dynamic model
    NNP.yn = B.get_target()

    new_state_old = new_state_new
    u_optimal = np.reshape(NR_opt.optimize(u = u_optimal, del_u = du, rtol = 1e-8, \
                    maxit = 6, verbose = False)[0], (-1, 1))

    u_optimal_list += [ u_optimal.flatten().tolist() ]

    du = np.array(u_optimal_old) - np.array(u_optimal)

    du = du.flatten()

    u_optimal_old = u_optimal

    # need to keep adding stuff. Also need to have deque with previous signals and states
