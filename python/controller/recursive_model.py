from .functions import *
from .cost import NN_Cost
from .constraints import *
from typing import List
from test.training_recnn import thousand_mse
import tensorflow as tf
import tensorflow.keras.backend as K
import keras.losses as klosses
import numpy as np
from collections import deque
import time
tf.enable_eager_execution()
klosses.custom_loss = thousand_mse

class ModelException(Exception):
    pass

"""
        ---------------------------------------------------------------------
            Soloway, D. and P.J. Haley, "Neural Generalized Predictive Control,"
            Proceedings of the 1996 IEEE International Symposium on Intelligent
            Control, 1996, pp. 277-281.

            Calculating h'th element of the Jacobian
            Calculating m'th and h'th element of the Hessian
        ---------------------------------------------------------------------
"""
class RecursiveNeuralNetworkPredictor():
    def __init__(self, model_file : str, N1 : int = 0, N2 : int = 3, Nu : int = 2,
                 K : int = 3, Q : List[List[float]] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                 Lambda : List[List[float]] = [[0.3, 0.0], [0., 0.2]], nd : int = 3,
                 dd : int = 3, y0 : List[float] = [0, 0],
                 u0 : List[float] = [0., 0.],
                 s : float = 1e-20, b : float = 1e-10, r : float = 4e-1,
                 states_to_control : List[bool] = [0, 1, 1], step_size : float = 5e-2):

        self.n1, self.n2, self.nu, self.y0, self.u0 = N1, N2, Nu, y0, u0

        self.ym = None
        self.yn = None

        self.nx = len(y0)
        self.ny = sum(states_to_control)
        self.m = len(u0)

        self.Q = np.array(Q)
        self.Lambda = np.array(Lambda)
        self.Q = 0.5*(self.Q + self.Q.T)
        self.Lambda = 0.5*(self.Lambda + self.Lambda.T)
        self.step_size = step_size
        self.K = K
        self.model = tf.keras.models.load_model(model_file, compile=False)

        self.C = self.__make_C_matrix(states_to_control)
        self.states_to_control = states_to_control

        self.first_layer_index, self.layertype = self.__get_hidden_layer_data()

        self.nd = nd # associated with u( . ) not counting u(n)
        self.dd = dd # associated with y( . )

        self.previous_first_der = 1.0 # important for recursions
        self.previous_second_der = 1.0 # important for recursions

        self.y_deque, self.delu_deque, self.u_deque, self.ym_deque = deque(), \
                                                            deque(), deque(), deque()
        self.prediction = None

        self.hid = self.model.layers[self.first_layer_index].units

        self.s = s
        self.b = b
        self.r = r

        self.initialize_deques(self.u0, self.y0)
        self.cost = NN_Cost(self)
        self.input_vector = None

    def __get_hidden_layer_data(self):
        first_layer_index = 0  # first layer may be Gaussian noise
        layertype = self.model.layers[first_layer_index]

        while not isinstance(layertype, tf.keras.layers.Dense):
            first_layer_index += 1
            layertype = self.model.layers[first_layer_index]
        return first_layer_index, layertype

    def __make_C_matrix(self, states_to_control):
        C = np.zeros((int(sum(states_to_control)), int(len(states_to_control))))
        ii = 0
        for t in range(len(states_to_control)):
            if states_to_control[t] == 1:
                C[ii, t] = 1
                ii += 1
        return C

    def initialize_deques(self, u0 : List[float], x0 : List[float]):
        for _ in range(self.n2 - self.n1):
            self.y_deque.appendleft(self.y0)
            self.ym_deque.appendleft(x0)
        for _ in range(self.nu):
            self.u_deque.appendleft(u0)
            self.delu_deque.appendleft([0, 0])

    def update_dynamics(self, u : List[float] = [0., -50.], del_u : List[float] = [0., 0.],
                            y : List[float] = [0., 0., 0.], ym : List[float] = [0., 0.]):
        """
            y_deque = y(n), y(n-1), y(n-2), ..., y(n-T)
            u_deque = u(n), u(n-1), u(n-2), ...., u(n-T)
        """
        self.y_deque.pop()
        self.u_deque.pop()
        self.delu_deque.pop()
        self.ym_deque.pop()

        self.u_deque.appendleft(u)
        self.delu_deque.appendleft(del_u)
        self.y_deque.appendleft(y)
        self.ym_deque.appendleft(ym)

    def get_computation_vectors(self):
        y = np.array(self.yn)
        ym = np.array(self.ym)
        return y, ym

    def __Phi_prime(self):
        if self.model.layers[-1].get_config()['activation'] == 'linear':
            return np.array([1.0]*self.nx) # linear activation on output
        if self.model.layers[-1].get_config()['activation'] == 'tanh':
            netj = np.arctanh(self.prediction)
            return  np.array(1.-np.tanh(netj)**2)
        if self.model.layers[-1].get_config()['activation'] == 'sigmoid':
            netj = np.ln(self.prediction / (1.-self.prediction))
            sigmoid = 1./(1.+np.exp(-1.*netj))
            return np.array(sigmoid*(1.-sigmoid))

    def __Phi_prime_prime(self):
        if self.model.layers[-1].get_config()['activation'] == 'linear':
            return np.array([0.0]*self.nx) # linear activation on output
        if self.model.layers[-1].get_config()['activation'] == 'tanh':
            netj = np.arctanh(self.prediction)
            return np.array(-2.*np.tanh(netj)*(1.-np.tanh(netj)**2))
        if self.model.layers[-1].get_config()['activation'] == 'sigmoid':
            x = np.ln(self.prediction / (1.-self.prediction))
            return np.array((2.*np.exp(-2.*x))/(np.exp(-1.*x)+ 1.)**3. - (np.exp(-1.*x))/(np.exp(-1.*x)+1.)**2.)

    def __partial_2_fnet_partial_nph_partial_npm(self, h, m, j):
        """
             D2^2f_j(net)
             ------------
            Du(n+h)Du(n+m)
        """
        return self.__Phi_prime() * \
                    self.__partial_2_net_partial_u_nph_partial_npm(h, m, j) + \
                        self.__Phi_prime_prime() @ \
                            self.__partial_net_partial_u(h, j) @ \
                                self.__partial_net_partial_u(m, j).T

    def __partial_2_yn_partial_nph_partial_npm(self, h, m, j):
        """
                 D^2yn
            ---------------
            Du(n+h) Du(n+m)
        """
        outputs = self.model.layers[-1].units
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = np.array([0.0]*outputs)
        for i in range(self.hid):
            sum_output += weights[i, :] * \
                        self.__partial_2_fnet_partial_nph_partial_npm(h, m, j).squeeze()
        self.previous_second_der = sum_output.T
        if len(sum_output) > 0:
            return self.C @ np.array(sum_output).T
        else:
            return self.C @ sum_output

    def __partial_2_net_partial_u_nph_partial_npm(self, h, m, j):
        """
              D^2 net_j
            -------------
            Du(n+h)Du(n+m)
        """
        weights = self.model.layers[self.first_layer_index].get_weights()[0]
        sum_output = 0.0
        for i in range(min(j, self.dd*self.nx)):
            step_ = []
            for ii in range(self.nx):
                step_ += [step(j - i + ii - 1)]
            sum_output += np.sum(weights[i * self.m + self.nd * self.m: i * self.m + \
                                                                        self.nd * self.m + \
                                                                        self.nx, j] * \
                                        self.previous_second_der * np.array(step_))
        return np.array(sum_output)

    def __partial_yn_partial_u(self, h, j):
        """
               D yn
            -----------
             D u(n+h)
        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = np.zeros((self.nx, self.m))

        for i in range(self.hid):
            sum_output += (weights[i, :] * \
                                 self.__partial_fnet_partial_u(h, j).T).T

        self.previous_first_der = sum_output
        return np.array(self.C @ sum_output)

    def __partial_fnet_partial_u(self, h, j):
        """
            D f_j(net)
            ---------
             D u(u+h)
        """
        return (self.__Phi_prime() * \
                             self.__partial_net_partial_u(h, j).T).T

    def __partial_net_partial_u(self, h : int, j : int):
        """
             D net_j
            ---------
            D u(n+h)

            h-> [0, ..., Nu]
            j-> [0, ..., hid]
            k-> [0, ..., m]
        """
        weights = self.model.layers[self.first_layer_index].get_weights()[0]
        sum_output = [0.0]*self.m

        for i in range(self.nd):
            if (j - self.nu) < i:
                delta = [kronecker_delta(j - i + ii, h) for ii in range(self.m)]
            else:
                delta = [kronecker_delta(self.nu, h)]*self.m
            index_weights = np.arange(i * self.m, i * self.m + self.m)
            sum_output += weights[index_weights, j] * delta

        sum_output = np.tile(sum_output, self.nx).reshape(self.nx, -1)

        for i in range(min(j, self.dd)):
            step_ = step(j-i)
            index_weights = np.arange(i * self.nx + self.nd*self.m,
                                     i * self.nx + self.nd * self.m + self.nx)
            sum_output += (weights[index_weights, j] * step_ * \
                                        np.array(self.previous_first_der).T).T
        return np.array(sum_output)

    def __partial_delta_u_partial_u(self, j, h):
        """
            D delta u
            ---------
            D u(n+h)
        """
        return kronecker_delta(h, j) - kronecker_delta(h, j-1)

    def hessian(self, u, del_u):
        y, ym = self.get_computation_vectors()
        del_y = ym[self.n1:self.n2, :] - y[self.n1:self.n2, :]

        hessian = np.zeros((self.nu, self.nu))
        ynu = self.gradient()
        dynu_du = self.second_derivative()
        hessian += np.sum(2. * self.Q @ (ynu * ynu)) - np.sum(2. * (self.Q @ dynu_du).T @ del_y.T)

        for h in range(self.nu):
            for m in range(self.nu):
                second_y, second_y1, temp = [], [], []
                for j in range(self.m):
                    second_y += [self.__partial_delta_u_partial_u(j, m)]
                    second_y1 += [self.__partial_delta_u_partial_u(j, h)]
                hessian[h, m] += np.sum(2.* self.Lambda @
                                np.array(second_y) @ np.array(second_y1).T)

                for j in range(self.nu):
                   for i in range(self.m):
                       hessian[h, m] += kronecker_delta(h, j)*kronecker_delta(m, j) * \
                               (2.0*self.s / np.power((u[j, i] + self.r / 2. - \
                               self.b), 3.0) + \
                               2.0 * self.s / np.power(self.r/2. +\
                               self.b - u[j, i], 3.0))
        return hessian

    @tf.function
    def grad_grad(self, x):
        with tf.GradientTape(persistent = True) as tape:
            tape.watch(x)
            with tf.GradientTape(persistent=True) as ttape:
                ttape.watch(x)
                y = self.model(x, training=False)
            grad = ttape.jacobian(y, x)
        second_der = tape.jacobian(grad, x)
        del tape
        del ttape
        return second_der.numpy()

    def num_grads(self, x):
        shape = list(x.shape)
        h = self.step_size
        x = x.flatten()
        x_p = x + np.eye(len(x))*h
        x_m = x - np.eye(len(x))*h
        shape[0] = len(x)
        shape = tuple(shape)
        jacobian = np.array((self.model.predict(x_p.reshape(shape), batch_size = len(x)).flatten() - \
                              self.model.predict(x_m.reshape(shape), batch_size = len(x)).flatten()))\
                   / (2.*h)
        #jacobian = np.array((self.model.predict(x_p.reshape(shape), batch_size = len(x)).flatten() - \
        #                    self.model.predict(np.tile(x, reps = len(x)).reshape(shape),
        #                                       batch_size = len(x)).flatten())) / h
        return jacobian

    def num_grad_grad(self, x):
        shape = list(x.shape)
        h = self.step_size
        x = x.flatten()

        x_p = x + np.eye(len(x)) * h
        x_m = x - np.eye(len(x)) * h
        xx = x + np.eye(len(x)) * 0.0

        shape[0] = len(x)
        shape = tuple(shape)
        hessian = (self.model.predict(x_p.reshape(shape), batch_size=len(x)).flatten() - \
                   2. * self.model.predict(xx.reshape(shape), batch_size=len(x)).flatten() +
                   self.model.predict(x_m.reshape(shape), batch_size=len(x)).flatten()) / \
                  (h * h)
        return np.array(hessian)

    @tf.function
    def grads(self, x):
        with tf.GradientTape(persistent=True, watch_accessed_variables=False) as tape:
            tape.watch(x)
            y = self.model(x, training=False)
        jacobian = tape.jacobian(y, x)
        del tape # just in case
        return jacobian

    def gradient(self):
        """
            Gradient tapes: https://www.tensorflow.org/api_docs/python/tf/GradientTape
        """
        gradient = self.num_grads(self.input_vector).reshape(-1, self.nx).T
        ynu = gradient[:, :self.m * self.nd]
        ynu = ynu.reshape(self.nx, -1, self.m)
        ynu = np.sum(ynu, axis = 1)
        return self.C @ ynu

    def second_derivative(self):
        """
            Using Keras gradient tapes

            TODO: Fixme
        """
        second_gradient = self.num_grad_grad(self.input_vector)
        second_gradient = second_gradient.reshape(-1, self.input_vector.shape[1], self.nx)
        second_gradient = second_gradient[:self.nd * self.m, :self.nd*self.m, :]
        second_gradient = second_gradient.reshape(self.m, self.nx, self.nd)
        second_gradient = second_gradient.sum(axis = 0).sum(axis = 0)
        return second_gradient

    def jacobian(self, u, del_u):
        y, ym = self.get_computation_vectors()
        del_y = ym[self.n1:self.n2, :] - y[self.n1:self.n2, :]
        del_u = del_u.copy()
        jacobian = np.zeros((self.nu, self.m))

        sub_sum = del_y @ self.Q
        ynu = self.gradient()
        sum_output = np.sum(sub_sum @ ynu, axis=0)

        for h in range(self.nu):
            for j in range(self.nu):
                ynu1 = self.__partial_delta_u_partial_u(h, j)
                ynu1 = np.array(ynu1)
                sum_output += 2. * np.squeeze(np.array(del_u) @ self.Lambda) * ynu1

                sub_sum = np.array([0.0, 0.0])
                for i in range(self.m):
                    sub_sum[i] += kronecker_delta(h, j) * (-self.s / np.power(u[j, i] + \
                                          self.r / 2.0 - self.b, 2) + \
                                self.s / np.power(self.r / 2.0 + \
                                             self.b - u[j, i], 2.0))
                jacobian[j, :] += sub_sum
        jacobian += np.array(sum_output).reshape(self.nu, -1)
        return jacobian

    def compute_cost(self):
        return self.cost.compute_cost()

    def measure(self, u):
        pass

    def predict(self, x):
        self.input_vector = x
        self.prediction = self.model.predict(x, batch_size=1)
        return self.prediction




