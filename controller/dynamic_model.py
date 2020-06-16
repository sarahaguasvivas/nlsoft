from functions import *
from cost import NN_Cost
from constraints import *

from keras import layers
from keras.models import load_model
import keras

import numpy as np
from keras import backend as K
import tensorflow as tf

from collections import deque

def custom_loss(y_true, y_pred):
    return 1000*K.mean(K.square(y_pred - y_true), axis = 1)

keras.losses.custom_loss = custom_loss

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
class NeuralNetworkPredictor():
    def __init__(self, model_file, N1 = 0 , N2 = 3 ,  Nu = 2 , \
                            K = 3 , Q = [[1, 0, 0], [0, 1, 0], [0, 0, 1]],\
                            Lambda = [[0.3, 0.0], [0., 0.2]] , nd = 3,\
                                    dd = 3, x0= [0, 0], u0= [0, 0], \
                                        s = 1e-20, b = 1e-10, r = 4e-1,
                                        states_to_control = [0, 1, 1]):

        self.N1, self.N2, self.Nu, self.x0, self.u0 = N1, N2, Nu, x0, u0

        self.ym = None
        self.yn = None

        self.nx = len(x0)
        self.ny = sum(states_to_control)
        self.nu = len(u0)

        self.Q = np.array(Q)
        self.Lambda = np.array(Lambda)
        self.Q = 0.5*(self.Q + self.Q.T)
        self.Lambda = 0.5*(self.Lambda + self.Lambda.T)

        self.K = K
        self.model = load_model(model_file)

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

        self.constraints = Constraints(s = s, b = b, r = r)

        self.hid = self.model.layers[-1].input_shape[1] - 11 # minus 11 sensor signals

        self.initialize_deques(self.u0, self.x0)
        self.cost = NN_Cost(self)

    def __get_hidden_layer_data(self):
        first_layer_index = 0  # first layer may be Gaussian noise
        layertype = self.model.layers[first_layer_index]

        while not isinstance(layertype, layers.Dense):
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

    def initialize_deques(self, u0, x0):
        for _ in range(self.N2 - self.N1):
            self.y_deque.appendleft(self.x0)
            self.ym_deque.appendleft(x0)
        for _ in range(self.Nu):
            self.u_deque.appendleft(u0)
            self.delu_deque.appendleft([0, 0])

    def update_dynamics(self, u = [0, -50], del_u = [0, 0],\
                            y = [0, 0, 0], ym = [0, 0]):
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
        Y = np.array(self.yn)
        YM = np.array(self.ym)

        U = np.array(list(self.u_deque))
        delU = np.array(list(self.delu_deque))
        return Y, YM, U, delU

    def __Phi_prime(self, x = 0):
        if self.model.layers[-1].get_config()['activation'] == 'linear':
            return np.array([1.0]*self.nx) # linear activation on output
        if self.model.layers[-1].get_config()['activation'] == 'tanh':
            netj = np.arctanh(self.prediction)
            return  np.array(1.-np.tanh(netj)**2)
        if self.model.layers[-1].get_config()['activation'] == 'sigmoid':
            netj = np.ln(self.prediction / (1-self.prediction))
            sigmoid = 1./(1.+np.exp(-1.*netj))
            return np.array(sigmoid*(1.-sigmoid))

    def __Phi_prime_prime(self, x = 0):
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
        return self.__Phi_prime().dot( \
                    self.__partial_2_net_partial_u_nph_partial_npm(h, m, j)) + \
                        self.__Phi_prime_prime().dot( \
                            self.__partial_net_partial_u(h, j)).dot( \
                                self.__partial_net_partial_u(m, j))

    def __partial_2_yn_partial_nph_partial_npm(self, h, m, j):
        """
                 D^2yn
            ---------------
            Du(n+h) Du(n+m)
        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = 0.0
        for i in range(self.hid):
            for ii in range(weights.shape[1]):
                sum_output+= weights[i, ii] * \
                            self.__partial_2_fnet_partial_nph_partial_npm(h, m, j)
        self.previous_second_der = self.C.dot(sum_output.T)
        if len(sum_output) > 0:
            return self.C.dot(np.array(sum_output).T)
        else:
            return self.C.dot(sum_output)
    def __partial_2_net_partial_u_nph_partial_npm(self, h, m, j):
        """
              D^2 net_j
            -------------
            Du(n+h)Du(n+m)
        """
        weights = self.model.layers[self.first_layer_index].get_weights()[0]
        sum_output = 0.0
        for i in range(min(j, self.dd)):
            step_ = []

            for ii in range(self.nx):
                step_ += [step(j - i + ii - 1)]
            sum_output += np.sum(weights[i*self.nu + self.nd*self.nu: i*self.nu + \
                                        self.nd*self.nu + \
                                        self.nx, j] * \
                                        self.previous_second_der * np.array(step_))
        return np.array(sum_output)

    def __partial_yn_partial_u(self, h, j):

        """
               D yn
            -----------
             D u(n+h)

            TODO this is supposed to be a vector
            num_y x num_u
        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = np.array([0.0]*weights.shape[1])
        for i in range(self.hid):
            sum_output += np.dot(np.array(weights[i, :]) ,
                            self.__partial_fnet_partial_u(h, j).T)
        self.previous_first_der = sum_output.tolist()
        sum_output = self.C.dot(sum_output.T).T

        return np.array(sum_output)

    def __partial_fnet_partial_u(self, h, j):
        """
            D f_j(net)
            ---------
             D u(u+h)
        """
        return np.dot(self.__Phi_prime(), self.__partial_net_partial_u(h, j))

    def __partial_net_partial_u(self, h, j):
        """
             D net_j
            ---------
            D u(n+h)
        """
        weights = self.model.layers[self.first_layer_index].get_weights()[0]
        sum_output = 0.0
        for i in range(self.nd):
            delta = []
            if (j - self.Nu) < i:
                for ii in range(self.nu):
                    delta += [kronecker_delta(j - i + ii, h)]
                sum_output += np.dot(weights[i*self.nu:i*self.nu + self.nu, j] , \
                                                                delta)
            else:
                delta = []
                for ii in range(self.nu):
                    delta += [kronecker_delta(self.Nu, h)]
                sum_output += np.dot(weights[i*self.nu:i*self.nu + self.nu, j] ,\
                                                                delta)
        for i in range(min(j, self.dd)):
            step_ = []
            for enum, ii in enumerate(self.states_to_control):
                step_ += [step(j-i+ii-1)]
            sum_output += np.dot(weights[i*self.nx + self.nd*self.nu: i*self.nx + \
                                        self.nd*self.nu + \
                                        self.nx, j], \
                                np.multiply(self.previous_first_der , np.array(step_)))
        return np.array(sum_output)

    def __partial_delta_u_partial_u(self, j, h):
        """
            D delta u
            ---------
            D u(n+h)
        """
        return kronecker_delta(h, j) - kronecker_delta(h, j-1)

    def compute_hessian(self, u, del_u):
        Y, YM , _, _ = self.get_computation_vectors()
        delY = YM[self.N1:self.N2, :] - Y[self.N1:self.N2, :]

        U = u.copy()
        del_u = del_u.copy()

        hessian = np.zeros((self.Nu, self.Nu))
        for h in range(self.Nu):
            for m in range(self.Nu):
                ynu, ynu1, temp = [], [], []
                for j in range(self.N1, self.N2):
                    ynu  += [self.__partial_yn_partial_u(j, m)]
                    ynu1 += [self.__partial_yn_partial_u(j, h)]
                    temp += [self.__partial_2_yn_partial_nph_partial_npm(h, m, j)]

                ynu, ynu1, temp = np.array(ynu), np.array(ynu1), \
                        np.array(temp).reshape(-1, self.ny)

                hessian[h, m] += np.sum(2.*ynu.dot(self.Q).dot(np.array(ynu1).T) -
                                            2.*delY.dot(self.Q).dot(temp.T))

                second_y, second_y1, temp = [], [], []
                for j in range(self.nu):
                    second_y+=[self.__partial_delta_u_partial_u(j, m)]
                    second_y1+=[self.__partial_delta_u_partial_u(j, h)]
                hessian[h, m] += np.sum(2.* np.dot(self.Lambda,
                                second_y).dot(np.array(second_y1).T))

                for j in range(self.Nu):
                   for i in range(self.nu):
                       hessian[h, m] += kronecker_delta(h, j)*kronecker_delta(m, j) * \
                               (2.0*self.constraints.s / np.power((U[j, i] + self.constraints.r / 2. - \
                               self.constraints.b), 3.0) + \
                               2.0 * self.constraints.s / np.power(self.constraints.r/2. +\
                               self.constraints.b - U[j, i], 3.0))
        return hessian

    def compute_jacobian(self, u, del_u):
        Y, YM, _, _ = self.get_computation_vectors()
        delY = YM[self.N1:self.N2, :] - Y[self.N1:self.N2, :]

        U = u.copy()
        delU = del_u.copy()

        dJ = np.zeros((self.Nu, U.shape[1]))
        sum_output = np.array([0.0]*self.nu)

        for h in range(self.Nu):
            ynu, ynu1 = [], []

            for j in range(self.nu):
                ynu += [self.__partial_yn_partial_u(j, h).tolist()]
                ynu1+=[self.__partial_delta_u_partial_u(j, h)]

            ynu1 = np.array(ynu1)
            ynu = np.array(ynu)

            sub_sum = delY.dot(self.Q).dot(np.array(ynu.T))

            sum_output += (-2.*np.sum(sub_sum, axis = 0)).flatten().tolist()

            sum_output += 2.* np.sum(np.dot(np.array(delU),
                                        self.Lambda).dot(ynu1), axis = 0)
            for j in range(self.Nu):
               sub_sum = np.array([0.0, 0.0])
               for i in range(self.nu):
                   sub_sum[i] += kronecker_delta(h, j) * ( -self.constraints.s / np.power(U[j, i] +  \
                       self.constraints.r / 2.0 - self.constraints.b , 2) + \
                               self.constraints.s/ np.power(self.constraints.r/2.0 + \
                               self.constraints.b - U[j, i] , 2.0) )
               sum_output += sub_sum
            dJ[h, :] = sum_output
        return dJ

    def Fu(self, u, del_u):
        return self.compute_jacobian(u, del_u)

    def Ju(self, u, del_u):
        return self.compute_hessian(u, del_u)

    def compute_cost(self):
        return self.cost.compute_cost()

    def measure(self, u):
        pass

    def predict(self, x):
        self.prediction = self.model.predict(x, batch_size=1)
        return  self.prediction




