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
    def __init__(self, model_file, N1 = 1 , N2 = 3 ,  Nu = 3 , \
                            K = 2 , Q = [[1, 0, 0], [0, 1, 0], [0, 0, 1]],\
                            Lambda = [[0.3, 0.0], [0., 0.2]] , nd = 3,\
                                    dd = 3, y0= [0, 0], u0= [0, 0], \
                                        s = 1e-20, b = 1e-10, r = 4e-1):
        self.N1 = N1
        self.N2 = N2
        self.Nu = Nu

        self.ym = None
        self.yn = None

        self.y0 = y0
        self.u0 = u0

        self.Q = np.array(Q)
        self.Lambda = np.array(Lambda)

        self.K = K
        self.constraints = Constraints(s = s, b = b, r = r)
        self.model = load_model(model_file)

        self.first_layer_index = 0 # first layer may be Gaussian noise
        layertype = self.model.layers[self.first_layer_index]

        while not isinstance(layertype, layers.Dense):
            self.first_layer_index += 1
            layertype = self.model.layers[self.first_layer_index]

        self.nd = nd # associated with u( . ) not counting u(n)
        self.dd = dd # associated with y( . )
        self.Hessian = None

        self.previous_first_der = 0.0 # important for recursions
        self.previous_second_der = 0.0 # important for recursions

        self.y_deque = deque()
        self.delu_deque = deque()
        self.u_deque = deque()
        self.ym_deque = deque()

        self.hid = self.model.layers[-1].input_shape[1] - 11 # minus 11 sensor signals

        self.num_u = 2
        self.num_y = 3

        self.initialize_deques(self.u0, self.y0)
        self.cost = NN_Cost(self)

    def initialize_deques(self, u0, y0):
        for _ in range(self.N2):
            self.y_deque.appendleft(self.y0)
            self.ym_deque.appendleft(y0)

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
        Y = 1000*np.array(self.yn) # converting to mm
        YM = 1000*np.array(self.ym) # converting to mm
        U = np.array(list(self.u_deque))
        delU = np.array(list(self.delu_deque))
        return Y, YM, U, delU

    def __Phi_prime(self, x = 0):
        if self.model.layers[-1].get_config()['activation'] == 'linear':
            return 1.0 # linear activation on output
        if self.model.layers[-1].get_config()['activation'] == 'tanh':
            return  1. / np.cosh(x)**2.
        if self.model.layers[-1].get_config()['activation'] == 'sigmoid':
            sigmoid = 1./(1.+np.exp(-1.*x))
            return sigmoid*(1-sigmoid)

    def __Phi_prime_prime(self, x = 0):
        if self.model.layers[-1].get_config()['activation'] == 'linear':
            return 0.0 # linear activation on output
        if self.model.layers[-1].get_config()['activation'] == 'tanh':
            return-2.*np.tanh(x)/ np.cosh(x)**2.
        if self.model.layers[-1].get_config()['activation'] == 'sigmoid':
            return (2.*np.exp(-2.*x))/(np.exp(-1.*x)+ 1.)**3. - (np.exp(-1.*x))/(np.exp(-1.*x)+1.)**2.

    def __partial_2_fnet_partial_nph_partial_npm(self, h, m, j):
        """
             D2^2f_j(net)
             ------------
            Du(n+h)Du(n+m)
        """
        return self.__Phi_prime() * \
                    self.__partial_2_net_partial_u_nph_partial_npm(h, m, j) + \
                        self.__Phi_prime_prime() * \
                            self.__partial_net_partial_u(h, j) * \
                                self.__partial_net_partial_u(m, j)

    def __partial_2_yn_partial_nph_partial_npm(self, h, m, j):
        """
                 D^2yn
            ---------------
            Du(n+h) Du(n+m)
            TODO: This should be a num_y x num_y matrix
        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = 0.0
        for i in range(self.hid):
            for j in range(weights.shape[1]):
                sum_output+= np.multiply(weights[i, j], \
                            self.__partial_2_fnet_partial_nph_partial_npm(h, m, j))
        self.previous_second_der = sum_output
        return sum_output

    def __partial_2_net_partial_u_nph_partial_npm(self, h, m, j):
        """
              D^2 net_j
            -------------
            Du(n+h)Du(n+m)
        """
        weights = self.model.layers[self.first_layer_index].get_weights()[0]
        sum_output = 0.0
        for i in range(0, min(j, self.dd)):
            step_ = []
            for ii in range(self.num_y):
                step_ += [step(j - i + ii - 1)]
            sum_output += np.sum(weights[i*self.num_u + self.nd*self.num_u: i*self.num_u + \
                                        self.nd*self.num_u + \
                                        self.num_y, j] * \
                                        self.previous_second_der * np.array(step_))
        return sum_output

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
            sum_output += np.dot(weights[i, :] , self.__partial_fnet_partial_u(h, j))
        self.previous_first_der = sum_output.tolist()
        return sum_output

    def __partial_fnet_partial_u(self, h, j):
        """
            D f_j(net)
            ---------
             D u(u+h)
        """
        return self.__Phi_prime()*self.__partial_net_partial_u(h, j)

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
            if (j - self.Nu) < i*self.num_u:
                for ii in range(self.num_u):
                    delta += [kronecker_delta(j - i + ii, h)]
                sum_output += np.dot(weights[i*self.num_u:i*self.num_u + self.num_u, j] , \
                                                                delta)
            else:
                delta = []
                for ii in range(self.num_u):
                    delta += [kronecker_delta(self.Nu, h)]
                sum_output += np.dot(weights[i*self.num_u:i*self.num_u + self.num_u, j] ,\
                                                                delta)
        for i in range(0, min(j, self.dd)):
            step_ = []
            for ii in range(self.num_y):
                step_ += [step(j-i+ii-1)]
            sum_output += np.dot(weights[i*self.num_y + self.nd*self.num_u: i*self.num_y + \
                                        self.nd*self.num_u + \
                                        self.num_y, j] , \
                                self.previous_first_der * np.array(step_))
        return sum_output

    def __partial_delta_u_partial_u(self, j, h):
        """
            D delta u
            ---------
            D u(n+h)
        """
        return kronecker_delta(h, j) - kronecker_delta(h, j-1)

    def compute_hessian(self, u, del_u):
        Y, YM , U, delU = self.get_computation_vectors()
        Hessian = np.zeros((self.Nu, self.Nu))

        for h in range(self.Nu):
            for m in range(self.Nu):
                ynu, ynu1, temp = [], [], []
                for j in range(self.N1, self.N2):
                    ynu += [self.__partial_yn_partial_u(j, m)]
                    ynu1 += [self.__partial_yn_partial_u(j, h)]
                    temp += [self.__partial_2_yn_partial_nph_partial_npm(h, m, j)]
                ynu, ynu1, temp = np.array(ynu), np.array(ynu1), np.array(temp)
                Hessian[h, m] += np.sum(2.*np.dot(np.dot(np.dot(ynu, ynu1.T), temp), \
                        np.dot(self.Q, (YM[self.N1:self.N2, :] -\
                            Y[self.N1:self.N2, :]).T)), axis = 0)

                for j in range(self.Nu):
                    Hessian[h, m] += np.sum(2.*np.dot(self.Lambda, \
                                    np.array([self.__partial_delta_u_partial_u(j, m)]*self.Nu) *\
                                                np.array([self.__partial_delta_u_partial_u(j, h)]*\
                                                    self.Nu).T).flatten())
                for j in range(self.Nu):
                    for i in range(self.num_u):
                        Hessian[h, m] += kronecker_delta(h, j)*kronecker_delta(m, j) * \
                                (2.0*self.constraints.s / np.power((U[j, i] + self.constraints.r / 2. - \
                                self.constraints.b), 3.0) + \
                                2.0 * self.constraints.s / np.power(self.constraints.r/2. +\
                                self.constraints.b - U[j, i], 3.0))
        print "Hessian: ", Hessian
        return Hessian

    def compute_jacobian(self, u, del_u):
        Y, YM, U, delU = self.get_computation_vectors()

        dJ = np.zeros((self.Nu, U.shape[1]))
        sum_output = np.array([0.0]*self.num_u)

        for h in range(self.Nu):
            ynu = []
            for j in range(self.N1, self.N2):
                ynu += [self.__partial_yn_partial_u(j, h)]
            sum_output += -2. * np.sum(np.dot(np.dot((YM[self.N1:self.N2, :] - \
                            Y[self.N1:self.N2, :]).T, self.Q), np.array(ynu).T), axis = 0)
            ynu = []
            for j in range(self.Nu):
                ynu+=[self.__partial_delta_u_partial_u(j, h)]
            if self.Nu == 1:
                ynu = np.asscalar(np.array(ynu))
            else:
                ynu = np.array(ynu)

            sum_output += np.sum(2.* np.dot(np.dot(np.array(delU).T, self.Lambda.T),
                                        ynu), axis = 0)
            for j in range(self.Nu):
                sub_sum = np.array([0.0, 0.0])
                for i in range(self.num_u):
                    sub_sum[i] += kronecker_delta(h, j) * ( -self.constraints.s / np.power(U[j, i] +  \
                        self.constraints.r / 2.0 - self.constraints.b , 2) + \
                                self.constraints.s/ np.power(self.constraints.r/2.0 + \
                                self.constraints.b - U[j, i] , 2.0) )
                sum_output += sub_sum
            dJ[h, :] = sum_output
        return dJ

    def Fu(self, u, del_u):
        jacobian = self.compute_jacobian(u, del_u)
        return jacobian

    def Ju(self, u, del_u):
        self.Hessian = self.compute_hessian(u, del_u)
        return self.Hessian

    def compute_cost(self):
        return self.cost.compute_cost()

    def measure(self, u):
        pass

    def predict(self, x):
        return self.model.predict(x, batch_size=1)





