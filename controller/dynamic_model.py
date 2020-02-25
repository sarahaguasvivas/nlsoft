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

class ModelException(Exception):
    pass

class NeuralNetworkPredictor():
    def __init__(self, model_file, N1 = 1 , N2 = 5 ,  Nu = 4 , \
                            K = 7 , lambd = [0.3, 0.2, 0.3] , nd = 3,\
                                                dd = 3, y0= [0, 0, 0], u0= [0, 0]):

        """
                w --> weights from input layer to hidden layer
        """
        self.N1 = N1
        self.N2 = N2
        self.Nu = Nu

        self.ym = None
        self.yn = None

        self.y0 = y0
        self.u0 = u0

        self.lambd = lambd

        if len(self.lambd) != self.Nu:
            raise ModelException("The length of lambda should be the same as Nu")

        self.K = K

        self.num_predicted_states = 3
        self.constraints = Constraints()

        self.model = load_model(model_file)

        output_size = self.model.layers[-1].output_shape[1]
        input_size = self.model.layers[0].input_shape[1]

        self.nd = nd # associated with u( . ) not counting u(n)
        self.dd = dd # associated with y( . )

        self.Hessian = np.zeros((output_size, output_size))

        # Important for recursions:
        self.previous_first_der = 1.0
        self.previous_second_der = 1.0

        # Initializing deques
        self.y_deque = deque()
        self.delu_deque = deque()
        self.u_deque = deque()
        self.ym_deque = deque()
        num_signals = 11

        # hid does not include the weights that have to do with the signals
        self.hid = self.model.layers[-1].input_shape[1] - num_signals

        self.num_u = 2
        self.num_y = 3

        self.initialize_deques(self.u0, self.y0)
        self.Cost = NN_Cost(self, self.lambd)

    def initialize_deques(self, u0, y0):
        for i in range(self.N2):
            self.y_deque.appendleft(self.y0)
            self.ym_deque.appendleft(y0) # get wand location

        for _ in range(self.Nu):
            self.u_deque.appendleft(u0)
            self.delu_deque.appendleft([0, 0])

    def update_dynamics(self, u = [0, -50], del_u = [0, 0], y = [0, 0, 0], ym = [0, 0, 0]):
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
        Y = np.array(list(self.y_deque))
        YM = np.array(list(self.ym_deque))
        U = np.array(list(self.u_deque))
        delU = np.array(list(self.delu_deque))
        return Y, YM, U, delU

    def __Phi_prime(self, x = 0):
        """
            Linear output function
        """
        return 1.0

    def __Phi_prime_prime(self, x = 0):
        """
            Linear output function
        """
        return 0.0

    """
        ---------------------------------------------------------------------
            Soloway, D. and P.J. Haley, "Neural Generalized Predictive Control,"
            Proceedings of the 1996 IEEE International Symposium on Intelligent
            Control, 1996, pp. 277-281.

            Calculating h'th element of the Jacobian
            Calculating m'th and h'th element of the Hessian
        ---------------------------------------------------------------------
    """

    def __partial_2_fnet_partial_nph_partial_npm(self, h, m, j):
        """
             D2^2f_j(net)
             ------------
            Du(n+h)Du(n+m)
        """
        return self.__Phi_prime()*self.__partial_2_net_partial_u_nph_partial_npm(h, m, j)\
                         + self.__Phi_prime_prime() * self.__partial_net_partial_u(h, j) * \
                                  self.__partial_net_partial_u(m, j)

    def __partial_2_yn_partial_nph_partial_npm(self, h, m, j):
        """
                 D^2yn
            ---------------
            Du(n+h) Du(n+m)

        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output=0.0

        for i in range(self.hid):
            sum_output+= np.mean(np.dot(weights[i, :], self.__partial_2_fnet_partial_nph_partial_npm(h, m, j)))

        self.previous_second_der = sum_output

        return sum_output

    def __partial_2_net_partial_u_nph_partial_npm(self, h, m, j):
        """
              D^2 net_j
            -------------
            Du(n+h)Du(n+m)
        """
        weights = self.model.layers[0].get_weights()[0]
        sum_output = 0.0
        for i in range(0, min(self.K, self.dd)):
            sum_output+= np.mean(weights[j, self.num_y*i+self.nd+1:self.num_y*i+self.nd+3] * \
                                        self.previous_second_der * step(self.K-i-1))
        return sum_output

    def __partial_yn_partial_u(self, h, j):
        """
               D yn
            -----------
             D u(n+h)
        """
        weights = self.model.layers[-1].get_weights()[0]
        sum_output = 0.0

        for i in range(self.hid):
            sum_output += weights[i, j] * self.__partial_fnet_partial_u( h, j)
        self.previous_first_der = sum_output
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
        weights = self.model.layers[0].get_weights()[0]
        sum_output = 0.0

        for i in range(self.nd):
            if (self.K - self.Nu) < i:
                sum_output+= np.mean(weights[j, i*self.num_u:i*self.num_u + self.num_u] * \
                                                                kronecker_delta(self.K - i, h))
            else:
                sum_output += np.mean(weights[j, self.num_u*i:self.num_u*i+self.num_u] *\
                                                                kronecker_delta(self.Nu, h))

        for i in range(0, min(self.K, self.dd)):
            sum_output += np.mean(weights[j, i*self.num_y + self.nd + 1] * self.previous_first_der * \
                                                step(self.K - i -1))
        return sum_output

    def __partial_delta_u_partial_u(self, j, h):
        """
            D delta u
            ---------
            D u(n+h)

        """
        return kronecker_delta(h, j) - kronecker_delta(h, j-1)

    def compute_hessian(self):
        Y, YM , U, delU = self.get_computation_vectors()
        Hessian = np.zeros((self.Nu, self.Nu))

        for h in range(self.Nu):
            for m in range(self.Nu):
                sum_output = 0.0

                for j in range(self.N1, self.N2):
                    sum_output += np.mean(2.*(self.__partial_yn_partial_u(h, j)*\
                                self.__partial_yn_partial_u(m, j) - \
                                self.__partial_2_yn_partial_nph_partial_npm(h, m, j)* \
                                    (YM[j, :] - Y[j, :])))

                for j in range(self.Nu):
                    sum_output += np.mean(2.*(self.lambd[j] * (self.__partial_delta_u_partial_u(j, h) * \
                                        self.__partial_delta_u_partial_u(j, m))))

                for j in range(self.Nu):
                    sum_output += np.mean(kronecker_delta(h, j) * kronecker_delta(m, j) * \
                            (np.divide(2.0*self.constraints.s , np.power(U[j, :] + self.constraints.r/2. - \
                                    self.constraints.b, 3)) + np.divide(2.*self.constraints.s , \
                                        (self.constraints.r/2. + self.constraints.b - \
                                                np.power(U[j, :], 3)))))
                Hessian[m, h] = sum_output
        return Hessian

    def compute_jacobian(self):
        Y, YM, U, delU = self.get_computation_vectors()

        dJ = np.zeros((self.Nu, U.shape[1]))

        for inp_ in range(U.shape[1]):
            for h in range(self.Nu):
                sum_output = np.array([0.0]*U.shape[1])

                for j in range(self.N1, self.N2):
                    sum_output[inp_]+= np.mean(-2.*(YM[j, :]- Y[j, :])*self.__partial_yn_partial_u(h, j))

                for j in range(self.Nu):
                    sum_output[inp_]+= np.mean(2.*self.lambd[j]*delU[j, inp_]*\
                                self.__partial_delta_u_partial_u(j, h))

                for j in range(self.Nu):
                    sum_output[inp_] += np.mean(kronecker_delta(h, j) * \
                                (-self.constraints.s/(U[j, inp_] + \
                                    self.constraints.r/2. - self.constraints.b)**2  + \
                                        self.constraints.s / (self.constraints.r/2. + \
                                            self.constraints.b - U[j, inp_])**2))

                dJ[:, inp_] = sum_output[inp_].tolist()
        return dJ

    def Fu(self):
        jacobian = self.compute_jacobian()
        return jacobian

    def Ju(self):
        self.Hessian = self.compute_hessian()
        return self.Hessian

    def compute_cost(self):
        return self.Cost.compute_cost()

    def measure(self, u):
        pass

    def predict(self, x):
        return self.model.predict(x, batch_size=1)





