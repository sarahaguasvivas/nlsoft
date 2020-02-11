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

#sess = tf.compat.v1.global_variables_initializer()
#sess.run(session = sess)


class ModelException(Exception):
    pass

class NeuralNetworkPredictor():
    def __init__(self, model_file, N1 = 1 , N2 = 5 ,  Nu = 3 , \
                            K = 0.7 , lambd = [0.3, 0.2, 0.3] , nd = 3,\
                                                dd = 3, y0= [0, 0, 0], u0= [0, 0]):
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

        self.output_size = self.model.layers[-1].output_shape[1]
        self.input_size = self.model.layers[0].input_shape[1]

        self.hd = len(self.model.layers) - 1
        self.nd = nd
        self.dd = dd

        self.Hessian = np.zeros((self.output_size, self.output_size))

        # Important for recursions:
        self.previous_first_der = 1
        self.previous_second_der = 1

        # Initializing deques
        self.y_deque = deque()
        self.delu_deque = deque()
        self.u_deque = deque()
        self.ym_deque = deque()

        self.initialize_deques(self.u0, self.y0)
        self.Cost = NN_Cost(self, self.lambd)

    def initialize_deques(self, u0, y0):
        for i in range(self.N2):
            self.y_deque.appendleft(self.y0)
            self.ym_deque.appendleft(y0) # get wand location

        for _ in range(self.Nu):
            self.u_deque.appendleft(u0)
            self.delu_deque.appendleft([0, 0])

    def update_dynamics(u = [0, -50], del_u = [0, 0], y = [0, 0, 0], ym = [0, 0, 0]):
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
        return self._Phi_prime()*self.__partial_2_net_partial_u_nph_partial_npm(h, m, j)*\
                         + self.__Phi_prime_prime() * self.__partial_net_partial_u(h, j) * \
                                  self.__partial_net_partial_u(m, j)

    def __partial_2_yn_partial_nph_partial_npm(self, h, m, j):
        """
                 D^2yn
            ---------------
            Du(n+h) Du(n+m)

        """
        weights = self.model.layers[j].get_weights()[0]
        hid = weights.shape[1]
        sum_output=0.0
        for i in range(hid):
            sum_output+= weights[j,i] * self.__partial_2_fnet_partial_nph_partial_npm(h, m, j)

        self.previous_second_der = sum_output
        return sum_output

    def __partial_2_net_partial_u_nph_partial_npm(self, h, m, j):
        """
              D^2 net_j
            -------------
            Du(n+h)Du(n+m)
        """
        weights = self.model.layers[j].get_weights()[0]
        sum_output=0.0
        for i in range(1, min(self.K, self.dd)):
            sum_output+= weights[j, i+self.nd+1] * self.previous_second_der * step(self.K-i-1)
        return sum_output

    def __partial_yn_partial_u(self, h, j):
        """
               D yn
            -----------
             D u(n+h)
        """
        weights = self.model.layers[j].get_weights()[0]
        hid = self.model.layers[j].output_shape[1]
        sum_output = 0.0
        for i in range(hid):
            sum_output += weights[j, i] * self.__partial_fnet_partial_u( h, j)

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

        weights = self.model.layers[j].get_weights()[0]
        sum_output = 0.0

        for i in range(self.nd):
            if (self.K - self.Nu) < i:
                sum_output+= weights[j, i] * kronecker_delta(self.K - i, h)
            else:
                sum_output+=weights[j, i] * kronecker_delta(self.Nu, h)

        for i in range(0, min(self.K, self.dd)-1):
            sum_output+= weights[j, i+self.nd ] * self.previous_first_der * \
                                                 step(self.K - i -1)
        return sum_output


    def __partial_delta_u_partial_u(self, j, h):
        """
            D delta u
            ---------
            D u(n+h)

        """
        return kronecker_delta(h, j) - kronecker_delta(h, j-1)

    def compute_hessian(self, u, del_u):
        Hessian = np.zeros((self.Nu, self.Nu))

        Y = np.array(list(self.y_deque))
        YM = np.array(list(self.ym_deque))

        for h in range(self.Nu):
            for m in range(self.Nu):
                sum_output=0.0

                for j in range(self.N1, self.N2):
                    sum_output += 2.*(self.__partial_yn_partial_u(h, j)*self.__partial_yn_partial_u(m, j) - \
                                        self.__partial_2_yn_partial_nph_partial_npm(h, m, j)* \
                                        (self.y_deque[j] - self.yn_deque[j]))

                for j in range(self.Nu):
                    sum_output += 2.*( self.lambd[j] * (self.__partial_delta_u_partial_u(j, h) * self.__partial_delta_u_partial_u(j, m) + del_u[j] * 0.0))


                for j in range(self.Nu):
                    sum_output += kronecker_delta(h, j) * kronecker_delta(m, j) * \
                                        ( 2.0*self.constraints.s/( u[j] + self.constraints.r/2. - \
                                                self.constraints.b)**3 + 2.*self.constraints.s/(self.constraints.r/2. + \
                                                                    self.constraints.b - u[j])**3)

                Hessian[m, h] = sum_output

        return Hessian

    def compute_jacobian(self, u, del_u):
        # working on this now
        dJ = []

        Y = np.array(list(self.y_deque))
        YM = np.array(list(self.ym_deque))
        delU = np.array(list(self.delu_deque))
        U = np.array(list(self.u_deque))

        for h in range(self.Nu):
            sum_output=0.0
            for j in range(self.N1, self.N2):
                sum_output+=-2.*(YM[j, :]- Y[j, :])*self.__partial_yn_partial_u(h, j)

            for j in range(self.Nu):
                sum_output+=2.*self.lambd[j]*delU[j, :]*self.__partial_delta_u_partial_u(j, h)

            for j in range(self.Nu):
                sum_output+=kronecker_delta(h, j) * ( -self.constraints.s/(u[j] + self.constraints.r/2. - self.constraints.b)**2  + \
                                            self.constraints.s / (self.constraints.r/2. + self.constraints.b - U[j])**2    )

            dJ+=[sum_output]
        return dJ

    def Fu(self, u, del_u):
        jacobian = self.compute_jacobian(u, del_u)
        return jacobian

    def Ju(self, u, del_u):
        self.Hessian = self.compute_hessian(u, del_u)
        return self.Hessian

    def compute_cost(self, del_u, u):
        return self.Cost.compute_cost(del_u, u)

    def measure(self, u):
        pass

    def predict(self, x):
        return self.model.predict(x, batch_size=1)





