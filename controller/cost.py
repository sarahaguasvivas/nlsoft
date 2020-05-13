from dynamic_model import *
import constraints as constraints
import numpy as np

class NN_Cost:
    def __init__(self, dynamic_model, lambd):
        """
        N1      : minimum costing horizon
        N2      : maximum costing horizon
        Nu      : control horizon
        ym      : reference trajectory
        lambd   : control input weighting factor (damper for u(n+1))
        s       : sharpness corners of constraint function
        r       : range of constraint
        b       : offset of the range
        """
        self.d_model = dynamic_model
        self.s = self.d_model.constraints.s
        self.r = self.d_model.constraints.r
        self.b = self.d_model.constraints.b
        self.cost= 0.0

    def compute_cost(self):
        """
        del_u is a list of the element wise differences between current and
        previous control inputs
        n is an int that represents the current discrete timestep
        """
        self.cost = 0.0

        Y, YM , U, delU = self.d_model.get_computation_vectors()

        for j in range(self.d_model.N1, self.d_model.N2):
            self.cost += np.dot(YM[j, :] - Y[j, :], YM[j, :] - Y[j, :])
        for j in range(self.d_model.Nu):
            self.cost += np.dot(np.array(self.d_model.lambd[:, j]), \
                                np.array(delU[j, :]) * np.array(delU[j, :]))
        for j in range(self.d_model.Nu):
            for i in range(self.d_model.num_u):
                self.cost += np.sum(self.s / (U[j, i] + self.r / 2.0 - \
                            self.b) + self.s / (self.r/2.0 + \
                                self.b - U[j, i]) - 4.0 / self.r)
        return self.cost


