from dynamic_model import *
import constraints as constraints
import numpy as np

class NN_Cost:
    def __init__(self, dynamic_model):
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
        delY = YM - Y
        for j in range(self.d_model.N1, self.d_model.N2):
            self.cost += delY[j, :].dot(self.d_model.Q).dot(delY[j, :].T)

        for j in range(self.d_model.Nu):
            self.cost += np.array(delU[j, :]).dot(np.array(self.d_model.Lambda)).dot(np.array(delU[j, :]).T)

        for j in range(self.d_model.Nu):
            for i in range(self.d_model.nu):
                self.cost += self.s / (U[j, i] + self.r / 2.0 - \
                            self.b) + self.s / (self.r/2.0 + \
                                self.b - U[j, i]) - 4.0 / self.r
        return self.cost


