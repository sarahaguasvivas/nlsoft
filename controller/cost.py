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
        self.N1 = dynamic_model.N1
        self.N2 = dynamic_model.N2
        self.Nu = dynamic_model.Nu
        self.ym = dynamic_model.ym
        self.yn = dynamic_model.yn
        self.lambd = dynamic_model.lambd
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
        #print("ym: ", self.d_model.ym, "yn: ", self.d_model.yn)

        self.cost = 0.0

        Y, YM , U, delU = self.d_model.get_computation_vectors()

        for j in range(self.N1, self.N2):
            self.cost += np.mean(np.power(YM[j, :] - Y[j, :], 2))

        for j in range(self.Nu):
            self.cost += np.mean(self.lambd[j]*np.power(delU[j, :], 2))

        for j in range(self.Nu):
            self.cost += np.mean(self.s / (U[j, :] + self.r / 2.0 - self.b) + \
                                        self.s / (self.r/2.0 + self.b - U[j, :]) - 4.0 / self.r)

        return self.cost


