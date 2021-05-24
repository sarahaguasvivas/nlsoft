from python.controller.recursive_model import *
import python.controller.constraints as constraints
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
        self.cost= 0.0

    def compute_cost(self, delU):
        """
        del_u is a list of the element wise differences between current and
        previous control inputs
        n is an int that represents the current discrete timestep
        """
        self.cost = 0.0

        Y, YM  = self.d_model.get_computation_vectors()
        delY = YM - Y
        for j in range(self.d_model.n1, self.d_model.n2):
            self.cost += delY[j, :].dot(self.d_model.Q).dot(delY[j, :].T)

        for j in range(self.d_model.nu):
            self.cost += np.array(delU[j, :]).dot(np.array(self.d_model.Lambda)).dot(np.array(delU[j, :]).T)

        return self.cost


