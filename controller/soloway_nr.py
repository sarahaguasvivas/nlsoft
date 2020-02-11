import numpy as np
from newton_raphson import *
from cost import *
from dynamic_model import *
import copy

class SolowayNR:

    def __init__(self, cost, d_model):
        self.cost = cost
        self.d_model = d_model

    def __fsolve_newton(self, u0, del_u, verbose=False):

        Y, YM, U, delU = self.d_model.get_computation_vectors()

        Ju = self.d_model.Ju()
        Fu = self.d_model.Fu()

        delU[:, 0] = np.linalg.solve(Ju, -Fu[:, 0])
        delU[:, 1] = np.linalg.solve(Ju, -Fu[:, 1])

        U += delU

        return U, delU

    def optimize(self, u = [0, 0], del_u=[0,0], verbose=False):
       return self.__fsolve_newton(u, del_u, verbose)
