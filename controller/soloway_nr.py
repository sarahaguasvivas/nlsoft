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
        u = u0
        du = del_u

        Ju = self.d_model.Ju(u, du)
        Fu = self.d_model.Fu(u, du)

        Ju1 = Ju[0, :, :]
        Fu1 = Fu[0, :]

        Ju2 = Ju[1, :, :]
        Fu2 = Fu[1, :]

        print Ju1, Fu1, Ju2, Fu2

        du1 = np.linalg.solve(Ju1, -Fu1)

        du2 = np.linalg.solve(Ju2, -Fu2)

        print du1, du2

        u[:, 0] += du1
        u[:, 1] += du2

        return u, du

    def optimize(self, u = [0, 0], del_u=[0,0], verbose=False):
       return self.__fsolve_newton(u, del_u, verbose)
