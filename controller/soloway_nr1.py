from scipy import optimize
from newton_raphson import *
from dynamic_model import *


class SolowayNR1:
    def __init__(self, cost, d_model):
        self.cost = cost
        self.d_model = d_model

    def function(self, u):
        Fu = self.d_model.Fu()
        Ju = self.d_model.Ju()

        delU = np.dot(np.dot(Ju, u), Fu)

        return delU

    def __fsolve_newton(self, u0, del_u, maxit = 8, rtol = 1e-8, verbose=False):
        Y, YM, U, delU = self.d_model.get_computation_vectors()
        delU = optimize.newton(self.function, U)
        U += delU
        return U, delU

    def optimize(self, u = [0, 0], del_u=[0,0], maxit = 8, rtol = 1e-8, verbose=False):
       return self.__fsolve_newton(u0 = u, del_u = del_u,maxit = maxit, rtol = rtol,  verbose = verbose)
