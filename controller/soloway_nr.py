import numpy as np
from cost import *
from dynamic_model import *
from scipy import optimize
import copy

class SolowayNR:

    def __init__(self, cost, d_model):
        self.cost = cost
        self.d_model = d_model

    def __fsolve_newton(self, u0, rtol=1e-10, maxit=50, verbose=False):
        u = np.array(u0).copy()

        del_u = np.zeros(u.shape)

        Fu = -self.d_model.Fu(u, del_u)

        norm0 = np.linalg.norm(Fu)
        enorm_last = np.linalg.norm(u - np.array([1,1]))

        for i in range(maxit):
            du = -np.linalg.solve(self.d_model.Ju(u, del_u), Fu)
            u += du
            del_u = du
            Fu = -self.d_model.Fu(u, del_u)
            norm = np.linalg.norm(Fu)
            if verbose:
                enorm = np.linalg.norm(u - np.array([1,1]))
                print('Newton {:d} anorm {:6.2e} rnorm {:6.2e} eratio {:6.2f}'.
                format(i+1, norm, norm/norm0, enorm/enorm_last**2))
                enorm_last = enorm
            if norm < rtol * norm0:
                break
        return u, del_u, i


    def optimize(self, u, maxit = 1, rtol = 1e-8, verbose=False):
       return self.__fsolve_newton(u0 = u, rtol = rtol, maxit = maxit,  verbose = verbose)
