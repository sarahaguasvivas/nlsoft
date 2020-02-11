import numpy as np
from newton_raphson import *
from cost import *
from dynamic_model import *
import copy

class SolowayNR:

    def __init__(self, cost, d_model):
        self.cost = cost
        self.d_model = d_model

    def __fsolve_newton(self, u0, del_u, maxit = 8, rtol = 1e-8, verbose=False):

        Y, YM, U, delU = self.d_model.get_computation_vectors()

        Fu = self.d_model.Fu()
        norm0 = np.linalg.norm(Fu)
        enorm_last= np.linalg.norm(U - np.ones(U.shape))

#        for i in range(maxit):
        Ju = self.d_model.Ju()
        delU =  -np.dot(np.linalg.inv(Ju),  Fu)
        U += delU
        Fu = self.d_model.Fu()
        norm = np.linalg.norm(Fu)

        if verbose:
            enorm = np.linalg.norm(U - np.ones(U.shape))
            print "Newton: ", i, " anorm: ", norm, " rnorm: ", norm/norm0, " eratio: ", enorm/enorm_last**2
            enorm_last = enorm

           # if norm < rtol * norm0:
           #     break

        return U, delU

    def optimize(self, u = [0, 0], del_u=[0,0], verbose=False):
       return self.__fsolve_newton(u0 = u, del_u = del_u,  verbose = verbose)
