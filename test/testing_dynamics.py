from controller.dynamic_model import *
import unittest
import os, sys

class NN_test(unittest.TestCase):
    self.model_filename = str(os.environ["HOME"]) + '/gpc_controller/test/sys_id.hdf5'
    self.dynamics = NeuralNetworkPredictor(model_file = self.model_filename)

    def test_initialize_deques(self):
        self.dynamics(initialize_deques(self.dynamics.u0, \
                        self.dynamics.y0))

        print self.dynamics.u_deque, self.dynamics.y_deque
if __name__ == '__main__':
    unittest.main()


