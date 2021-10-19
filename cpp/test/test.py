import helpers
import os, sys
import unittest
sys.path.append(os.path.join(os.environ['HOME'], 'github_repos', 'nlsoft'))
from python.controller.recursive_model import *
from python.controller.soloway_nr import *
from python.utilities.util import *
import numpy as np
import ctypes
from typing import List, Final

model_filename = str(os.environ["HOME"]) + "/github_repos/nlsoft/python/test/sys_id.hdf5"
NUM_TESTS = 1000
NN_INPUT_SIZE = 26

def swig_py_object_2_list(object, size : int) -> List[float]:
    """
        Converts SwigPyObject to List[float]
    """
    y = (ctypes.c_float * size).from_address(int(object))
    new_object = []
    for i in range(size):
        new_object += [float(y[i])]
    return new_object

def swig_py_object_2_list_int(object, size : int) -> List[int]:
    """
        Converts SwigPyObject to List[float]
    """
    y = (ctypes.c_float * size).from_address(int(object))
    new_object = []
    for i in range(size):
        new_object += [int(y[i])]
    return new_object

def list_2_swig_float_pointer(list : List[float], size : int):
    """
        Converts from list of floats to swig float* object
    """
    test_buffer = helpers.input(size)
    for i in range(size):
        test_buffer[i] = float(list[i])
    return test_buffer

NNP = RecursiveNeuralNetworkPredictor(model_file = model_filename,
                                      N1 = 0, N2 = 2, Nu = 1,
                                      nd = 3, dd = 3, K = 2,
                                      Q = np.array([[1e-3, 0., 0],
                                                    [0., 1e3, 0],
                                                    [0., 0., 1e3]]),
                                      Lambda = np.array([[1., 0.],
                                                         [0., 1.]]),
                                      s = 1e-20, b = 1e-5, r = 4e3,
                                      states_to_control = [1, 1, 1],
                                      y0= [0.0, 0.0, 0.0],
                                      u0 = [np.deg2rad(-70.), np.deg2rad(-50.)],
                                      step_size = 5e-2)

def python_nn_prediction(nn_input, u, N, n, ydeq):
    NNP.yn = []
    for k in range(NNP.K):
        predicted_states = NNP.predict(nn_input).flatten()
        NNP.yn += [(NNP.C @ predicted_states).tolist()]
        ydeq = roll_deque(ydeq, predicted_states.tolist())
        flatten_ydeq = np.array(list(ydeq)).flatten().tolist()
        for i in range(3*3):
            nn_input[0, i+2] = flatten_ydeq[i]
    return NNP.yn

class TestUtilities(unittest.TestCase):
    def test_prediction(self):
        for _ in range(NUM_TESTS):
            pred_size = (1, 26)
            #prediction_data = np.array([0.2777778804,-0.5555555224,0.2777778804,-0.5555555224,0.2777778804,-0.5555555224,
            #                   -0.0251509957,-0.0251509957,1.9656401873,-0.0251509957,-0.0251509957,1.9656401873,
            #                   -0.0251509957,-0.0251509957,1.9656401873,0.0000000000,0.0000000000,0.0000000000,
            #                   0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,
            #                   0.0000000000,0.0000000000]).reshape(pred_size)
            prediction_data = np.random.normal(-10.0, 10.0, size = pred_size)
            nn_input = list_2_swig_float_pointer(
                                            prediction_data.flatten().tolist(),
                                            prediction_data.size
                                            )
            input_data = prediction_data[0, :2]
            motor_inputs = list_2_swig_float_pointer(input_data.flatten().tolist(), 2)
            helpers.setup_nn_utils();
            c_prediction = helpers.nn_prediction(2, 1, 3, 2, prediction_data.size,
                                                 3, 3, nn_input, motor_inputs)
            print(c_prediction.rows, c_prediction.cols, c_prediction.tiny, c_prediction.data)
            c_output_prediction = np.reshape(swig_py_object_2_list(c_prediction.data, 2 * 3), (2, 3))
            _, ydeq = first_load_deques(prediction_data[0, 2:2+3].tolist(), input_data.tolist(), 3, 3)
            python_prediction = python_nn_prediction(prediction_data, input_data, 2, 3, ydeq)
            np.testing.assert_allclose(np.reshape(c_output_prediction, (2, 3)),
                                       np.reshape(python_prediction, (2, 3)), rtol = 1e-5)

if __name__ == '__main__':
    unittest.main()