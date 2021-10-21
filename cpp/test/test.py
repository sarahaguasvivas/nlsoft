import helpers
import os, sys
import unittest
sys.path.append(os.path.join(os.environ['HOME'], 'nlsoft'))
from python.controller.recursive_model import *
from python.controller.soloway_nr import *
from python.utilities.util import *
import numpy as np
import ctypes
from typing import List, Final
from collections import deque

model_filename = str(os.environ["HOME"]) + "/nlsoft/python/test/sys_id.hdf5"
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

def python_nn_prediction(nn_input, ydeq, N):
    NNP.yn = []
    nn_input = nn_input.copy()
    for k in range(N):
        flatten_ydeq = list(np.array(list(ydeq.copy())).flatten())
        print(nn_input)
        predicted_states = NNP.predict(nn_input).flatten().copy()
        NNP.yn += [(NNP.C @ predicted_states).tolist()]
        ydeq = roll_deque(ydeq.copy(), predicted_states.tolist().copy())
        for i in range(3 * 3):
            nn_input[0][i + 2 * 3] = flatten_ydeq[i]
    return NNP.yn

class TestUtilities(unittest.TestCase):
    def test_prediction(self):
        for _ in range(NUM_TESTS):
            pred_size = (1, 26)
            N = np.random.randint(1, 10, size = 1)[0]
            print(N)
            prediction_data = np.random.normal(-10.0, 10.0, size = pred_size)
            nn_input = list_2_swig_float_pointer(
                                            prediction_data.flatten().tolist(),
                                            prediction_data.size
                                            )
            input_data = prediction_data[0, :2]
            motor_inputs = list_2_swig_float_pointer(input_data.flatten().tolist(), 2)
            helpers.setup_nn_utils()
            c_prediction = helpers.nn_prediction(int(N), 1, 3, 2, prediction_data.size,
                                                 3, 3, nn_input, motor_inputs)
            c_output_prediction = np.reshape(swig_py_object_2_list(c_prediction.data, N * 3), (N, 3))
            y_deq = prediction_data[0, 2*3:2*3 + 3*3]
            ydeq = deque([])
            for i in range(3):
                ydeq.append(y_deq[i:i+3])
            python_prediction = python_nn_prediction(prediction_data, ydeq, N)
            np.testing.assert_allclose(np.reshape(c_output_prediction, (N, 3)),
                                       np.reshape(python_prediction, (N, 3)),
                                       atol = 1e-5, rtol = 1e-5)
            print("test passed!")
    def test_deg2rad(self):
        for _ in range(NUM_TESTS):
            val = np.random.normal(-100, 50, size = 1)[0]
            np.testing.assert_allclose(helpers.deg2rad(val), np.deg2rad(val), rtol = 1e-5)

if __name__ == '__main__':
    unittest.main()