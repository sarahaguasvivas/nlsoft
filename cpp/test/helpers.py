# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.1
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _helpers
else:
    import _helpers

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)



def activate(arg1, arg2, arg3):
    return _helpers.activate(arg1, arg2, arg3)

def sigmoid(arg1, arg2):
    return _helpers.sigmoid(arg1, arg2)

def softplus(arg1, arg2):
    return _helpers.softplus(arg1, arg2)

def softsign(arg1, arg2):
    return _helpers.softsign(arg1, arg2)

def hard_sigmoid(arg1, arg2):
    return _helpers.hard_sigmoid(arg1, arg2)

def exp_activation(arg1, arg2):
    return _helpers.exp_activation(arg1, arg2)

def exponential(arg1):
    return _helpers.exponential(arg1)

def relu(arg1, arg2):
    return _helpers.relu(arg1, arg2)

def elu(arg1, arg2, arg3):
    return _helpers.elu(arg1, arg2, arg3)

def selu(arg1, arg2):
    return _helpers.selu(arg1, arg2)

def hyper_tan(arg1, arg2):
    return _helpers.hyper_tan(arg1, arg2)

def softmax(arg1, arg2):
    return _helpers.softmax(arg1, arg2)
PI = _helpers.PI

def setup_nn_utils():
    return _helpers.setup_nn_utils()

def nn_gradients(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8):
    return _helpers.nn_gradients(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8)

def roll_window(arg1, arg2, arg3, arg4):
    return _helpers.roll_window(arg1, arg2, arg3, arg4)

def normalize_array(arg1, arg2, arg3, arg4):
    return _helpers.normalize_array(arg1, arg2, arg3, arg4)

def nn_prediction(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
    return _helpers.nn_prediction(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9)

def build_input_vector(arg1, u, signal_, posish, ndm, ddn, m, n, arg9):
    return _helpers.build_input_vector(arg1, u, signal_, posish, ndm, ddn, m, n, arg9)

def partial_delta_u_partial_u(arg1, arg2):
    return _helpers.partial_delta_u_partial_u(arg1, arg2)

def kronecker_delta(arg1, arg2):
    return _helpers.kronecker_delta(arg1, arg2)

def get_jacobian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
    return _helpers.get_jacobian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9)

def get_hessian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
    return _helpers.get_hessian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9)

def solve(arg1, arg2, arg3):
    return _helpers.solve(arg1, arg2, arg3)

def clip_action(arg1, arg2):
    return _helpers.clip_action(arg1, arg2)
class Dense(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    weights = property(_helpers.Dense_weights_get, _helpers.Dense_weights_set)
    biases = property(_helpers.Dense_biases_get, _helpers.Dense_biases_set)
    activation = property(_helpers.Dense_activation_get, _helpers.Dense_activation_set)
    weight_shape = property(_helpers.Dense_weight_shape_get, _helpers.Dense_weight_shape_set)
    input_shape = property(_helpers.Dense_input_shape_get, _helpers.Dense_input_shape_set)
    output_shape = property(_helpers.Dense_output_shape_get, _helpers.Dense_output_shape_set)

    def __init__(self):
        _helpers.Dense_swiginit(self, _helpers.new_Dense())
    __swig_destroy__ = _helpers.delete_Dense

# Register Dense in _helpers:
_helpers.Dense_swigregister(Dense)
cvar = _helpers.cvar
dense_19_W = cvar.dense_19_W
dense_19_b = cvar.dense_19_b
dense_20_W = cvar.dense_20_W
dense_20_b = cvar.dense_20_b


def cleanup(arg1):
    return _helpers.cleanup(arg1)

def build_layer_dense(arg1, arg2, arg3, arg4, arg5):
    return _helpers.build_layer_dense(arg1, arg2, arg3, arg4, arg5)

def fwd_dense(arg1, arg2):
    return _helpers.fwd_dense(arg1, arg2)
class Matrix2(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    rows = property(_helpers.Matrix2_rows_get, _helpers.Matrix2_rows_set)
    cols = property(_helpers.Matrix2_cols_get, _helpers.Matrix2_cols_set)
    data = property(_helpers.Matrix2_data_get, _helpers.Matrix2_data_set)
    tiny = property(_helpers.Matrix2_tiny_get, _helpers.Matrix2_tiny_set)

    def __init__(self):
        _helpers.Matrix2_swiginit(self, _helpers.new_Matrix2())
    __swig_destroy__ = _helpers.delete_Matrix2

# Register Matrix2 in _helpers:
_helpers.Matrix2_swigregister(Matrix2)


def set(arg1, arg2, arg3):
    return _helpers.set(arg1, arg2, arg3)

def set_to_zero(arg1):
    return _helpers.set_to_zero(arg1)

def set_to_ones(arg1):
    return _helpers.set_to_ones(arg1)

def sum_axis(arg1, arg2):
    return _helpers.sum_axis(arg1, arg2)

def transpose(arg1):
    return _helpers.transpose(arg1)

def scale(arg1, arg2):
    return _helpers.scale(arg1, arg2)

def add(arg1, arg2):
    return _helpers.add(arg1, arg2)

def subtract(arg1, arg2):
    return _helpers.subtract(arg1, arg2)

def multiply(arg1, arg2):
    return _helpers.multiply(arg1, arg2)

def hadamard(arg1, arg2):
    return _helpers.hadamard(arg1, arg2)

def inverse(arg1):
    return _helpers.inverse(arg1)

def release(a):
    return _helpers.release(a)

def ludcmp(arg1, arg2, arg3):
    return _helpers.ludcmp(arg1, arg2, arg3)

def repmat(arg1, arg2, arg3):
    return _helpers.repmat(arg1, arg2, arg3)

def equal(arg1, arg2):
    return _helpers.equal(arg1, arg2)

def nr_optimizer(arg1, arg2, arg3):
    return _helpers.nr_optimizer(arg1, arg2, arg3)

def solve_matrix_eqn(arg1, arg2):
    return _helpers.solve_matrix_eqn(arg1, arg2)

def cleanup_helper(arg1):
    return _helpers.cleanup_helper(arg1)

def deg2rad(deg):
    return _helpers.deg2rad(deg)

def lubksb(*args):
    return _helpers.lubksb(*args)
class input(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self, nelements):
        _helpers.input_swiginit(self, _helpers.new_input(nelements))
    __swig_destroy__ = _helpers.delete_input

    def __getitem__(self, index):
        return _helpers.input___getitem__(self, index)

    def __setitem__(self, index, value):
        return _helpers.input___setitem__(self, index, value)

    def cast(self):
        return _helpers.input_cast(self)

    @staticmethod
    def frompointer(t):
        return _helpers.input_frompointer(t)

# Register input in _helpers:
_helpers.input_swigregister(input)

def input_frompointer(t):
    return _helpers.input_frompointer(t)



