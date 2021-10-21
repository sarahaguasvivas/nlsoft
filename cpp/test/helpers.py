# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_helpers', [dirname(__file__)])
        except ImportError:
            import _helpers
            return _helpers
        if fp is not None:
            try:
                _mod = imp.load_module('_helpers', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _helpers = swig_import_helper()
    del swig_import_helper
else:
    import _helpers
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0



def activate(arg1, arg2, arg3):
    return _helpers.activate(arg1, arg2, arg3)
activate = _helpers.activate

def sigmoid(arg1, arg2):
    return _helpers.sigmoid(arg1, arg2)
sigmoid = _helpers.sigmoid

def softplus(arg1, arg2):
    return _helpers.softplus(arg1, arg2)
softplus = _helpers.softplus

def softsign(arg1, arg2):
    return _helpers.softsign(arg1, arg2)
softsign = _helpers.softsign

def hard_sigmoid(arg1, arg2):
    return _helpers.hard_sigmoid(arg1, arg2)
hard_sigmoid = _helpers.hard_sigmoid

def exp_activation(arg1, arg2):
    return _helpers.exp_activation(arg1, arg2)
exp_activation = _helpers.exp_activation

def exponential(arg1):
    return _helpers.exponential(arg1)
exponential = _helpers.exponential

def relu(arg1, arg2):
    return _helpers.relu(arg1, arg2)
relu = _helpers.relu

def elu(arg1, arg2, arg3):
    return _helpers.elu(arg1, arg2, arg3)
elu = _helpers.elu

def selu(arg1, arg2):
    return _helpers.selu(arg1, arg2)
selu = _helpers.selu

def hyper_tan(arg1, arg2):
    return _helpers.hyper_tan(arg1, arg2)
hyper_tan = _helpers.hyper_tan

def softmax(arg1, arg2):
    return _helpers.softmax(arg1, arg2)
softmax = _helpers.softmax

_helpers.PI_swigconstant(_helpers)
PI = _helpers.PI

def setup_nn_utils():
    return _helpers.setup_nn_utils()
setup_nn_utils = _helpers.setup_nn_utils

def nn_gradients(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8):
    return _helpers.nn_gradients(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8)
nn_gradients = _helpers.nn_gradients

def roll_window(arg1, arg2, arg3, arg4):
    return _helpers.roll_window(arg1, arg2, arg3, arg4)
roll_window = _helpers.roll_window

def normalize_array(arg1, arg2, arg3, arg4):
    return _helpers.normalize_array(arg1, arg2, arg3, arg4)
normalize_array = _helpers.normalize_array

def build_input_vector(arg1, u, signal_, posish, ndm, ddn, m, n, arg9):
    return _helpers.build_input_vector(arg1, u, signal_, posish, ndm, ddn, m, n, arg9)
build_input_vector = _helpers.build_input_vector

def partial_delta_u_partial_u(arg1, arg2):
    return _helpers.partial_delta_u_partial_u(arg1, arg2)
partial_delta_u_partial_u = _helpers.partial_delta_u_partial_u

def kronecker_delta(arg1, arg2):
    return _helpers.kronecker_delta(arg1, arg2)
kronecker_delta = _helpers.kronecker_delta

def get_jacobian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
    return _helpers.get_jacobian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9)
get_jacobian = _helpers.get_jacobian

def get_hessian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
    return _helpers.get_hessian(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9)
get_hessian = _helpers.get_hessian

def solve(arg1, arg2, arg3):
    return _helpers.solve(arg1, arg2, arg3)
solve = _helpers.solve

def clip_action(arg1, arg2):
    return _helpers.clip_action(arg1, arg2)
clip_action = _helpers.clip_action
class Dense(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, Dense, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, Dense, name)
    __repr__ = _swig_repr
    __swig_setmethods__["weights"] = _helpers.Dense_weights_set
    __swig_getmethods__["weights"] = _helpers.Dense_weights_get
    if _newclass:
        weights = _swig_property(_helpers.Dense_weights_get, _helpers.Dense_weights_set)
    __swig_setmethods__["biases"] = _helpers.Dense_biases_set
    __swig_getmethods__["biases"] = _helpers.Dense_biases_get
    if _newclass:
        biases = _swig_property(_helpers.Dense_biases_get, _helpers.Dense_biases_set)
    __swig_setmethods__["activation"] = _helpers.Dense_activation_set
    __swig_getmethods__["activation"] = _helpers.Dense_activation_get
    if _newclass:
        activation = _swig_property(_helpers.Dense_activation_get, _helpers.Dense_activation_set)
    __swig_setmethods__["weight_shape"] = _helpers.Dense_weight_shape_set
    __swig_getmethods__["weight_shape"] = _helpers.Dense_weight_shape_get
    if _newclass:
        weight_shape = _swig_property(_helpers.Dense_weight_shape_get, _helpers.Dense_weight_shape_set)
    __swig_setmethods__["input_shape"] = _helpers.Dense_input_shape_set
    __swig_getmethods__["input_shape"] = _helpers.Dense_input_shape_get
    if _newclass:
        input_shape = _swig_property(_helpers.Dense_input_shape_get, _helpers.Dense_input_shape_set)
    __swig_setmethods__["output_shape"] = _helpers.Dense_output_shape_set
    __swig_getmethods__["output_shape"] = _helpers.Dense_output_shape_get
    if _newclass:
        output_shape = _swig_property(_helpers.Dense_output_shape_get, _helpers.Dense_output_shape_set)

    def __init__(self):
        this = _helpers.new_Dense()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _helpers.delete_Dense
    __del__ = lambda self: None
Dense_swigregister = _helpers.Dense_swigregister
Dense_swigregister(Dense)
cvar = _helpers.cvar
dense_19_W = cvar.dense_19_W
dense_19_b = cvar.dense_19_b
dense_20_W = cvar.dense_20_W
dense_20_b = cvar.dense_20_b


def cleanup(arg1):
    return _helpers.cleanup(arg1)
cleanup = _helpers.cleanup

def build_layer_dense(arg1, arg2, arg3, arg4, arg5):
    return _helpers.build_layer_dense(arg1, arg2, arg3, arg4, arg5)
build_layer_dense = _helpers.build_layer_dense

def fwd_dense(arg1, arg2):
    return _helpers.fwd_dense(arg1, arg2)
fwd_dense = _helpers.fwd_dense
class Matrix2(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, Matrix2, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, Matrix2, name)
    __repr__ = _swig_repr
    __swig_setmethods__["rows"] = _helpers.Matrix2_rows_set
    __swig_getmethods__["rows"] = _helpers.Matrix2_rows_get
    if _newclass:
        rows = _swig_property(_helpers.Matrix2_rows_get, _helpers.Matrix2_rows_set)
    __swig_setmethods__["cols"] = _helpers.Matrix2_cols_set
    __swig_getmethods__["cols"] = _helpers.Matrix2_cols_get
    if _newclass:
        cols = _swig_property(_helpers.Matrix2_cols_get, _helpers.Matrix2_cols_set)
    __swig_setmethods__["data"] = _helpers.Matrix2_data_set
    __swig_getmethods__["data"] = _helpers.Matrix2_data_get
    if _newclass:
        data = _swig_property(_helpers.Matrix2_data_get, _helpers.Matrix2_data_set)
    __swig_setmethods__["tiny"] = _helpers.Matrix2_tiny_set
    __swig_getmethods__["tiny"] = _helpers.Matrix2_tiny_get
    if _newclass:
        tiny = _swig_property(_helpers.Matrix2_tiny_get, _helpers.Matrix2_tiny_set)

    def __init__(self):
        this = _helpers.new_Matrix2()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _helpers.delete_Matrix2
    __del__ = lambda self: None
Matrix2_swigregister = _helpers.Matrix2_swigregister
Matrix2_swigregister(Matrix2)


def set(arg1, arg2, arg3):
    return _helpers.set(arg1, arg2, arg3)
set = _helpers.set

def set_to_zero(arg1):
    return _helpers.set_to_zero(arg1)
set_to_zero = _helpers.set_to_zero

def set_to_ones(arg1):
    return _helpers.set_to_ones(arg1)
set_to_ones = _helpers.set_to_ones

def sum_axis(arg1, arg2):
    return _helpers.sum_axis(arg1, arg2)
sum_axis = _helpers.sum_axis

def transpose(arg1):
    return _helpers.transpose(arg1)
transpose = _helpers.transpose

def scale(arg1, arg2):
    return _helpers.scale(arg1, arg2)
scale = _helpers.scale

def add(arg1, arg2):
    return _helpers.add(arg1, arg2)
add = _helpers.add

def subtract(arg1, arg2):
    return _helpers.subtract(arg1, arg2)
subtract = _helpers.subtract

def multiply(arg1, arg2):
    return _helpers.multiply(arg1, arg2)
multiply = _helpers.multiply

def hadamard(arg1, arg2):
    return _helpers.hadamard(arg1, arg2)
hadamard = _helpers.hadamard

def inverse(arg1):
    return _helpers.inverse(arg1)
inverse = _helpers.inverse

def release(a):
    return _helpers.release(a)
release = _helpers.release

def ludcmp(arg1, arg2, arg3):
    return _helpers.ludcmp(arg1, arg2, arg3)
ludcmp = _helpers.ludcmp

def repmat(arg1, arg2, arg3):
    return _helpers.repmat(arg1, arg2, arg3)
repmat = _helpers.repmat

def equal(arg1, arg2):
    return _helpers.equal(arg1, arg2)
equal = _helpers.equal

def nr_optimizer(arg1, arg2, arg3):
    return _helpers.nr_optimizer(arg1, arg2, arg3)
nr_optimizer = _helpers.nr_optimizer

def solve_matrix_eqn(arg1, arg2):
    return _helpers.solve_matrix_eqn(arg1, arg2)
solve_matrix_eqn = _helpers.solve_matrix_eqn

def cleanup_helper(arg1):
    return _helpers.cleanup_helper(arg1)
cleanup_helper = _helpers.cleanup_helper

def deg2rad(deg):
    return _helpers.deg2rad(deg)
deg2rad = _helpers.deg2rad

def nn_prediction(*args):
    return _helpers.nn_prediction(*args)
nn_prediction = _helpers.nn_prediction

def lubksb(*args):
    return _helpers.lubksb(*args)
lubksb = _helpers.lubksb
class input(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, input, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, input, name)
    __repr__ = _swig_repr

    def __init__(self, nelements):
        this = _helpers.new_input(nelements)
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _helpers.delete_input
    __del__ = lambda self: None

    def __getitem__(self, index):
        return _helpers.input___getitem__(self, index)

    def __setitem__(self, index, value):
        return _helpers.input___setitem__(self, index, value)

    def cast(self):
        return _helpers.input_cast(self)
    __swig_getmethods__["frompointer"] = lambda x: _helpers.input_frompointer
    if _newclass:
        frompointer = staticmethod(_helpers.input_frompointer)
input_swigregister = _helpers.input_swigregister
input_swigregister(input)

def input_frompointer(t):
    return _helpers.input_frompointer(t)
input_frompointer = _helpers.input_frompointer

# This file is compatible with both classic and new-style classes.


