swig -python -c++ helpers.i
c++ -c -fpic activations.cpp matrix.cpp helpers.cpp
c++ -c -fpic helpers_wrap.cxx -I/usr/include/python3.8
c++ -shared helpers.o activations.o helpers_wrap.o -o _helpers.so
