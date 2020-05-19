#!/usr/bin/env python2.7
import serial
import time

class OpticalSignal:
    def __init__(self, port = '/dev/ttyACM0', baudrate = 2000000, timeout = 0.3):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.data = None
        self.serial_ = None
        self.num_sensors = 11
        self.initialize_serial()

    def initialize_serial(self):
        self.serial_ = serial.Serial(self.port, self.baudrate, \
                        stopbits = serial.STOPBITS_TWO, timeout = self.timeout)
        print "Serial port initialized!"
    def collect_signal_sample(self):
        if not self.serial_.is_open:
            self.initialize_serial()
        self.data = []

        x = []
        array_string = []
        self.data = []
        while len(x) < 25 or x[0] == "," or len(array_string) != self.num_sensors \
                or len(self.data) != self.num_sensors:
            x = self.serial_.readline()
            array_string = x.split(",")
            try:
                self.data = [int(i) for i in array_string]
            except:
                continue

        return self.data

    def close_network(self):
        self.serial_.close()

if __name__== '__main__':
    signal = OpticalSignal()
    while True:
        start = time.time()
        signal.collect_signal_sample()
        print signal.data
        elapsed = time.time() - start
        print "elapsed" , elapsed
    signal.close_network()
