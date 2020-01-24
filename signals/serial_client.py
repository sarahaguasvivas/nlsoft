import serial
import time

class OpticalSignal:
    def __init__(self, port = '/dev/ttyACM0', baudrate =2000000, timeout = 0.3):
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

    def collect_signal_sample(self):
        if not self.serial_.is_open:
            self.initialize_serial()
        self.data = []

        x = []
        array_string = []

        while len(x) < 29 or len(array_string) != self.num_sensors:
            x = self.serial_.readline()
            print len(x), x
            if x[0] == ",":
                print "here"
            #    x[0] = ""
            array_string = x.split(",")

        for ii in array_string:
            try:
                int (ii)
            except:
                print ii

            self.data = [int(i) for i in array_string]
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
