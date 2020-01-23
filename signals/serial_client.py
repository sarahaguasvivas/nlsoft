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
        while len(self.data) != self.num_sensors:
            x = []
            while len(x) < 60 or len(x) > 68:
                x = self.serial_.readline()
            print x
            array_string = x.split(",")
            array_string[-1] = array_string[-1].replace("\n", "")
            array_string[-1] = array_string[-1].replace("\r", "")
            array_string[-1] = array_string[-1].replace("/n", "")
            array_string[-1] = array_string[-1].replace("n", "")
            array_string[-1] = array_string[-1].replace("..", ".")
            array_string[0] = array_string[0].replace("..", ".")

            for ii in array_string:
                try:
                    float (ii)
                except:
                    while array_string[ii][0] == ",":
                        array_string[ii][0] = ""
                    while array_string[ii][0] == ".":
                        array_string[ii][0] = ""
                    array_split = array_string[ii].split(",")
                    if len(array_split) > 1:
                        array_string[ii] = array_split[0]
                        array_string += [array_split[1]]
            self.data = [float(i) for i in array_string]
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
