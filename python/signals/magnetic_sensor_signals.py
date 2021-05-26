import time
import tensorflow as tf
from typing import List
import numpy as np

class MagneticSignal:
    def __init__(self, model):
        self.signal_model = tf.keras.models.load_model(model, compile = False)

    def collect_signal_sample(self, hasel_inputs : List[float]):
        x = np.array(hasel_inputs)
        x = x.reshape(1, 1, -1)
        self.data = self.signal_model.predict(x)[0]
        return self.data

if __name__== '__main__':
    signal = MagneticSignal(model = '../models/model_signals_may_25_2021.hdf5')
    while True:
        start = time.time()
        signal.collect_signal_sample(hasel_inputs = [1.]*6)
        print(signal.data)
        elapsed = time.time() - start
        print("elapsed" , elapsed)