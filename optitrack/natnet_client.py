import natnetclient as natnet
import numpy as np
from scipy.spatial.transform import Rotation as R

class NatNetClient:
    """
        This client is compatible with >= Python3
    """
    def __init__(self):
        pass


    def sample_data(self):
        pass
    def get_observation(self):
        pass


class BlockOrientation():
    def __init__(self, ip = '192.168.50.24:3883'):
        pass

    def get_observation(self):
        pass

    def get_target(self):
        pass

if __name__=='__main__':
    import time
    C = NatNetClient("DHead", "tcp://192.168.50.24:3883")
    B = NatNetClient("DBase", "tcp://192.168.50.24:3883")
    while True:
        start = time.time()
        print("head: ", C.get_observation())
        print("base: ", B.get_observation())
        elapsed = time.time() - start


