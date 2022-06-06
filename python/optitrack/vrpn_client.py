import vrpn
import numpy as np
from scipy.spatial.transform import Rotation as R

class VRPNclient:
    """
        This client has only been tested in Python3.5, 2.7
    """
    def callback(self, userdata, data):
        self.tracked = True
        self.data_read = {userdata: data}

    def __init__(self, tracker_name, hostID):
        self.tracker_name = tracker_name
        self.hostID= hostID
        self.tracked = False
        self.data_read = None

        self.tracker = vrpn.receiver.Tracker(tracker_name + "@" + hostID)
        self.tracker.register_change_handler(self.tracker_name, self.callback, "position")
<<<<<<< HEAD
=======
        #self.analog = vrpn.receiver.Analog(tracker_name+"@"+hostID)
        #self.analog.register_change_handler("analog", self.callback)
        #self.button = vrpn.receiver.Button(tracker_name+"@"+hostID)
        #self.button.register_change_handler("button", self.callback)
>>>>>>> 8d728c44ae8f58201c8c8fd3f5e1a172c5f30f96
        self.info = []

    def sample_data(self):
        self.tracker.mainloop()
<<<<<<< HEAD
=======
        #self.analog.mainloop()
        #self.button.mainloop()
>>>>>>> 8d728c44ae8f58201c8c8fd3f5e1a172c5f30f96

    def get_observation(self):
        while not self.tracked:
            self.sample_data()
        self.info = []

        self.info += list(self.data_read[self.tracker_name]['position'])
        q = list(self.data_read[self.tracker_name]['quaternion'])
        self.info += q
        self.tracked = False
        return self.info

class BlockState():
    def __init__(self, ip = "192.168.50.24:3883"):
        self.v = []
        #self.wand = VRPNclient("Wand",  "tcp://" + ip)
        self.head = VRPNclient("DHead", "tcp://" + ip)
        #self.base = VRPNclient("DBase", "tcp://" + ip)
        self.end_eff_orientation = None
        print("Optitrack Comm Initialized!")

    def get_observation(self):
        head = self.head.get_observation()
        base = self.base.get_observation()
        head_o = head[:3]
<<<<<<< HEAD
        base_o = base[:3]
        v = np.array(head_o) - np.array(base_o)
=======
        v = np.array(head_o)
>>>>>>> 8d728c44ae8f58201c8c8fd3f5e1a172c5f30f96
        return v.tolist()

if __name__=='__main__':
    import time
<<<<<<< HEAD
    C = VRPNclient("Hand", "tcp://192.168.50.10:3883")
    while True:
        start = time.time()
        print("hand: ", C.get_observation()) # collect a single observation
=======
    C = VRPNclient("DHead", "tcp://192.168.50.24:3883")
    while True:
        start = time.time()
        print("head: ", C.get_observation()) # collect a single observation
>>>>>>> 8d728c44ae8f58201c8c8fd3f5e1a172c5f30f96
        elapsed = time.time() - start
        print("vrpn elapsed: ", 1./elapsed, " Hz")

