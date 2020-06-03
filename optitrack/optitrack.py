import vrpn
import numpy as np
from scipy.spatial.transform import Rotation as R

class VRPNclient:
    def callback(self, userdata, data):
        self.tracked = True
        self.data_read = {userdata: data}
        #print self.data_read;

    def __init__(self, tracker_name, hostID):
        self.tracker_name = tracker_name
        self.hostID= hostID

        self.tracked = False

        self.data_read = None

        self.tracker = vrpn.receiver.Tracker(tracker_name + "@" + hostID)
        self.tracker.register_change_handler(self.tracker_name, self.callback, "position")

        self.analog = vrpn.receiver.Analog(tracker_name+"@"+hostID)
        self.analog.register_change_handler("analog", self.callback)

        self.button = vrpn.receiver.Button(tracker_name+"@"+hostID)
        self.button.register_change_handler("button", self.callback)

        self.info = []

    def sample_data(self):
        self.tracker.mainloop()
        self.analog.mainloop()
        self.button.mainloop()

    def get_observation(self):
        while not self.tracked:
            self.sample_data()

        # parsing data
        self.info = []
        self.info+= list(self.data_read[self.tracker_name]['position'])

        q = list(self.data_read[self.tracker_name]['quaternion'])

        self.info += q
        self.tracked = False
        return self.info

class BlockOrientation():
    def __init__(self, ip = "192.168.50.24:3883"):
        self.v = []
        self.wand = VRPNclient("Wand",  "tcp://" + ip)
        self.head = VRPNclient("DHead", "tcp://" + ip)
        self.base = VRPNclient("DBase", "tcp://" + ip)
        self.end_eff_orientation = None
        print "Optitrack Comm Initialized!"

    def get_observation(self):
        #del_centroid_head = []
        #del_centroid_base = []

        head_o = self.head.get_observation()
        base_o = self.base.get_observation()
        v = (np.array(head_o) - np.array(base_o))[:3]
        Rot = R.from_rotvec(np.array([np.deg2rad(53.),0,0]))

        #print Rot.as_euler('zyx')
        return  Rot.apply(v).tolist()

    def get_target(self):
        wand_o = self.wand.get_observation()
        base_o = self.base.get_observation()
        target = wand_o[:3]
        target_orientation = np.array(target) - np.array(base_o[:3])
        return target_orientation

if __name__=='__main__':
    import time

    C = VRPNclient("DHead", "tcp://192.168.50.24:3883")
    B = VRPNclient("DBase", "tcp://192.168.50.24:3883")
    while True:
        start = time.time()
        print "head: ", C.get_observation() # collect a single observation
        print "base: ", B.get_observation() # collect a single observation

        elapsed = time.time() - start
        #print "elapsed: ", elapsed, " ms"
