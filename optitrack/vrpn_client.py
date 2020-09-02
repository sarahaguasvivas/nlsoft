import vrpn
import numpy as np
from scipy.spatial.transform import Rotation as R

class VRPNclient:
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
        print("Optitrack Comm Initialized!")

    def get_observation(self):
        head = self.head.get_observation()
        base = self.base.get_observation()
        head_o = head[:3]
        base_o = base[:3]

        rot_head = R.from_quat(head[3:]).as_euler('xyz', degrees=False)
        rot_base = R.from_quat(base[3:]).as_euler('xyz', degrees=False)

        print((rot_head - rot_base))
        #[0.15742808  0.45209943 -0.0200593]

        v = (np.array(head_o) - np.array(base_o))
        shift = np.array([0.0, -37.7/1000., 7.87/1000.])
        #shift = [-0.01436728, -0.01648215, 0.00491437]

        Rot = R.from_rotvec(np.array([np.pi/4., 0, 0]))
        ob= Rot.apply(v).tolist() - np.array(shift)
        return ob.tolist()

    def get_target(self):
        wand_o = self.wand.get_observation()
        base_o = self.base.get_observation()
        target = wand_o[:3]
        target_orientation = np.array(target) - np.array(base_o[:3])
        return target_orientation

if __name__=='__main__':
    import time
    #C = VRPNclient("DHead", "tcp://192.168.50.24:3883")
    #B = VRPNclient("DBase", "tcp://192.168.50.24:3883")
    A = BlockOrientation()
    while True:
        start = time.time()
        print("head: ", A.get_observation()) # collect a single observation
        #print("base: ", B.get_observation()) # collect a single observation
        elapsed = time.time() - start
        print("vrpn elapsed: ", 1./elapsed, " Hz")

