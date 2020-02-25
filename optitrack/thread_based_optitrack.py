import vrpn
import numpy as np
import threading import Thread, Queue


class VRPNclientThread:

    def __init__(self, tracker_name, hostID):
        self.tracker_name = tracker_name
        self.hostID= hostID

        self.tracked = False
        self.data_read = None

        self.q = Queue.Queue()

        def callback(self, userdata, data):
            self.tracked = True
            self.data_read = {userdata: data}

        self.tracker = vrpn.receiver.Tracker(tracker_name + "@" + hostID)
        self.tracker.register_change_handler(self.tracker_name, self.callback, "position")

        self.analog = vrpn.receiver.Analog(tracker_name+"@"+hostID)
        self.analog.register_change_handler("analog", self.callback)

        self.button = vrpn.receiver.Button(tracker_name+"@"+hostID)
        self.button.register_change_handler("button", self.callback)

        self.info = []

        self.start_thread()

    def thread_data_collect(self):
        """
            This function will be called
            in a separate thread
        """
        while True:
            self.tracker.mainloop()
            self.analog.mainloop()
            self.button.mainloop()

    def start_thread(self):
        self.thread = Thread(target= self.thread_data_collect
        self.thread.start()
        self.thread.join()

    def get_observation(self):
        return self.data_read

class BlockOrientation():
    def __init__(self, ip = "192.168.50.33:3883"):
        self.wand = VRPNclient("Wand",  "tcp://" + ip)
        self.head = VRPNclient("DHead", "tcp://" + ip)
        self.base = VRPNclient("DBase", "tcp://" + ip)
        self.end_eff_orientation = None

    def get_observation(self):
        head_o= self.head.get_observation()
        base_o= self.base.get_observation()

        self.end_eff_orientation = np.array(head_o) - np.array(base_o)

        return self.end_eff_orientation.tolist()

    def get_target(self):
        wand_o = self.wand.get_observation()
        base_o= self.base.get_observation()
        target = wand_o[:3] # only target position
        target_orientation = np.array(target) - np.array(base_o[:3])
        return target_orientation

if __name__=='__main__':
    import time

    C = VRPNclient("DHead", "tcp://192.168.50.33:3883")
    B = VRPNclient("DBase", "tcp://192.168.50.33:3883")

    while True:
        start = time.time()
        print C.get_observation() # collect a single observation
        print B.get_observation() # collect a single observation

        elapsed = time.time() - start
        print "elapsed: ", elapsed, " ms"
