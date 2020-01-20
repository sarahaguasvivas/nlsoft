import vrpn


class VRPNclient:
    def callback(self, userdata, data):
        self.tracked = True
        self.data_read = {userdata: data}
        print self.data_read;

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

    def sample_data(self):
        self.tracker.mainloop()
        self.analog.mainloop()
        self.button.mainloop()

    def get_observation(self):
        while not self.tracked:
            self.sample_data()
        self.tracked = False
        return self.data_read

if __name__=='__main__':
    import time
    C = VRPNclient("DHead", "tcp://192.168.50.33:3883")
    B = VRPNclient("DBase", "tcp://192.168.50.33:3883")
    while True:
        # Collect a single observation
        start = time.time()
        while True:
            C.get_observation()
            B.get_observation()
        elapsed = time.time() - start
        print "elapsed: ", elapsed, " ms"
