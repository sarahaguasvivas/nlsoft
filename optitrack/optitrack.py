import vrpn


class VRPNclient:
    def callback(self, userdata, data):
        self.tracked = True
        print {userdata: data};

    def __init__(self, tracker_name, hostID):
        self.tracker_name = tracker_name
        self.hostID= hostID

        self.tracked = False

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

if __name__=='__main__':
    import time
    C = VRPNclient("DHead", "tcp://192.168.50.33:3883")
    B = VRPNclient("DBase", "tcp://192.168.50.33:3883")
    while True:
        # Collect a single observation
        start = time.time()
        while not C.tracked:
            C.sample_data()
            B.sample_data()
        elapsed = time.time() - start
        print "elapsed: ", elapsed, " ms"
        C.tracked = False
        B.tracked = False
