import vrpn

def callback(userdata, data):
    print (userdata, " => " , data);

class VRPNclient:
    def __init__(self, tracker_name, hostID):
        self.tracker_name = tracker_name
        self.hostID= hostID

        self.tracker = vrpn.receiver.Tracker(tracker_name + "@" + hostID)
        self.tracker.register_change_handler(self.tracker_name, callback, "position")

        self.analog = vrpn.receiver.Analog(tracker_name+"@"+hostID)
        self.analog.register_change_handler("analog", callback)

        self.button = vrpn.receiver.Button(tracker_name+"@"+hostID)
        self.button.register_change_handler("button", callback)

    def sample_data(self):
        self.tracker.mainloop()
        self.analog.mainloop()
        self.button.mainloop()

if __name__=='__main__':
    C = VRPNclient("DHead", "tcp://192.168.50.33:3883")
    B = VRPNclient("DBase", "tcp://192.168.50.33:3883")

    while True:
        C.sample_data()
        B.sample_data()
