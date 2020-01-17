#ifndef VRPN_CLIENT_H
#define VRPN_CLIENT_H

#include <vrpn_Tracker.h>
#include <string>
#include <vector>

class VRPNclient{
    public:
        std::string host_IP;
        std::vector<std::string> tracker_name; 
        float * data;
        VRPNclient(std::vector<std::string> tracker_name, std::string host_IP);
        VRPNclient(){};
        void start_connection();
        float * sample();
        
        // VRPN client variables:
        vrpn_Tracker_Remote * Tracker;
        ~VRPNclient();
};

#endif
