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
        vrpn_Connection * connection;
        VRPNclient(std::vector<std::string> tracker_name, std::string host_IP);
        VRPNclient(){};
        void start_connection();
        float * sample();
        ~VRPNclient();
};

#endif
