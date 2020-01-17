#include "optitrack/vrpn_client.h"

VRPNclient::VRPNclient(std::vector<std::string> tracker_name, std::string host_IP){
    this->host_IP= host_IP;
    for (int i=0; i< tracker_name.size(); i++)
        this->tracker_name.push_back(tracker_name[i]);

    this->Tracker = new vrpn_Tracker_Remote("D-Head@192.168.50.33", NULL);
}

void VRPNclient::start_connection(){
     
}

VRPNclient::~VRPNclient(){

}


