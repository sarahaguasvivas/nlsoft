#include "optitrack/vrpn_client.h"

VRPNclient::VRPNclient(std::vector<std::string> tracker_name, std::string host_IP){
    this->host_IP= host_IP;
    for (int i=0; i< tracker_name.size(); i++)
        this->tracker_name.push_back(tracker_name[i]);
    this->connection = new vrpn_Connection();
    long float = connection->register_message_type(float);
    long id = connection->register_sender(host_IP);

    struct timeval now;
    gettimeofday(&now, NULL):

    connection->pack_message(sizeof("VRPN Connection established!"), new, float, id, "VRPN Connection established!", vrpn_CONNECTION_RELIABLE);
    connection->mainloop();
}

VRPNclient::start_connection(){
     
}

VRPNclient::~VRPNclient(){

}


