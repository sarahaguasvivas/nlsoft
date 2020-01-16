#include <iostream>
#include "optitrack/vrpn_client.h"
#include <vector>
#include <string>
#include <typeinfo>

int main(){
    std::vector<std::string> tracking_id;
    std::string body1="D_Head";
    std::string body2="D_Base";

    tracking_id.push_back(body1);
    tracking_id.push_back(body2);

    std::string id = "192.168.50.33"; 

    VRPNclient * C = new VRPNclient (tracking_id, id); 
    return 0;
}
