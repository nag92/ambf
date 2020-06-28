#include "ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

int main(int argc, char* argv[])
{
//    string client_node = "ambf_client";


//    ros::init(argc, argv, client_node);
//    ros::NodeHandle nh;
//    Client client(&nh);

    Client client;
    client.connect();


    vector<string> object_names = client.get_obj_names();

    cout << "object_names.size() - main: " <<object_names.size() << "\n";
    for(string obj : object_names) {
        cout << "obj: " << obj << "\n";
    }

    ObjectRosComClient* psm_baselink_handle = client.get_obj_handle("psm/baselink");
    client.clean_up();
    return 0;
}
