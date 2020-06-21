#include "ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

int main(int argc, char* argv[])
{
    string client_node = "ambf_client";


    ros::init(argc, argv, client_node);
//    ros::NodeHandle nh;
//    Client client(&nh);

    Client client;
    ros::spin();

    return 0;
}
