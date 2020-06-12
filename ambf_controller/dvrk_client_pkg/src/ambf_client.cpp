#include "ambf_client.h"

Client::Client(ros::NodeHandle *nh)
{

}


void Client::create_objs_from_rostopics()
{

}
/**
 * @brief      the Main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     success
 */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ambf_client_node");
    ros::NodeHandle nh;

    Client client(&nh);

    ros::spin();
    return 0;
}
