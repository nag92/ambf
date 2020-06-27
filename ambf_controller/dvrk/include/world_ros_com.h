#ifndef WORLD_ROS_COM_H
#define WORLD_ROS_COM_H

#include "ros_com_base.h"
#include "ambf_msgs/WorldState.h"
#include "ambf_msgs/WorldCmd.h"


//class WorldRosComClient: public RosComBaseClient<ambf_msgs::WorldState, ambf_msgs::WorldCmd>{
class WorldRosComClient: public RosComBaseClient<ambf_msgs::WorldCmd, ambf_msgs::WorldState>{
public:
    WorldRosComClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    ~WorldRosComClient();
    virtual void init();

protected:
    bool m_enableSimThrottle;
    bool m_stepSim;
    int m_num_skip_steps;
    int m_skip_steps_ctr;
    virtual void reset_cmd();
//    void sub_cb(ambf_msgs::WorldCmdConstPtr msg);
    void sub_cb(ambf_msgs::WorldStateConstPtr msg);

};

#endif // WORLD_ROS_COM_H
