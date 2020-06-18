#ifndef OBJECT_ROS_COM_H
#define OBJECT_ROS_COM_H


#include "ros_com_base.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/ObjectCmd.h"


class ObjectRosComClient: public RosComBaseClient<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>{
public:
    ObjectRosComClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    ~ObjectRosComClient();
    virtual void init();

protected:
    virtual void reset_cmd();
    void sub_cb(ambf_msgs::ObjectCmdConstPtr msg);
};

#endif // OBJECT_ROS_COM_H
