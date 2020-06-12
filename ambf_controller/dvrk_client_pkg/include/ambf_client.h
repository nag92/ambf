#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H

#include<ros/ros.h>
#include <ros/master.h>
#include<geometry_msgs/WrenchStamped.h>

#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>

using namespace std;

class Client
{
private:
    vector<string> ros_topics_;
    vector<string> sub_list_;
    map<string, string> objects_dict_;
    float rate_ = 0;
    string world_name_ = "";
    string common_obj_namespace_ = "";
    string client_name_ = "";
    string world_handle_ = NULL;
    ros::master::V_TopicInfo topic_infos_;

    void create_objs_from_rostopics();
    void connect();
    void refresh();
    void start();
    void get_common_namespace();
    void get_world_handle();
    void get_obj_names();
    void get_obj_handle();
    void get_obj_pose();

    void set_obj_cmd();
    void start_pubs();
    void run_obj_publishers();
    void print_active_topics();
    void print_summary();
    void clean_up();

//https://answers.ros.org/question/206893/getting-list-of-published-topics-from-within-c-code/
public:
    Client(ros::NodeHandle *nh);

};

#endif // AMBF_CLIENT_H
