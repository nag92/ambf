#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H

#include<ros/ros.h>
#include <ros/master.h>
#include<geometry_msgs/WrenchStamped.h>

#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>

#include "ambf_world.h"
#include "ambf_object.h"

using namespace std;

class Client
{
private:
    ros::master::V_TopicInfo ros_topics_;
    vector<string> sub_list_;
    map<string, string> objects_dict_;
    float rate_ = 1000;
    string world_name_ = "";
    string a_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded
    string client_name_ = "";
    string world_handle_ = "";
    int a_freq_min_ = 50;
    int a_freq_max_ = 100;
    double time_out_ = 10.0;

//    ros::master::V_TopicInfo topic_infos_;

    vector<string> lv_elems_;
//    char lc_delim_[2];
    ros::Subscriber sub_;


    bool getPublishedTopics();
    void create_objs_from_rostopics();
    bool endsWith(const std::string& stack, const std::string& needle);

//    void MyCallBack(const ambf_msgs::ObjectState::ConstPtr& msg);

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


public:
//    Client(ros::NodeHandle *nh);
    Client();
    ~Client(void);
};

#endif // AMBF_CLIENT_H
