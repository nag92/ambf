#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H

#include<ros/ros.h>
#include <ros/master.h>
#include<geometry_msgs/WrenchStamped.h>
//#include<geometry_msgs/Pose.h>

#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>
#include <unordered_map>

#include "ambf_world.h"
#include "ambf_object.h"

using namespace std;
using namespace ambf_client;

class Client
{
private:
    ros::master::V_TopicInfo ros_topics_;
    vector<string> sub_list_;
    std::unordered_map<string, ObjectClient *> objects_map_;
    float rate_ = 1000;
    string world_name_ = "";
    string a_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded
    string client_name_ = "";
    WorldClient *world_handle_ = NULL;

    int a_freq_min_ = 50;
    int a_freq_max_ = 100;
    double time_out_ = 10.0;

    vector<string> lv_elems_;
    ros::Subscriber sub_;


    bool getPublishedTopics();
    bool endsWith(const std::string& stack, const std::string& needle);


    void refresh();
    void start();

    string get_common_namespace();



public:
    Client();
    void connect();
    void create_objs_from_rostopics();
//    WorldRosComClient* get_world_handle();
    WorldClient* get_world_handle();
    vector<string> get_obj_names();
    ObjectClient* get_obj_handle(string a_name);
    void get_obj_pose(string a_name);


    void set_obj_cmd();
    void start_pubs();
    void run_obj_publishers();
    void print_active_topics();
    void print_summary();
    void clean_up();
    ~Client(void);

};

#endif // AMBF_CLIENT_H
