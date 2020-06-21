#include "ambf_client.h"

//Client::Client(ros::NodeHandle *nh)
Client::Client()
{

    this->create_objs_from_rostopics();

    std::string a_name_world = "/World/";
//    std::string a_name_object = "/World/";

    std::string a_namespace = "/ambf/env/";
    int a_freq_min = 50;
    int a_freq_max = 100;
    double time_out = 10.0;

    WorldRosComClient wc1(a_name_world, a_namespace, a_freq_min, a_freq_max, time_out);
//    Ob(a_name, a_namespace, a_freq_min, a_freq_max, time_out);
    sleep(10000);


//    ros::master::getTopics(topic_infos_);
//    ROS_INFO("%lu", topic_infos_.size());


//    for(int i = 0; i < topic_infos_.size(); i++) {
//        string topic = topic_infos_[i].name;
//        ROS_INFO("%s", topic.c_str());
//    }

//    lc_delim_[0] = '/';
//    lc_delim_[1] = '\0';

//    boost::algorithm::split( lv_elems_, topic_infos_[0].name, boost::algorithm::is_any_of( lc_delim_ ) );


//    if ( lv_elems_[0] == "vicon" )
//    {
//       // topic of topic_infos[0] is really in namespace vicon!!
//    }

//    ROS_INFO("%d", v_topic_info_.size());

//    sub_ = nh->subscribe("/ambf/env/psm/baselink/State", 1000, &Client::MyCallBack, this);
}

void Client::create_objs_from_rostopics()
{
    this->getPublishedTopics();

//    common_obj_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded
//    world_name_ = "World";
    for(int i = 0; i < ros_topics_.size(); i++)
    {
        string topic_name = ros_topics_[i].name;
        string msg_type = ros_topics_[i].datatype;

        ROS_INFO("%s: %s", topic_name.c_str(), msg_type .c_str());

//        if(msg_type == "ambf_msgs/ObjectState") {

//        }
    }






}

bool Client::getPublishedTopics(){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

    ros_topics_.clear();
    ROS_INFO("%d", args[0].size());
    ROS_INFO("%d", result.size());
    ROS_INFO("%d", payload.size());

//    ROS_INFO("%s", (string(args[0][0])).c_str());
//    ROS_INFO("%s", (string(payload[0][0])).c_str());
//    ROS_INFO("%s", (string(payload[0][1])).c_str());


    for (int i = 0; i < payload.size(); ++i) {
//        ROS_INFO("%s", (string(payload[i][0])).c_str());
//        ROS_INFO("%s", (string(payload[i][1])).c_str());
        ros_topics_.emplace_back(ros::master::TopicInfo(string(payload[i][0]), string(payload[i][1])));
    }
    return true;
}

  void Client::MyCallBack(const ambf_msgs::ObjectState::ConstPtr& msg)
  {
      ROS_INFO("%f", msg->mass);
  }



Client::~Client(void){}


