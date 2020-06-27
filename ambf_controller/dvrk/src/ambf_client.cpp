#include "ambf_client.h"

//Client::Client(ros::NodeHandle *nh)
Client::Client()
{

    this->create_objs_from_rostopics();



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


    string trim_topic = "/State";
    for(int i = 0; i < ros_topics_.size(); i++)
    {
        string topic_name = ros_topics_[i].name;
        string msg_type = ros_topics_[i].datatype;

        if(endsWith(topic_name, trim_topic)) {
//            ROS_INFO("%s: %s", topic_name.c_str(), msg_type .c_str());
            topic_name.erase (topic_name.begin(), topic_name.begin() + a_namespace_.length());
            topic_name.erase (topic_name.end() - trim_topic.length(), topic_name.end());

            if(msg_type == "ambf_msgs/WorldState") {
                new WorldRosComClient(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/ObjectState") {
                ROS_INFO("%s", topic_name.c_str());
                objects_map_[topic_name.c_str()] =  new ObjectRosComClient(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
//                new ObjectRosComClient(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            }
        }
    }



//        std::string a_name_world = "/World/";
//        std::string a_name_object = "/Object/";
//        std::string a_name_psm_base = "/psm/baselink/";

//        std::string a_namespace = "/ambf/env/";

//        WorldRosComClient wrc1(a_name_world, a_namespace, a_freq_min_, a_freq_max_, time_out_);

//        ObjectRosComClient orc1(a_name_object, a_namespace, a_freq_min_, a_freq_max_, time_out_);
//        ObjectRosComClient occb = new ObjectRosComClient(a_name_psm_base, a_namespace, a_freq_min_, a_freq_max_, time_out_);
//        ObjectRosComClient *d1 = new ObjectRosComClient(a_name_psm_base, a_namespace, a_freq_min_, a_freq_max_, time_out_);
//        new WorldRosComClient (a_name_world, a_namespace, a_freq_min_, a_freq_max_, time_out_);
//        new ObjectRosComClient (a_name_psm_base, a_namespace, a_freq_min_, a_freq_max_, time_out_);

//    objects_map_[a_name_psm_base] = d1;
//      objects_map_[a_name_psm_base] =  new ObjectRosComClient(a_name_psm_base, a_namespace, a_freq_min_, a_freq_max_, time_out_);
//    delete d1;





}

bool Client::getPublishedTopics(){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

    ros_topics_.clear();
//    ROS_INFO("%d", args[0].size());
//    ROS_INFO("%d", result.size());
//    ROS_INFO("%d", payload.size());

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

//  void Client::MyCallBack(const ambf_msgs::ObjectState::ConstPtr& msg)
//  {
//      ROS_INFO("%f", msg->mass);
//  }


bool Client::endsWith(const std::string& stack, const std::string& needle) {
    return stack.find(needle, stack.size() - needle.size()) != std::string::npos;
}

Client::~Client(void){}
