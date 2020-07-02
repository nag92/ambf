#include "ambf_client.h"

Client::Client()
{
    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "ambf_client");
}

void Client::connect() {
    this->create_objs_from_rostopics();
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
            topic_name.erase (topic_name.begin(), topic_name.begin() + a_namespace_.length());
            topic_name.erase (topic_name.end() - trim_topic.length(), topic_name.end());

            if(msg_type == "ambf_msgs/WorldState") {
                world_handle_ = new WorldClient(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/ObjectState") {
                ROS_INFO("%s", topic_name.c_str());
                objects_map_[topic_name.c_str()] =  new ObjectClient(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);

            }
        }
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

    for (int i = 0; i < payload.size(); ++i) {
        ros_topics_.emplace_back(ros::master::TopicInfo(string(payload[i][0]), string(payload[i][1])));
    }
    return true;
}


bool Client::endsWith(const std::string& stack, const std::string& needle) {
    return stack.find(needle, stack.size() - needle.size()) != std::string::npos;
}


string Client::get_common_namespace() {
    return a_namespace_;
}

WorldClient* Client::get_world_handle() {
    return world_handle_;
}


vector<string> Client::get_obj_names() {
    vector<string> object_names;
    std::transform (objects_map_.begin(), objects_map_.end(),back_inserter(object_names), [] (std::pair<string, ObjectClient *> const & pair)
    {

    return pair.first;

    });

    return object_names;
}

ObjectClient* Client::get_obj_handle(string a_name) {
    if(objects_map_.find(a_name) == objects_map_.end()) {
        // Object Name does not exist
//        cout << a_name << " NAMED OBJECT NOT FOUND";
        return NULL;
    }

    return objects_map_[a_name];
}

void Client::get_obj_pose(string a_name) {
//    ObjectRosComClient* object_handler = this->get_obj_handle(a_name);

//    double px = object_handler->m_State.pose.position.x;
//    double py = object_handler->m_State.pose.position.y;
//    double pz = object_handler->m_State.pose.position.z;


}

void Client::clean_up() {
//    ros::spin();

    world_handle_->~WorldClient();

    for(std::unordered_map<string, ObjectClient *>::iterator it = objects_map_.begin(); it != objects_map_.end(); ++it) {
        cout << "Closing publisher for: " << it->first << "\n";
        it->second->~ObjectClient();
    }
}

Client::~Client(void){}
