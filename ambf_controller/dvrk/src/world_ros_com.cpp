#include "world_ros_com.h"

WorldRosComClient::WorldRosComClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RosComBaseClient(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    init();
}

void WorldRosComClient::init(){
    m_State.sim_step = 0;
    m_enableSimThrottle = false;
    m_stepSim = true;

    m_pub = nodePtr->advertise<ambf_msgs::WorldCmd>("/" + m_namespace + "/" + m_name + "/Command", 10);
    m_sub = nodePtr->subscribe("/" + m_namespace + "/" + m_name + "/State", 10, &WorldRosComClient::sub_cb, this);

    m_thread = boost::thread(boost::bind(&WorldRosComClient::run_publishers, this));
    std::cerr << "Thread Joined: " << m_name << std::endl;
}

void WorldRosComClient::enable_throttling(bool flag) {
    m_Cmd.enable_step_throttling = flag;
}

void WorldRosComClient::set_num_step_skips(int n) {

    if ( n <= 0 || n >= 100 ) {
        throw std::invalid_argument( "received invalid value" );
        return;
    }
    m_Cmd.n_skip_steps = n;
}

void WorldRosComClient::ros_cb(ambf_msgs::WorldStateConstPtr msg) {
    m_State = *msg;
}

void WorldRosComClient::update(){
    m_Cmd.step_clock = !m_Cmd.step_clock;
    m_pub.publish(m_Cmd);
    m_watchDogPtr->acknowledge_wd();
}

void WorldRosComClient::clear_cmd(){
    m_Cmd.enable_step_throttling = false;
}

void WorldRosComClient::set_name(string name) {
    m_name = name;
}

string WorldRosComClient::get_name() {
    return m_name;
}

WorldRosComClient::~WorldRosComClient(){
    ros::shutdown();
    std::cerr << "Thread Shutdown: " << m_name << std::endl;
}

void WorldRosComClient::reset_cmd(){
    m_enableSimThrottle = false;
    m_stepSim = true;
}

//void WorldRosComClient::sub_cb(ambf_msgs::WorldCmdConstPtr msg){
//    m_CmdPrev = m_Cmd;
//    m_Cmd = *msg;
//    m_num_skip_steps = m_Cmd.n_skip_steps;
//    m_enableSimThrottle = (bool)m_Cmd.enable_step_throttling;
//    if (m_enableSimThrottle){
//        if(!m_stepSim){
//            m_stepSim = (bool)m_Cmd.step_clock ^ (bool)m_CmdPrev.step_clock;
//        }
//    }
//    else{
//            m_stepSim = true;
//    }
//    m_watchDogPtr->acknowledge_wd();
//}

void WorldRosComClient::sub_cb(ambf_msgs::WorldStateConstPtr msg){
    m_StatePrev = m_State;
    m_State = *msg;
//    m_num_skip_steps = m_State.n_skip_steps;
//    m_enableSimThrottle = (bool)m_Cmd.enable_step_throttling;
//    if (m_enableSimThrottle){
//        if(!m_stepSim){
//            m_stepSim = (bool)m_Cmd.step_clock ^ (bool)m_CmdPrev.step_clock;
//        }
//    }
//    else{
//            m_stepSim = true;
//    }
    m_watchDogPtr->acknowledge_wd();
}
