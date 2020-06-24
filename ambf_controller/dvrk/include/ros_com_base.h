#ifndef ROS_COM_BASE_H
#define ROS_COM_BASE_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "watch_dog.h"

//template <class T_state, class T_cmd>
template <class T_cmd, class T_state>
class RosComBaseClient{
public:
    RosComBaseClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out)
    {
        m_name = a_name;
        m_namespace = a_namespace;

//        int argc = 0;
//        char **argv = 0;
//        ros::init(argc, argv, "ambf_client");
        nodePtr.reset(new ros::NodeHandle);
        aspinPtr.reset(new ros::AsyncSpinner(1));
        nodePtr->setCallbackQueue(&m_custom_queue);
        m_watchDogPtr.reset(new CmdWatchDogClient(a_freq_min, a_freq_max, time_out));
    }
    virtual void init() = 0;
    virtual void run_publishers();
    virtual void cleanUp();

protected:
    boost::shared_ptr<ros::NodeHandle> nodePtr;
    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
    boost::shared_ptr<CmdWatchDogClient> m_watchDogPtr;

    std::string m_namespace;
    std::string m_name;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_StatePrev;

    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

    boost::thread m_thread;
    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;
};

//template<class T_state, class T_cmd>
template <class T_cmd, class T_state>


//void RosComBaseClient<T_cmd, T_state>::run_publishers(){
//    while(nodePtr->ok()){
//        m_pub.publish(m_State);
//        m_custom_queue.callAvailable();
//        if(m_watchDogPtr->is_wd_expired()){
//            m_watchDogPtr->consolePrint(m_name);
//            reset_cmd();
//        }
//        m_watchDogPtr->m_ratePtr->sleep();
//    }
//}



//void RosComBaseClient<T_state, T_cmd>::run_publishers(){
void RosComBaseClient<T_cmd, T_state>::run_publishers(){
    while(nodePtr->ok()){
        m_pub.publish(m_Cmd);
        m_custom_queue.callAvailable();
        if(m_watchDogPtr->is_wd_expired()){
            m_watchDogPtr->consolePrint(m_name);
            reset_cmd();
        }
        m_watchDogPtr->m_ratePtr->sleep();
    }
}


#endif // ROS_COM_BASE_H
