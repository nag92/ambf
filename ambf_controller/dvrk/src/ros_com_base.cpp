#include "ros_com_base.h"

#include "ros_com_base.h"
#include "ambf_msgs/ObjectCmd.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/WorldCmd.h"
#include "ambf_msgs/WorldState.h"


template<>
///
/// \brief RosComBase::cleanUp
///

void RosComBaseClient<ambf_msgs::ObjectCmd, ambf_msgs::ObjectState>::cleanUp(){
    m_pub.shutdown();
    m_sub.shutdown();
}

template<>
///
/// \brief RosComBase::cleanUp
///

void RosComBaseClient<ambf_msgs::WorldCmd, ambf_msgs::WorldState>::cleanUp(){
    m_pub.shutdown();
    m_sub.shutdown();
}
