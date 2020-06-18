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
void RosComBaseClient<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>::cleanUp(){
    m_pub.shutdown();
    m_sub.shutdown();
}

template<>
///
/// \brief RosComBase::cleanUp
///
void RosComBaseClient<ambf_msgs::WorldState, ambf_msgs::WorldCmd>::cleanUp(){
    m_pub.shutdown();
    m_sub.shutdown();
}
