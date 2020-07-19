
//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar
    \version   1.0$
*/
//==============================================================================

#ifndef AMBF_CLIENT_H
#define AMBF_CLIENT_H
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include<geometry_msgs/WrenchStamped.h>

#include "Actuator.h"
#include "Camera.h"
#include "World.h"
#include "RosComBase.h"
#include "Object.h"
#include "Light.h"
#include "RigidBody.h"
#include "Sensor.h"
#include "Vehicle.h"

#include<ambf_msgs/ActuatorCmd.h>
#include<ambf_msgs/ActuatorState.h>
#include<ambf_msgs/CameraCmd.h>
#include<ambf_msgs/CameraState.h>
#include<ambf_msgs/LightCmd.h>
#include<ambf_msgs/LightState.h>
#include<ambf_msgs/ObjectCmd.h>
#include<ambf_msgs/ObjectState.h>
#include<ambf_msgs/RigidBodyCmd.h>
#include<ambf_msgs/RigidBodyState.h>
#include<ambf_msgs/SensorCmd.h>
#include<ambf_msgs/SensorState.h>
#include<ambf_msgs/VehicleCmd.h>
#include<ambf_msgs/VehicleState.h>
#include<ambf_msgs/WorldCmd.h>
#include<ambf_msgs/WorldState.h>

#include <unordered_map>
#include <memory>
#include <vector>
#include <iostream>
#include <string>

using namespace std;
using namespace ambf_client;

//------------------------------------------------------------------------------
typedef IBaseObject* iBaseObjectPtr;
typedef std::unordered_map<string, iBaseObjectPtr> iBaseObjectMap;
typedef Actuator* actuatorPtr;
typedef Camera* cameraPtr;
typedef Light* lightPtr;
typedef Object* objectPtr;
typedef RigidBody* rigidBodyPtr;
typedef Sensor* sensorPtr;
typedef Vehicle* vehiclePtr;
typedef World* worldPtr;
//------------------------------------------------------------------------------

struct Observation{
public:
    Observation();

    geometry_msgs::PoseStamped m_nextState;
    double m_reward;
    double m_done;
};

class Client{
public:
    Client();
    ~Client(void);

    void connect();
    void createObjsFromRostopics();
    actuatorPtr getAActuator(std::string a_name, bool suppress_warning);
    cameraPtr getACamera(std::string a_name, bool suppress_warning);
    worldPtr getAWorld(std::string a_name, bool suppress_warning);
    objectPtr getAObject(std::string a_name, bool suppress_warning);
    lightPtr getALight(std::string a_name, bool suppress_warning);
    rigidBodyPtr getARigidBody(std::string a_name, bool suppress_warning);
    sensorPtr getASensor(std::string a_name, bool suppress_warning);
    vehiclePtr getAVehicle(std::string a_name, bool suppress_warning);

//    void add_object(std::string name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5);
//    ambf_client::Object* get_object_handle(std::string name);
//    bool object_cur_position(std::string name, double px, double py, double pz);
//    bool object_cur_orientation(std::string name, double roll, double pitch, double yaw);
//    bool object_cur_force(std::string name, double fx, double fy, double fz);
//    bool object_cur_torque(std::string name, double nx, double ny, double nz);
    void cleanUp();

private:
    ros::master::V_TopicInfo ros_topics_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> > objects_map_;
    std::unordered_map<string, std::unordered_map<string, IBaseObject *> >::iterator itr_;
    std::unordered_map<string, IBaseObject *>::iterator ptr_;


    float rate_ = 1000;
    string world_name_ = "";
    string a_namespace_ = "/ambf/env/"; //This needs to be fixed, should not be hardcoded


    int a_freq_min_ = 50;
    int a_freq_max_ = 100;
    double time_out_ = 10.0;


    bool getPublishedTopics();
    bool endsWith(const std::string& stack, const std::string& needle);


    void refresh();
    void start();

    string getCommonNamespace();

    template <typename T, typename TMap>
    T getObject(std::string a_name, TMap* a_map, bool suppress_warning);
    bool checkMessageType(std::string msg_type);

//    World* get_world_handle();
//    bool object_exists(std::string name);
//    static const int max_obj_size=10;
//    int m_numObjects;
//    std::map<std::string, boost::shared_ptr<ambf_client::Object> > m_objectMap;
//    std::map<std::string, boost::shared_ptr<ambf_client::Object> >::iterator m_objectIt;
//    boost::shared_ptr<ambf_client::Object> m_Objects[max_obj_size];
};


#endif
