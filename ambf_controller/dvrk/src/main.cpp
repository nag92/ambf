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
    \author    <schandrasekhar@wpi.edu>
    \author    Shreyas Chandra Sekhar
    \version   1.0$
*/
//==============================================================================
#include "ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

int main(int argc, char* argv[])
{
    Client client;
    client.connect();

//    std::string a_name = "World";
//    worldPtr world_handler = client.getAWorld(a_name, true);

    client.printSummary();
    vector<string> object_names;

    object_names = client.getRigidBodyNames();

    string psm_baselink = object_names[5];
    cout << psm_baselink << "\n";
    rigidBodyPtr psm_baselink_handler = client.getARigidBody(psm_baselink, true);

    usleep(1000000);

    cout << "get_num_of_children(): " << psm_baselink_handler->get_num_of_children() << "\n";

    std::vector<std::string> base_children = psm_baselink_handler->get_children_names();
    for(string name : base_children) {
        cout << "name: " << name << "\n";
    }

    tf::Pose pose = psm_baselink_handler->get_pose();
    psm_baselink_handler->set_pose(pose);

    cout << "is_joint_idx_valid(): " << psm_baselink_handler->is_joint_idx_valid(50) << "\n";

    client.cleanUp();
	return 0;
}
