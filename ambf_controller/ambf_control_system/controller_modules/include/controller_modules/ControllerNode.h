#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H

#include <Eigen/Core>
#include <ambf_client/ambf_client.h>
#include <ambf_client/RigidBody.h>
#include "rbdl_server/RBDLForwardDynamics.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLBodyNames.h"
#include "ros/ros.h"
#include "controller_modules/PDController.h"
#include <vector>
#include "trajectory_generator/trajectory.h"
#include <boost/thread/thread.hpp>

class ControllerNode
{

    public:
        ControllerNode(rigidBodyPtr, ros::NodeHandle*, const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::MatrixXd>&);
        void setGain(const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::MatrixXd>&);
        void updataPath(const trajectory_generator::trajectory&);
        void control();
        bool startController();

   private:
        Eigen::VectorXd VectToEigen(const std::vector<double> &msg);
        rigidBodyPtr handle;
        bool have_path;
        int path_index;
        int path_length;
        ros::NodeHandle n;
        ros::ServiceClient client_ID; //nh.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
        PDController controller;
        Eigen::VectorXd desired_pos; //=  VectToEigen(pos_vec);
        Eigen::VectorXd desired_vel;
        Eigen::VectorXd desired_accel ;// =  VectToEigen(vel_vec);
        trajectory_generator::trajectory path;
};

#endif // CONTROLLERNODE_H
