#include "ambf_object.h"

namespace ambf_client{

    ObjectClient::ObjectClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): ObjectRosComClient(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
      m_objectCommandClient.enable_position_controller = false;
    }

    tf::Vector3 ObjectClient::get_pos() {
        double px = m_State.pose.position.x;
        double py = m_State.pose.position.y;
        double pz = m_State.pose.position.z;
        tf::Vector3 v3 = tf::Vector3(px, py, pz);
    }


    void ObjectClient::cur_position(double px, double py, double pz){
        m_trans.setOrigin(tf::Vector3(px, py, pz));
        m_State.pose.position.x = px;
        m_State.pose.position.y = py;
        m_State.pose.position.z = pz;
    }

    void ObjectClient::cur_orientation(double roll, double pitch, double yaw){
        tf::Quaternion rot_quat;
        rot_quat.setRPY(roll, pitch, yaw);
        m_trans.setRotation(rot_quat);
        tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
    }

    void ObjectClient::cur_orientation(double qx, double qy, double qz, double qw){
        tf::Quaternion rot_quat(qx, qy, qz, qw);
        m_trans.setRotation(rot_quat);
        tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
    }

    void ObjectClient::cur_force(double fx, double fy, double fz){
        tf::Vector3 f(fx, fy, fz);
        tf::vector3TFToMsg(f, m_State.wrench.force);
    }

    void ObjectClient::cur_torque(double nx, double ny, double nz){
        tf::Vector3 n(nx, ny, nz);
        tf::vector3TFToMsg(n, m_State.wrench.torque);
    }

    void ObjectClient::update_af_cmd(){
        m_objectCommandClient.update(&m_Cmd);
    }

    void ObjectClient::set_wall_time(double a_sec){
        m_State.wall_time = a_sec;
        increment_sim_step();
        m_State.header.stamp = ros::Time::now();
    }

    void ObjectClient::set_userdata(float a_data){
        if (m_State.userdata.size() != 1){
            m_State.userdata.resize(1);
        }
        m_State.userdata[0] = a_data;
    }

    void ObjectClient::set_userdata(std::vector<float> &a_data){
        if (m_State.userdata.size() != a_data.size()){
            m_State.userdata.resize(a_data.size());
        }
        m_State.userdata = a_data;
    }

    void ObjectClient::set_children_names(std::vector<std::string> children_names){
        m_State.children_names = children_names;
    }

    void ObjectClient::set_joint_names(std::vector<std::string> joint_names){
        m_State.joint_names = joint_names;
    }

    void ObjectClient::set_joint_positions(std::vector<float> joint_positions){
        if (m_State.joint_positions.size() != joint_positions.size()){
            m_State.joint_positions.resize(joint_positions.size());
        }
        m_State.joint_positions = joint_positions;
    }


    extern "C"{
        ObjectClient* create_object(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
            return new ObjectClient(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
        }

        void destroy_object(ObjectClient* obj){
            delete obj;
        }
    }
}
