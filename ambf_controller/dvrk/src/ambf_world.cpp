#include "ambf_world.h"

namespace ambf_client{

WorldClient::WorldClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): WorldRosComClient(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    m_num_skip_steps = 10;
    m_skip_steps_ctr = 0;
}

void WorldClient::set_wall_time(double a_sec){
    m_State.wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
}

void WorldClient::increment_sim_step(){
    if(m_enableSimThrottle){
        m_skip_steps_ctr++;
        if (m_skip_steps_ctr == m_num_skip_steps){
            m_stepSim = false;
            m_skip_steps_ctr = 0;
        }
        if (m_skip_steps_ctr > m_num_skip_steps){
            std::cerr << "WARN, Skipped " << m_skip_steps_ctr << " steps, Default skip limit " << m_num_skip_steps << std::endl;
        }
    }
    m_State.sim_step++;
}

extern "C"{

WorldClient* create_world(std::string a_name, std::string a_namespace="/ambf_client/", int a_min_freq=50, int a_max_freq=1000, double time_out=10.0){
    return new WorldClient(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_world(WorldClient* obj){
    delete obj;
}

}
}
