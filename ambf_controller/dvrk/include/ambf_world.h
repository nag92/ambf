#ifndef AMBF_WORLD_H
#define AMBF_WORLD_H


#include <string>
#include "world_ros_com.h"

namespace ambf_client{
class WorldClient: public WorldRosComClient{
public:
    WorldClient(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    void set_wall_time(double a_sec);
    void increment_sim_step();
    inline void set_sim_time(double a_sec){m_State.sim_time = a_sec;}
    inline void set_num_devices(uint a_num){m_State.n_devices = a_num;}
    inline void set_loop_freq(double a_freq){m_State.dynamic_loop_freq = a_freq;}

    ////////////////////////////
    inline bool step_sim(){return m_stepSim;}
};
}

#endif // AMBF_WORLD_H
