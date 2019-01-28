#include "maxon_hardware/cepos.h"
#include <boost/foreach.hpp>
#include <math.h>

// #define VCS(func, ...)
// if (!VCS_##func)

void CEpos::write(){
    if (m_has_init){
        return;
    }

    unsigned int error_code;
    int result = 0;
    if (m_OperationMode == PROFILE_VELOCITY_MODE){
        if (isnan(m_velocity_cmd)){
            return;
        }
        int cmd = (int)m_velocity_cmd;
        if (m_max_profile_velocity >= 0){
            if (cmd < -m_max_profile_velocity){
                cmd =  -m_max_profile_velocity;
            }
            if (cmd > m_max_profile_velocity){
                cmd = m_max_profile_velocity;
            }
        }
        if (cmd == 0 && m_halt_velocity){
            VCS_HaltVelocityMovement(m_keyhandle, m_nodeid, &error_code);
        }
        else{
            VCS_MoveWithVelocity(m_keyhandle, m_nodeid, cmd, &error_code);
        }
    }
    else if (m_OperationMode == PROFILE_POSITION_MODE){
        if (isnan(m_position_cmd)){
            return;
        }
        VCS_MoveToPosition(m_keyhandle, m_nodeid, (int)m_position_cmd, true, true, &error_code);
    }

}

void CEpos::read(){
    if (!m_has_init){
        return;
    }

    unsigned int error_code;

    // Read statusword, Memory Index 0x6041, Sub Index 0x00
    unsigned int bytes_read;
    VCS_GetObject(m_keyhandle, m_nodeid, 0x6041, 0x00, &m_statusword, 2, &bytes_read, &error_code);

    int position_raw;
    int velocity_raw;
    short current_raw;
    VCS_GetPositionIs(m_keyhandle, m_nodeid, &position_raw, &error_code);
    VCS_GetVelocityIs(m_keyhandle, m_nodeid, &velocity_raw, &error_code);
    VCS_GetCurrentIs(m_keyhandle, m_nodeid, &current_raw, &error_code);
    m_position = position_raw;
    m_velocity = velocity_raw;
    m_current = current_raw / 1000.0;
    m_effort = m_current * m_torque_constant;
}