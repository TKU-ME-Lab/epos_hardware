#include "maxon_hardware/cepos.h"
#include <boost/foreach.hpp>
#include <math.h>
#include <iostream>

CEpos::CEpos(const EposParameter Param):
             m_device_name(Param.actuator), m_serial_number(Param.serial_number), m_nodeid(Param.nodeid)
{
    std::cout << "    Init Epos" << std::endl;

    unsigned int error_code;
    if (CreateDeviceKeyHandle(m_device_name, Param.protocol, Param.interface, m_nodeid, m_serial_number, &m_keyhandle)){
        std::cout << "    Got Key Handle" << std::endl;
    }
    else{
        std::cout << "    Device not found, Actuator:" << Param.actuator << ", Protocol:" << Param.protocol << ", Interface:" << ", Serial Number: " << Param.serial_number << std::endl;
        return;
    }

    if (Param.mode == "profile_velocity"){
        m_OperationMode = PROFILE_VELOCITY_MODE;

        if (VCS_ActivateProfileVelocityMode(m_keyhandle, m_nodeid, &error_code)){
            if (!VCS_SetVelocityProfile(m_keyhandle, m_nodeid, Param.velocity_profile.acceleration, Param.velocity_profile.deceleration, &error_code)){
                std::cout << "Failed to Set Velocity Profile" << ", Error Code:" << (char)error_code << std::endl;
            }
        }
    }
    else if (Param.mode == "profile_position"){
        m_OperationMode = PROFILE_POSITION_MODE;

        if (VCS_ActivateProfilePositionMode(m_keyhandle, m_nodeid, &error_code)){
            if (!VCS_SetPositionProfile(m_keyhandle, m_nodeid, Param.position_profile.velocity, Param.velocity_profile.acceleration, Param.velocity_profile.deceleration, &error_code)){
                std::cout << "Failed to Set Position Profile"  << ", Error Code:" << (char)error_code << std::endl;
            }
        }
    }
    
    if (Param.clear_fault){
        VCS_ClearFault(m_keyhandle, m_nodeid, &error_code);
    }

    if (!VCS_SetEnableState(m_keyhandle, m_nodeid, &error_code)){
        m_has_init = false;
    }

    m_has_init = true;
    std::cout << "    Init Epos done" << std::endl;
}

CEpos::CEpos(const EposParameter Param, const HANDLE keyhandle):
          m_keyhandle(keyhandle), m_device_name(Param.actuator), m_serial_number(Param.serial_number), m_nodeid(Param.nodeid)
{
    std::cout << "    Init Epos" << std::endl;    
    unsigned int error_code;

    if (Param.mode == "profile_velocity"){
        m_OperationMode = PROFILE_VELOCITY_MODE;
        if (VCS_ActivateProfileVelocityMode(m_keyhandle, m_nodeid, &error_code)){
            if (!VCS_SetVelocityProfile(m_keyhandle, m_nodeid, Param.velocity_profile.acceleration, Param.velocity_profile.deceleration, &error_code)){
                std::cout << "Failed to Set Velocity Profile" << ", Error Code:" << (char)error_code << std::endl;
            }
        }
    }
    else if (Param.mode == "profile_position"){
        m_OperationMode = PROFILE_POSITION_MODE;
        if (VCS_ActivateProfilePositionMode(m_keyhandle, m_nodeid, &error_code)){
            if (!VCS_SetPositionProfile(m_keyhandle, m_nodeid, Param.position_profile.velocity, Param.velocity_profile.acceleration, Param.velocity_profile.deceleration, &error_code)){
                std::cout << "Failed to Set Position Profile"  << ", Error Code:" << (char)error_code << std::endl;
            }
        }
    }

    if (Param.clear_fault){
        VCS_ClearFault(m_keyhandle, m_nodeid, &error_code);
    }

    if (!VCS_SetEnableState(m_keyhandle, m_nodeid, &error_code)){
        m_has_init = false;
    }

    m_has_init = true;
    std::cout << "    Init Epos done" << std::endl;
}

CEpos::~CEpos(){
    unsigned int error_code;
    if (m_keyhandle){
        VCS_SetDisableState(m_keyhandle, m_nodeid, &error_code);

    }
}

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

double* CEpos::GetPositionPtr(){
    return &m_position;
}

double* CEpos::GetPositionCmdPtr(){
    return &m_position_cmd;
}

double* CEpos::GetVelocityPtr(){
    return &m_velocity;
}

double* CEpos::GetVelocityCmdPtr(){
    return &m_velocity_cmd;
}

double* CEpos::GetEffortPtr(){
    return &m_effort;
}
