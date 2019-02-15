#include "maxon_hardware/cepos_manager.h"
#include <boost/foreach.hpp>
#include <iostream>

CEposManager::CEposManager(std::vector<EposParameter> Params){
    std::cout << "Init EposManager, Get " << Params.size() << " Motor Parameters." << std::endl;
    BOOST_FOREACH(const EposParameter &Param, Params){
        if (!Param.is_sub_device){
            std::cout << "  Motor:" << Param.motor_name << std::endl;
            std::string motor_name = Param.motor_name;
            m_motor_names.push_back(motor_name);
            CEpos* motor = new CEpos(Param);
            m_motors.push_back(motor);
            std::cout << "  Get Iter" << std::endl;
            std::vector<std::string>::reverse_iterator it = m_motor_names.rbegin();
            std::cout << "  Insert" << std::endl;
            m_motormap.insert({*it, motor});
        }
    }

    BOOST_FOREACH(const EposParameter &Param, Params){
        if (Param.is_sub_device){
            std::cout << "  Motor:" << Param.motor_name << std::endl;
            std::string motor_name = Param.motor_name;
            m_motor_names.push_back(motor_name);
            std::cout << "  Find Master Device" << std::endl;
            MapMotor::iterator iter = m_motormap.find(Param.master_device);
            std::cout << "  Get Handle" << std::endl;
            CEpos* motor = new CEpos(Param, iter->second->GetHANDLE());
            m_motors.push_back(motor);
            std::cout << "  Get Iter" << std::endl;
            std::vector<std::string>::reverse_iterator it = m_motor_names.rbegin();
            std::cout << "  Push back" << std::endl;
            m_motormap.insert(std::make_pair(*it, motor));
        }
    }
}

MapMotor CEposManager::GetMotors(){
    return m_motormap;
}

bool CEposManager::init(){
    bool has_init = false;
    for (MapMotor::iterator motor_iter = m_motormap.begin(); motor_iter != m_motormap.end(); motor_iter++)
    {
        has_init = motor_iter->second->hasInit();
        if (has_init){
            std::cout << motor_iter->first << " Init Succeeded" << std::endl;
        }
        else{
            std::cout << motor_iter->first << " Init Failed" << std::endl;
        }
    }

    return has_init;
}

void CEposManager::read(){
    for (MapMotor::iterator motor_iterator = m_motormap.begin(); motor_iterator != m_motormap.end(); motor_iterator++)
    {
        motor_iterator->second->read();
    }
}

void CEposManager::write(){
    for (MapMotor::iterator motor_iterator = m_motormap.begin(); motor_iterator != m_motormap.end(); motor_iterator++)
    {
        motor_iterator->second->write();
    }
}
