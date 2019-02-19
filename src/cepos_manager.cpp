#include "maxon_hardware/cepos_manager.h"
#include <boost/foreach.hpp>
#include <iostream>

CEposManager::CEposManager(std::vector<EposParameter> Params){
    std::cout << "Init EposManager, Get " << Params.size() << " Motor Parameters." << std::endl;
    BOOST_FOREACH(const EposParameter &Param, Params){
        if (!Param.is_sub_device){
            std::cout << "  Motor:" << Param.motor_name << std::endl;
            CEpos* motor = new CEpos(Param);
            m_motormap.insert({Param.motor_name, motor});
        }
    }

    BOOST_FOREACH(const EposParameter &Param, Params){
        if (Param.is_sub_device){
            std::cout << "  Motor:" << Param.motor_name << std::endl;
            MapMotor::iterator iter = m_motormap.find(Param.master_device);
            CEpos* motor = new CEpos(Param, iter->second->GetHANDLE());
            m_motormap.insert(std::make_pair(Param.motor_name, motor));
        }
    }
}

MapMotor CEposManager::GetMotors(){
    return m_motormap;
}

MapMotor* CEposManager::GetMotorsPtr(){
    return &m_motormap;
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
