#include "maxon_hardware/cepos_manager.h"
#include <boost/foreach.hpp>

CEposManager::CEposManager(std::vector<EposParameter> Params){
    BOOST_FOREACH(const EposParameter &Param, Params){
        if (!Param.is_sub_device){
            boost::shared_ptr<CEpos> motor(new CEpos(Param));

            m_motors.insert(std::pair<std::string, boost::shared_ptr<CEpos> >(Param.motor_name, motor));
        }
    }

    BOOST_FOREACH(const EposParameter &Param, Params){
        if (Param.is_sub_device){
            std::string master_device = Param.master_device;
            std::map<std::string, boost::shared_ptr<CEpos> >::iterator iter = m_motors.find(master_device.c_str());
            
            boost::shared_ptr<CEpos> motor(new CEpos(Param, iter->second->GetHANDLE()));
            m_motors.insert(std::pair<std::string, boost::shared_ptr<CEpos> >(Param.motor_name, motor));
        }
    }
}

MapMotor CEposManager::GetMotors(){
    return m_motors;
}

bool CEposManager::init(){
    bool has_init = false;
    for (MapMotor::iterator motor_iter = m_motors.begin(); motor_iter != m_motors.end(); motor_iter++)
    {
        has_init = motor_iter->second->hasInit();
    }

    return has_init;
}

void CEposManager::read(){
    for (MapMotor::iterator motor_iterator = m_motors.begin(); motor_iterator != m_motors.end(); motor_iterator++)
    {
        motor_iterator->second->read();
    }
}

void CEposManager::write(){
    for (MapMotor::iterator motor_iterator = m_motors.begin(); motor_iterator != m_motors.end(); motor_iterator++)
    {
        motor_iterator->second->write();
    }
}