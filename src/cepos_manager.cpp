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
            
            if (iter != m_motors.end()){
                //boost::shared_ptr<CEpos> motor(new CEpos(Param.actuator, Param.protocol, Param, Param.nodeid, Param.serial_number, ))
            }
        }
    }
}

std::map<std::string, boost::shared_ptr<CEpos> > CEposManager::GetMotors(){
    return m_motors;
}