#include "maxon_hardware/cepos_manager.h"
#include <boost/foreach.hpp>

CEposManager::CEposManager(std::vector<EposParameter> Params){
    BOOST_FOREACH(const EposParameter &Param, Params){
        HANDLE keyHandle;
        if (CreateDeviceKeyHandle(Param.actuator, Param.protocol, Param.interface, Param.nodeid, Param.serial_number, &keyHandle)){

        }
    }
}

std::map<std::string, boost::shared_ptr<CEpos> > CEposManager::GetMotors(){
    return m_motors;
}