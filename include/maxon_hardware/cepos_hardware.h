#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_loader.h>

#include "maxon_hardware/cepos.h"

class CEposHardware : public hardware_interface::RobotHW {
private:


};