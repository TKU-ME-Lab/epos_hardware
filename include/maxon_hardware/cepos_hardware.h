#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_loader.h>

#include "maxon_hardware/cepos_manager.h"

class CEposHardware : public hardware_interface::RobotHW {
private:
    ros::NodeHandle m_private_nh;
    CEposManager* m_EposManager;

    std::vector<hardware_interface::ActuatorStateHandle> m_ActuatorStateHandles;

    hardware_interface::ActuatorStateInterface m_asi;
    hardware_interface::VelocityActuatorInterface m_avi;
    hardware_interface::PositionActuatorInterface m_api;

    transmission_interface::RobotTransmissions m_robot_transmissions;

    boost::shared_ptr<transmission_interface::TransmissionLoader> m_transmission_loader;

    
public:
    CEposHardware(ros::NodeHandle&, const std::vector<std::string>);

    void read();
    void write();
    void update_diagnostics();

};