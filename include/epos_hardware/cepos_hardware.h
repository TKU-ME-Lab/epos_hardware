#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.hpp>

#include "epos_hardware/cepos_manager.h"

class CMotorStatus{
private:
    CEpos* m_pmotor;

public:
    CMotorStatus(CEpos*);

    void Statusword(diagnostic_updater::DiagnosticStatusWrapper&);
    void OutputStatus(diagnostic_updater::DiagnosticStatusWrapper&);
};    

class CEposHardware : public hardware_interface::RobotHW {
private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    CEposManager* m_EposManager;

    hardware_interface::ActuatorStateInterface m_asi;
    hardware_interface::VelocityActuatorInterface m_avi;

    transmission_interface::RobotTransmissions m_robot_transmissions;

    boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> m_transmission_loader;

    diagnostic_updater::Updater m_updater;
    std::vector<CMotorStatus*> m_MotorStatus;

public:
    CEposHardware(ros::NodeHandle&, ros::NodeHandle&, const std::vector<std::string>);

    bool init();

    void read();
    void write();
    void update_diagnostics();

};

