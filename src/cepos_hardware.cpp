#include "maxon_hardware/cepos_hardware.h"
#include <boost/foreach.hpp>
#include <boost/utility.hpp>
#include <string>
#include <iostream>

CEposHardware::CEposHardware(ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::vector<std::string> motor_names):
                            m_nh(nh), m_private_nh(pnh), m_updater(nh, pnh)
{        
    try{
        m_transmission_loader.reset(new transmission_interface::TransmissionInterfaceLoader(this, &m_robot_transmissions));
    }
    catch (const std::invalid_argument& ex){
        ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
        return;
    }
    catch(...){
        ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
        return;
    }

    std::string urdf_string;
    nh.getParam("robot_description", urdf_string);
    while (urdf_string.empty() && ros::ok()){
        ROS_INFO_STREAM_ONCE("Waiting for robot_description");
        nh.getParam("robot_description", urdf_string);
        ros::Duration(0.1).sleep();
    }

    transmission_interface::TransmissionParser parser;
    std::vector<transmission_interface::TransmissionInfo> infos;
    // TODO: throw exception
    if (!parser.parse(urdf_string, infos)) {
        ROS_ERROR("Error parsing URDF");
        return;
    }

    std::vector<EposParameter> Params;

    BOOST_FOREACH(const std::string &motor_name, motor_names){
        ROS_INFO_STREAM("Get [" + motor_name + "] Parameter");
        
        EposParameter Param;
        Param.motor_name = motor_name;

        ros::NodeHandle motor_config_nh(m_private_nh, motor_name);

        if (motor_config_nh.getParam("actuator", Param.actuator)){
            ROS_INFO_STREAM("Actuator: " + Param.actuator);
            if (motor_config_nh.getParam("protocol", Param.protocol)){
                ROS_INFO_STREAM("Protocol: " + Param.protocol);
                if (motor_config_nh.getParam("is_sub_device", Param.is_sub_device)){
                    if (!Param.is_sub_device){
                        if (motor_config_nh.getParam("interface", Param.interface)){
                            ROS_INFO_STREAM("Interface: " + Param.interface);
                            int _nodeid = 0;
                            if (motor_config_nh.getParam("node_id", _nodeid)){
                                Param.nodeid = (uint16_t)_nodeid;
                                std::stringstream ss;
                                ss << Param.nodeid;
                                ROS_INFO_STREAM("Node ID: " + ss.str());

                                if (motor_config_nh.getParam("serial_number", Param.serial_number)){
                                    if (!motor_config_nh.getParam("operation_mode", Param.mode)){
                                    ROS_WARN("No operation_mode in parameters");
                                    }
                                
                                    if (!motor_config_nh.getParam("clear_fault", Param.clear_fault)){
                                        ROS_WARN("No clear_fault in parameters");
                                    }
                                    
                                    if (!motor_config_nh.hasParam("position_profile")){
                                        ros::NodeHandle position_profile_nh(motor_config_nh, "position_profile");
                                        int _velocity;
                                        int _acceleration;
                                        int _deceleration;
                                        if (position_profile_nh.getParam("velocity", _velocity)){
                                            Param.position_profile.velocity = (unsigned int)_velocity;
                                        }
                                        else{
                                            ROS_WARN("Didn't have velocity in position_profile");
                                        }

                                        if (position_profile_nh.getParam("acceleration", _acceleration)){
                                            Param.position_profile.acceleration = (unsigned int)_acceleration;
                                        }
                                        else{
                                            ROS_WARN("Didn't have acceleration in position_profile");
                                        }

                                        if (position_profile_nh.getParam("deceleration", _deceleration)){
                                            Param.position_profile.deceleration = (unsigned int)_deceleration;
                                        }
                                        else{
                                            ROS_WARN("Didn't have deceleration in position_profile");
                                        }
                                    }

                                    if (!motor_config_nh.hasParam("velocity_profile")){
                                        ros::NodeHandle velocity_profile_nh(motor_config_nh, "velocity_profile");
                                        int _acceleration;
                                        int _deceleration;
                                        if (velocity_profile_nh.getParam("acceleration", _acceleration)){
                                            Param.velocity_profile.acceleration = (unsigned int)_acceleration;
                                        }
                                        else{
                                            ROS_WARN("Didn't have acceleration in velocity_profile");
                                        }

                                        if (velocity_profile_nh.getParam("deceleration", _deceleration)){
                                            Param.velocity_profile.deceleration = (unsigned int)_deceleration;
                                        }
                                        else{
                                            ROS_WARN("Didn't have deceleration in velocity_profile");
                                        }
                                    }

                                    Params.push_back(Param);
                                }
                                else
                                {
                                    ROS_WARN_STREAM(motor_name + ", didn't have serial_number");
                                    continue;
                                }
                                
                            }
                        }
                        else
                        {
                            ROS_WARN_STREAM(motor_name + "is master device,but it didn't have interface");
                            continue;
                        }
                        
                    }
                    else
                    {
                        if (motor_config_nh.getParam("master_device", Param.master_device)){
                            if (motor_config_nh.getParam("serial_number", Param.serial_number)){
                                if (!motor_config_nh.getParam("operation_mode", Param.mode)){
                                    ROS_WARN("No operation_mode in parameters");
                                }
                                
                                if (!motor_config_nh.getParam("clear_fault", Param.clear_fault)){
                                    ROS_WARN("No clear_fault in parameters");
                                }
                                
                                if (!motor_config_nh.hasParam("position_profile")){
                                    ros::NodeHandle position_profile_nh(motor_config_nh, "position_profile");
                                    int _velocity;
                                    int _acceleration;
                                    int _deceleration;
                                    if (position_profile_nh.getParam("velocity", _velocity)){
                                        Param.position_profile.velocity = (unsigned int)_velocity;
                                    }
                                    else{
                                        ROS_WARN("Didn't have velocity in position_profile");
                                    }

                                    if (position_profile_nh.getParam("acceleration", _acceleration)){
                                        Param.position_profile.acceleration = (unsigned int)_acceleration;
                                    }
                                    else{
                                        ROS_WARN("Didn't have acceleration in position_profile");
                                    }

                                    if (position_profile_nh.getParam("deceleration", _deceleration)){
                                        Param.position_profile.deceleration = (unsigned int)_deceleration;
                                    }
                                    else{
                                        ROS_WARN("Didn't have deceleration in position_profile");
                                    }
                                }

                                if (!motor_config_nh.hasParam("velocity_profile")){
                                    ros::NodeHandle velocity_profile_nh(motor_config_nh, "velocity_profile");
                                    int _acceleration;
                                    int _deceleration;
                                    if (velocity_profile_nh.getParam("acceleration", _acceleration)){
                                        Param.velocity_profile.acceleration = (unsigned int)_acceleration;
                                    }
                                    else{
                                        ROS_WARN("Didn't have acceleration in velocity_profile");
                                    }

                                    if (velocity_profile_nh.getParam("deceleration", _deceleration)){
                                        Param.velocity_profile.deceleration = (unsigned int)_deceleration;
                                    }
                                    else{
                                        ROS_WARN("Didn't have deceleration in velocity_profile");
                                    }
                                }

                                Params.push_back(Param);
                            }
                            else{
                                ROS_WARN_STREAM(motor_name + ", didn't have serial_number");
                                continue;
                            }
                        }
                        else
                        {
                            ROS_WARN_STREAM("Param is_sub_device: true, but didn't have master device in params." );
                            continue;
                        }
                        
                    }   
                }
            }
            else{
                ROS_WARN_STREAM(motor_name + ", didn't have protocol");  
                continue;  
            }
        }
        else{
            ROS_WARN_STREAM(motor_name + ", didn't have actuator");
            continue;
        }

        std::cout << std::endl;
    }

    m_EposManager = new CEposManager(Params);

    ROS_INFO("Setup Hardware Interface");
    for (MapMotor::iterator motor_iterator = m_EposManager->GetMotors().begin(); motor_iterator != m_EposManager->GetMotors().end(); motor_iterator++){
        //boost::shared_ptr<CEpos> pEpos(motor_iterator->second);
        ROS_INFO_STREAM(motor_iterator->first + " registerHandle");
        
        hardware_interface::ActuatorStateHandle statehandle(motor_iterator->first, motor_iterator->second->GetPositionPtr(), motor_iterator->second->GetVelocityPtr()
                                                            , motor_iterator->second->GetEffortPtr());
        m_asi.registerHandle(statehandle);

        hardware_interface::ActuatorHandle position_handle(statehandle, motor_iterator->second->GetPositionCmdPtr());
        m_asi.registerHandle(position_handle);
        hardware_interface::ActuatorHandle velocity_handle(statehandle, motor_iterator->second->GetVelocityCmdPtr());
        m_asi.registerHandle(velocity_handle);

        CMotorStatus* MotorStatus = new CMotorStatus(motor_iterator->second);
        m_updater.setHardwareID(motor_iterator->second->serial_number());
        std::stringstream ss;
        ss << motor_iterator->first << " (Status Word): ";
        m_updater.add(ss.str(), boost::bind(&CMotorStatus::Statusword, MotorStatus, _1));
        ss.clear();
        ss << motor_iterator->first << " (Output Status): ";
        m_updater.add(ss.str(), boost::bind(&CMotorStatus::OutputStatus, MotorStatus, _1));
        m_MotorStatus.push_back(MotorStatus);
    }

    registerInterface(&m_asi);
    registerInterface(&m_api);
    registerInterface(&m_avi);
    
    ROS_INFO("Compare Transmmision Interface");
    BOOST_FOREACH(const transmission_interface::TransmissionInfo& info, infos) {
        bool found_some = false;
        bool found_all = true;
        BOOST_FOREACH(const transmission_interface::ActuatorInfo& actuator, info.actuators_) {
            std::vector<std::string> motor_names;
            for (MapMotor::iterator motor_iterator = m_EposManager->GetMotors().begin(); motor_iterator != m_EposManager->GetMotors().end(); motor_iterator++){
                std::string motor_name(motor_iterator->first);
                motor_names.push_back(motor_name);
            }
            if(std::find(motor_names.begin(), motor_names.end(), actuator.name_) != motor_names.end())
	            found_some = true;
            else
	            found_all = false;
        }

        if(found_all) {
            if (!m_transmission_loader->load(info)) {
                ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
            return;
        }
        else
            ROS_INFO_STREAM("Loaded transmission: " << info.name_);
        }
        else if(found_some){
            ROS_ERROR_STREAM("Do not support transmissions that contain only some EPOS actuators: " << info.name_);
        }
    }
}

bool CEposHardware::init(){
    return m_EposManager->init();
}

void CEposHardware::read(){
    m_EposManager->read();
    if (m_robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>()){
        m_robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
    }
}

void CEposHardware::write(){
    m_EposManager->write();
    if (m_robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()){
        m_robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();
    }
    if (m_robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>()){
        m_robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
    }
}

void CEposHardware::update_diagnostics(){
    m_updater.update();
}

CMotorStatus::CMotorStatus(boost::shared_ptr<CEpos> pMotor)
{
    m_motor = pMotor;
}

void CMotorStatus::Statusword(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.add("Actuator Name", m_motor->device_name());
    unsigned int error_code;
    
    const unsigned short statusword = m_motor->statusword();

    bool enabled = STATUSWORD(READY_TO_SWITCH_ON, statusword) && STATUSWORD(SWITCHED_ON, statusword) && STATUSWORD(ENABLE, statusword);
    if(enabled) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
    }
    else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disabled");
    }

    // Quickstop is enabled when bit is unset (only read quickstop when enabled)
    if(!STATUSWORD(QUICK_STOP, statusword) && enabled) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Quickstop");
    }

    if(STATUSWORD(WARNING, statusword)) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Warning");
    }

    if(STATUSWORD(FAULT, statusword)) {
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Fault");
    }   

    stat.add<bool>("Enabled", STATUSWORD(ENABLE, statusword));
    stat.add<bool>("Fault", STATUSWORD(FAULT, statusword));
    stat.add<bool>("Voltage Enabled", STATUSWORD(VOLTAGE_ENABLED, statusword));
    stat.add<bool>("QuicK Stop", STATUSWORD(QUICK_STOP, statusword));
    stat.add<bool>("Warning", STATUSWORD(WARNING, statusword));

    unsigned char num_errors;
    if(VCS_GetNbOfDeviceError(m_motor->GetHANDLE(), m_motor->GetID(), &num_errors, &error_code)) {
        for(int i = 1; i<= num_errors; ++i) {
	        unsigned int error_number;
	        if(VCS_GetDeviceErrorCode(m_motor->GetHANDLE(), m_motor->GetID(), i, &error_number, &error_code)) {
	            std::stringstream error_msg;
	            error_msg << "EPOS Device Error: 0x" << std::hex << error_number;
	            stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
	        }
	        else {
	            std::string error_str;
	            if(GetErrorInfo(error_code, &error_str)) {
	                std::stringstream error_msg;
	                error_msg << "Could not read device error: " << error_str;
	                stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
	            }
	            else {
	                stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read device error");
	            }
	        }
        }
    }
    else {
        std::string error_str;
        if(GetErrorInfo(error_code, &error_str)) {
	        std::stringstream error_msg;
	        error_msg << "Could not read device errors: " << error_str;
        	stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
        }
        else {
	        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read device errors");
        }
    }
}

void CMotorStatus::OutputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    std::string operation_mode_str;
    const unsigned short statusword = m_motor->statusword();
    if(m_motor->GetMode() == PROFILE_POSITION_MODE) {
        operation_mode_str = "Profile Position Mode";
        stat.add("Commanded Position", boost::lexical_cast<std::string>(m_motor->GetPositionCmd()) + " rotations");
    }
    else if(m_motor->GetMode() == PROFILE_VELOCITY_MODE) {
        operation_mode_str = "Profile Velocity Mode";
        stat.add("Commanded Velocity", boost::lexical_cast<std::string>(m_motor->GetVelocityCmd()) + " rpm");
    }
    else {
        operation_mode_str = "Unknown Mode";
    }
    stat.add("Operation Mode", operation_mode_str);
    double nominalcurrent = m_motor->GetNominalCurrent();
    stat.add("Nominal Current", boost::lexical_cast<std::string>(nominalcurrent) + " A");
    stat.add("Max Current", boost::lexical_cast<std::string>(m_motor->GetMaxCurrent()) + " A");

    unsigned int error_code;
    stat.add("Position", boost::lexical_cast<std::string>(m_motor->GetPosition()) + " rotations");
    stat.add("Velocity", boost::lexical_cast<std::string>(m_motor->GetVelocity()) + " rpm");
    stat.add("Torque", boost::lexical_cast<std::string>(m_motor->GetEffort()) + " Nm");
    double current = m_motor->GetCurrent();
    stat.add("Current", boost::lexical_cast<std::string>(current) + " A");


    stat.add<bool>("Target Reached", STATUSWORD(TARGET_REACHED, statusword));
    stat.add<bool>("Current Limit Active", STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword));


    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EPOS operating in " + operation_mode_str);
    if(STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword))
        stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Current Limit Active");
    if(nominalcurrent > 0 && std::abs(current) > nominalcurrent) {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Nominal Current Exceeded (Current: %f A)", current);
    }
}