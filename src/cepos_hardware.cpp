#include "maxon_hardware/cepos_hardware.h"
#include <boost/foreach.hpp>
#include <boost/utility.hpp>
#include <string>

CEposHardware::CEposHardware(ros::NodeHandle &pnh, const std::vector<std::string> motor_names):m_private_nh(pnh)
{        
    std::vector<EposParameter> Params;

    BOOST_FOREACH(const std::string &motor_name, motor_names){
        ROS_INFO_STREAM("Get [" + motor_name + "] Parameter------------------------------------------------");
        
        EposParameter Param;

        ros::NodeHandle motor_config_nh(m_private_nh, motor_name);

        if (motor_config_nh.getParam("actuator", Param.actuator)){
            ROS_INFO_STREAM("Actuator: " + Param.actuator);
            if (motor_config_nh.getParam("protocol", Param.protocol)){
                ROS_INFO_STREAM("Protocol: " + Param.protocol);
                if (motor_config_nh.getParam("interface", Param.interface)){
                    ROS_INFO_STREAM("Interface: " + Param.interface);
                    int _nodeid = 0;
                    if (motor_config_nh.getParam("node_id", _nodeid)){
                        Param.nodeid = (uint16_t)_nodeid;
                        std::stringstream ss;
                        ss << Param.nodeid;
                        ROS_INFO_STREAM("Node ID: " + ss.str());
                        
                        if (motor_config_nh.getParam("serial_number", Param.serial_number)){
                            if (motor_config_nh.getParam("is_sub_device", Param.is_sub_device)){
                                if (Param.is_sub_device){
                                    if (!motor_config_nh.getParam("master_device", Param.master_device)){
                                        ROS_WARN_STREAM("Param is_sub_device: true, but didn't have master device in params." );
                                    }
                                }
                            }

                            if (!motor_config_nh.getParam("operation_mode", Param.mode)){
                                ROS_WARN("No operation_mode in parameters");
                            }
                            
                            if (!motor_config_nh.getParam("clear_fault", Param.clear_fault)){
                                ROS_WARN("No clear_fault in parameters");
                            }

                            Params.push_back(Param);
                        }
                        else{
                            ROS_WARN_STREAM("Motor:" + motor_name + ", didn't have serial_number");
                            continue;
                        }
                    }
                    else{
                        ROS_WARN_STREAM("Motor:" + motor_name + ", didn't have node_id");
                        continue;
                    }
                }
                else{
                    ROS_WARN_STREAM("Motor:" + motor_name + ", didn't have interface");
                    continue;
                }
            }
            else{
                ROS_WARN_STREAM("Motor:" + motor_name + ", didn't have protocol");  
                continue;  
            }
        }
        else{
            ROS_WARN_STREAM("Motor:" + motor_name + ", didn't have actuator");
            continue;
        }
    }    

    
    for (MapMotor::iterator motor_iterator = m_EposManager->GetMotors().begin(); motor_iterator != m_EposManager->GetMotors().end(); motor_iterator++){
        boost::shared_ptr<CEpos> pEpos(motor_iterator->second);
        
        hardware_interface::ActuatorStateHandle statehandle(pEpos->device_name(), pEpos->GetPosition(), pEpos->GetVelocity(), pEpos->GetEffort());
        m_asi.registerHandle(statehandle);

        hardware_interface::ActuatorHandle position_handle(statehandle, pEpos->GetPositionCmd());
        m_asi.registerHandle(position_handle);
        hardware_interface::ActuatorHandle velocity_handle(statehandle, pEpos->GetVelocityCmd());
        m_asi.registerHandle(velocity_handle);
    }
}

