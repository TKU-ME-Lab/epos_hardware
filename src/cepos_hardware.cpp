#include "maxon_hardware/cepos_hardware.h"
#include <boost/foreach.hpp>
#include <string>

CEposHardware::CEposHardware(ros::NodeHandle &pnh, const std::vector<std::string> motor_names):m_private_nh(pnh)
{    
    ROS_INFO("CEposHardware Init");
    // std::map<std::string, boost::shared_ptr<CEpos> > motors;
    
    std::vector<EposParameter> Params;

    BOOST_FOREACH(const std::string &motor_name, motor_names){
        ROS_INFO_STREAM("Get [" + motor_name + "] Parameter------------------------------------------------");
        std::string actuator_name = "";
        std::string protocol = "";
        std::string interface = "";
        std::string serial_number = "";
        std::string profile_mode = "";
        int id;
        bool clear_fault = true;
        bool is_sub_device = false;
        
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
                            if (motor_config_nh.getParam("is_sub_device", is_sub_device)){
                                if (!motor_config_nh.getParam("operation_mode", Param.mode)){
                                    ROS_WARN("No operation_mode in parameters");
                                }

                                if (!motor_config_nh.getParam("operation_mode", Param.mode)){
                                    ROS_WARN("No operation_mode in parameters");
                                }
                            }
                        }
                    }
                }
            }
        }

        // if (motor_config_nh.getParam("is_sub_device", is_sub_device)){
        //     if (!is_sub_device){
        //         if (motor_config_nh.getParam("actuator", actuator_name)){
        //             if (motor_config_nh.getParam("protocol", protocol)){
        //                 if (motor_config_nh.getParam("interface", interface)){
        //                     if (motor_config_nh.getParam("node_id", id)){
        //                         if (!motor_config_nh.getParam("serial_number", serial_number)){
        //                             ROS_WARN_STREAM("Device: " << motor_name << "- Didn't have serial_number parameter.");
        //                         }
        //                         if (!motor_config_nh.getParam("profile_mode", profile_mode)){
        //                             ROS_WARN_STREAM("Device: " << motor_name << "- Didn't have profile_mode parameter.");
        //                         }
        //                         // if (!motor_config_nh.getParam("clear_fault", clear_faults)){
        //                         //     ROS_WARN_STREAM("Device: " << motor_name << "- Didn't have clear_faults parameter.");
        //                         // }
        //                         //boost::shared_ptr<CEpos> motor(new CEpos())
                                
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }


    }
}

