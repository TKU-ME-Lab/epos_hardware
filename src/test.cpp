#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>
//#include "maxon_hardware/Definitions.h"
//#include "maxon_hardware/util.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "maxon_test");
    
    ros::NodeHandle nh;
    ros::NodeHandle config_nh("~");

    std::vector<std::string> device_names;
    for(int i = 0; i < argc-1; ++i){
        device_names.push_back(argv[i+1]);
    }

    BOOST_FOREACH(const std::string& name, device_names){
        std::cout << "Device Name: " << name << std::endl;
        bool is_sub_device = false;

        ros::NodeHandle motor_config_nh(config_nh, name);
        if (!motor_config_nh.getParam("is_sub_device", is_sub_device)){
            ROS_ERROR("Config Don't have is_sub_device");
        }
        else{
            if (!is_sub_device){
                std::string protocol;
                std::string interface;
                if (!motor_config_nh.getParam("protocol", protocol)){
                    ROS_ERROR("Config Don't have Protocol");
                }
                else{
                    std::cout << "Protocol: " << protocol << std::endl;
                }
                
                if (!motor_config_nh.getParam("interface", interface)){
                    ROS_ERROR("Config Don't have Interface");
                }
                else{
                    std::cout << "Interface: " << interface << std::endl;
                }
            }
            else{
                ROS_INFO("This device is sub device.");
            }
        }

        std::cout << "---------------------------------" << std::endl;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}