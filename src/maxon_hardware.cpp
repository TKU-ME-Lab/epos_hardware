#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "maxon_hardware/Definitions.h"

typedef void* HANDLE;

std::string device = "EPOS2";
std::string protocol = "MAXON SERIAL V2";
std::string interface = "USB";
std::string port = "USB0";
unsigned int error_code = 0;
HANDLE handle = NULL;

void velocity_callback(const std_msgs::Float64ConstPtr& msg){
    if (handle){
        std::cout << "Set Velocity: " << msg->data << std::endl;
       int result = VCS_MoveWithVelocity(handle, 1, (int)msg->data, &error_code);
        if (result){
            std::cout << "Moving" << std::endl;
        }
        else{
            std::cout << "Result: " << result << std::endl;
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "maxon_hardware");
    
    ros::NodeHandle nh;
    ros::Subscriber velocity_subsriber = nh.subscribe("/velocity_cmd", 10, velocity_callback);

    handle = VCS_OpenDevice((char*)device.c_str(), (char*)protocol.c_str(), (char*)interface.c_str(), (char*)port.c_str(), &error_code);
    if (!handle){
        return 0;
    }

    int result = VCS_ClearFault(handle, 1, &error_code);
    if (result){
        result = VCS_ActivateProfileVelocityMode(handle, 1, &error_code);
        if (result){
            result = VCS_SetVelocityProfile(handle, 1, 100, 100, &error_code);
            if (result){
                result = VCS_SetEnableState(handle, 1, &error_code);
                if (!result){
                    return 0;
                }
            }
        }

    }
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}