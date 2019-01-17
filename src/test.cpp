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
std::string subprotocol = "CANopen";
std::string interface = "USB";

std::string port = "USB0";
unsigned int error_code = 0;
HANDLE handle = NULL;
HANDLE subhandle = NULL;

void velocity1_callback(const std_msgs::Float64ConstPtr& msg){
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

void velocity2_callback(const std_msgs::Float64ConstPtr& msg){
    if(handle){
        std::cout << "Set Velocity: " << msg->data << std::endl;
        int result = VCS_MoveWithVelocity(handle, 2, (int)msg->data, &error_code);
        if (result){
            std::cout << "Moving" << std::endl;
        }
        else{
            std::cout << "Result: " << result << std::endl;
            std::cout << "Error: " << error_code << std::endl;
        }
    }
}

void velocity3_callback(const std_msgs::Float64ConstPtr& msg){
    if(handle){
        std::cout << "Set Velocity: " << msg->data << std::endl;
        int result = VCS_MoveWithVelocity(handle, 3, (int)msg->data, &error_code);
        if (result){
            std::cout << "Moving" << std::endl;
        }
        else{
            std::cout << "Result: " << result << std::endl;
            std::cout << "Error: " << error_code << std::endl;
        }
    }
}

void velocity4_callback(const std_msgs::Float64ConstPtr& msg){
    if(handle){
        std::cout << "Set Velocity: " << msg->data << std::endl;
        int result = VCS_MoveWithVelocity(handle, 4, (int)msg->data, &error_code);
        if (result){
            std::cout << "Moving" << std::endl;
        }
        else{
            std::cout << "Result: " << result << std::endl;
            std::cout << "Error: " << error_code << std::endl;
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "maxon_hardware");
    
    ros::NodeHandle nh;
    ros::Subscriber velocity_subscriber = nh.subscribe("/epos/1/velocity_cmd", 10, velocity1_callback);
    ros::Subscriber velocity_subscriber1 = nh.subscribe("/epos/2/velocity_cmd", 10, velocity2_callback);
    ros::Subscriber velocity_subscriber2 = nh.subscribe("/epos/3/velocity_cmd", 10, velocity3_callback);
    ros::Subscriber velocity_subscriber3 = nh.subscribe("/epos/4/velocity_cmd", 10, velocity4_callback);

    handle = VCS_OpenDevice((char*)device.c_str(), (char*)protocol.c_str(), (char*)interface.c_str(), (char*)port.c_str(), &error_code);
    if (!handle){
        return 0;
    }
    else{
        int result = VCS_ClearFault(handle, 1, &error_code);
        if (result){
            std::cout << "Activate Node 1" << std::endl;
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

        result = VCS_ClearFault(handle, 2, &error_code);
        if (result){
            std::cout << "Activate Node 2" << std::endl;
            result = VCS_ActivateProfileVelocityMode(handle, 2, &error_code);
            if (result){
                std::cout << "Set VP" << std::endl;
                result = VCS_SetVelocityProfile(handle, 2, 100, 100, &error_code);
                if (result){
                    result = VCS_SetEnableState(handle, 2, &error_code);
                    if (!result){
                        return 0;
                    }
                }
            }
        }

        result = VCS_ClearFault(handle, 3, &error_code);
        if (result){
            std::cout << "Activate Node 3" << std::endl;
            result = VCS_ActivateProfileVelocityMode(handle, 3, &error_code);
            if (result){
                std::cout << "Set VP" << std::endl;
                result = VCS_SetVelocityProfile(handle, 3, 100, 100, &error_code);
                if (result){
                    result = VCS_SetEnableState(handle, 3, &error_code);
                    if (!result){
                        return 0;
                    }
                }
            }
        }

        result = VCS_ClearFault(handle, 4, &error_code);
        if (result){
            std::cout << "Activate Node 4" << std::endl;
            result = VCS_ActivateProfileVelocityMode(handle, 4, &error_code);
            if (result){
                std::cout << "Set VP" << std::endl;
                result = VCS_SetVelocityProfile(handle, 4, 100, 100, &error_code);
                if (result){
                    result = VCS_SetEnableState(handle, 4, &error_code);
                    if (!result){
                        return 0;
                    }
                }
            }
        }
        // subhandle = VCS_OpenSubDevice(handle, (char*)device.c_str(), (char*)subprotocol.c_str(), &error_code);
        // if (subhandle){
        //     std::cout << "Open Sub Device" << std::endl;
        //     int result = VCS_ClearFault(subhandle, 1, &error_code);
        //     std::cout << "ClearFault, Result: " << result << std::endl;
        //     if (result){
        //         std::cout << "Activate PVM,";
        //         result = VCS_ActivateProfileVelocityMode(subhandle, 1, &error_code);
        //         std::cout << "Result" << result << std::endl;
        //         if (result){
        //             std::cout << "Set VP, ";
        //             result = VCS_SetVelocityProfile(subhandle, 1, 100, 100, &error_code);
        //             std::cout << "Result" << result << std::endl; 
        //             if (result){
        //                 std::cout << "Enable State, ";
        //                 result = VCS_SetEnableState(subhandle, 1, &error_code);
        //                 std::cout << "Result" << result << std::endl; 
        //                 if (!result){
        //                     return 0;
        //                 }
        //             }
        //         }
        //     }
        // }
    }    
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}