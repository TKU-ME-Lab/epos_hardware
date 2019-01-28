#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include "maxon_hardware/cepos_hardware.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "epos_hardware_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    
}