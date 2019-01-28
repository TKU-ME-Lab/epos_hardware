#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>
//#include "maxon_hardware/Definitions.h"
//#include "maxon_hardware/util.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "maxon_hardware");
    
    ros::NodeHandle nh;

    std::vector<std::string> device_names;
    for(int i = 0; i < argc-1; ++i){
        device_names.push_back(argv[i+1]);
    }

    BOOST_FOREACH(const std::string& name, device_names){
        std::cout << "Device Name: " << name << std::endl;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}