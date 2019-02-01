#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>
//#include "maxon_hardware/Definitions.h"
#include "maxon_hardware/util.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "maxon_test");
    
    ros::NodeHandle nh;
    ros::NodeHandle config_nh("~");

    std::string str = "602079024057";
    uint64_t str_num = 0;

    SerialNumberFromHex(str, &str_num);

    std::cout << str_num << std::endl;

    return 0;
}