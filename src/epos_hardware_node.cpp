#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include "maxon_hardware/cepos_hardware.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "epos_hardware_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::vector<std::string> motor_names;
    for (int i = 0; i < argc-1; ++i){
        motor_names.push_back(argv[i+1]);
    }

    CEposHardware robot(nh, pnh, motor_names);
    controller_manager::ControllerManager CM(&robot, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    if (!robot.init()){
        ROS_FATAL("Failed to initializer motors");
        return 0;
    }

    ros::Rate controller_rate(50);
    ros::Time last = ros::Time::now();
    while(ros::ok()){
        robot.read();
        ros::Time now = ros::Time::now();
        CM.update(now, now-last);
        robot.write();
        last = now;
        robot.update_diagnostics();
        controller_rate.sleep();
    }

    return 1;
}