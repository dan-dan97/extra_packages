#include <ros/ros.h>
#include <youbot_gazebo_controllers_remap/ControllersRemap.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_gazebo_controllers_remap");

    ControllersRemap controllersRemap;

    ros::Rate rate(200);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Duration(1).sleep();
    ros::shutdown();

    return 0;
}
