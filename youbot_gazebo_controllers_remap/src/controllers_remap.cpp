#include <ros/ros.h>
#include <youbot_gazebo_controllers_remap/ControllersRemap.hpp>
#include <std_msgs/Float64.h>

template <typename type>
std::string toString(type arg){
    std::stringstream stringstream;
    stringstream << arg;
    std::string result;
    stringstream >> result;
    return result;
}

ControllersRemap::ControllersRemap(){
    for(int i = 0; i < 5; i++){
        std::string topicName = "/arm_1/joint_" + toString(i+1) + "_position_controller/command";
        gazeboPositionCommandPulishers.push_back(nodeHandle.advertise<std_msgs::Float64>(topicName.c_str(), 1));
    }
    jointsPositionCommandSubscriber = nodeHandle.subscribe("/arm_1/arm_controller/position_command", 1, &ControllersRemap::jointsPositionCommandCallback, this);
}

ControllersRemap::~ControllersRemap(){

}

void ControllersRemap::jointsPositionCommandCallback(brics_actuator::JointPositions jointPositions){
    for(int i = 0; i < jointPositions.positions.size(); i++){
        std::string str = "arm_joint_";
        if(jointPositions.positions[i].joint_uri.substr(0, str.size()) != str) continue;
        size_t numberBegin = jointPositions.positions[i].joint_uri.find_first_of("0123456789");
        if (numberBegin >= jointPositions.positions[i].joint_uri.size()) continue;
        int jointNumber = atoi(jointPositions.positions[i].joint_uri.c_str() + numberBegin);
        if(!(jointNumber >= 0 && jointNumber < gazeboPositionCommandPulishers.size())) continue;
        if(jointNumber == 5) continue;
        std_msgs::Float64 value;
        value.data = jointPositions.positions[i].value;
        gazeboPositionCommandPulishers[jointNumber - 1].publish(value);
    }
}
