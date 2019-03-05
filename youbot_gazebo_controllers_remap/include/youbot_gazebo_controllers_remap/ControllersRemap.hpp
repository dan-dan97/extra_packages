#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>

class ControllersRemap{
public:

    ControllersRemap();
    ~ControllersRemap();

private:

    ros::NodeHandle nodeHandle;

    std::vector<ros::Publisher> gazeboPositionCommandPulishers;

    ros::Subscriber jointsPositionCommandSubscriber;

    void jointsPositionCommandCallback(brics_actuator::JointPositions jointPositions);

};
