#include <ros/ros.h>
#include <youbot_dashboard/youbot_battery_monitor.h>
#include <youbot_dashboard/SetDisplay.h>
#include <sensor_msgs/BatteryState.h>
#include <string>

#define DISPLAY_LINES 2
#define DISPLAY_LINE_LENGTH 16

youbot::YoubotBatteryMonitor youbotDashboard;

bool displayCallback(youbot_dashboard::SetDisplayRequest& request, youbot_dashboard::SetDisplayResponse& response){
    std::vector<std::string> textlines = request.textlines;
    if (textlines.size() > DISPLAY_LINES)
        textlines.resize(DISPLAY_LINES);
    while(textlines.size() != DISPLAY_LINES)
        textlines.push_back("");
    for(int i = 0; i < textlines.size(); i++)
        if (textlines[i].size() > DISPLAY_LINE_LENGTH)
            textlines[i].resize(DISPLAY_LINE_LENGTH);
    if (!textlines[0].empty()) youbotDashboard.setYoubotDisplayText(youbot::DisplayLine::line2, textlines[0]);
    if (!textlines[1].empty()) youbotDashboard.setYoubotDisplayText(youbot::DisplayLine::line3, textlines[1]);
    return 1;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_dashboard");
    ros::NodeHandle nodeHandle("~");

    std::string devName = nodeHandle.param<std::string>("device_path", "/dev/dashboard");
    double pubRate = nodeHandle.param<double>("pub_rate", 10);

    while(!youbotDashboard.connect(devName))
        ros::Duration(0.1).sleep();

    sensor_msgs::BatteryState batteryState;
    batteryState.cell_voltage.resize(2);
    batteryState.header.frame_id = "base_link";
    batteryState.header.seq = 0;

    ros::Publisher batteryStatePublisher = nodeHandle.advertise<sensor_msgs::BatteryState>("/youbot_dashboard/battery_state", 1);
    ros::ServiceServer displayServiceServer = nodeHandle.advertiseService("/youbot_dashboard/display", displayCallback);

    ros::Rate rate(pubRate);
    while(ros::ok()){

        batteryState.header.stamp = ros::Time::now();
        batteryState.cell_voltage[0] = youbotDashboard.getVoltage(youbot::VoltageSource::battery1);
        batteryState.cell_voltage[1] = youbotDashboard.getVoltage(youbot::VoltageSource::battery2);
        batteryState.charge = youbotDashboard.getVoltage(youbot::VoltageSource::powersupply);
        batteryState.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        batteryState.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        batteryState.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        batteryStatePublisher.publish(batteryState);

        batteryState.header.seq++;

        ros::spinOnce();
        rate.sleep();
    }

    youbotDashboard.disconnect();

    ros::Duration(1).sleep();
    ros::shutdown();

    return 0;
}
