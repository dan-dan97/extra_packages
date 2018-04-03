#include <ros/ros.h>
#include <pthread.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sensor_msgs/BatteryState.h>
#include <string>

double powerMinVoltage, cellCriticalMinVoltage, cellWarningMinVoltage;
double timeToFreeze, timeToCriticalState, timeToWarningState, timeToGoodState, timeToNotPowerState;
double warningStateCommandInterval, criticalStateCommandInterval;
std::string warningCommand, criticalCommand;

enum BATTERY_STATE { GOOD, CHARGE, WARNING, CRITICAL };
BATTERY_STATE batteryState;

ros::Time lastBatteryStateUpdateTime;
ros::Time lastNotGoodStateTime;
ros::Time lastNotWarningStateTime;
ros::Time lastNotCriticalStateTime;
ros::Time lastPowerTime;

void batteryStateCallback(const sensor_msgs::BatteryState& batteryStateMessage){
    lastBatteryStateUpdateTime = ros::Time::now();

    bool currentCriticalState = 0;
    bool currentWarningState = 0;
    bool currentPowerState = 0;
    for(auto cellVoltage : batteryStateMessage.cell_voltage)
        if (cellVoltage <= cellCriticalMinVoltage)
            currentCriticalState = 1;
        else if (cellVoltage <= cellWarningMinVoltage)
            currentWarningState = 1;
    if (batteryStateMessage.charge > powerMinVoltage)
        currentPowerState = 1;

    if (!currentCriticalState)
        lastNotCriticalStateTime = ros::Time::now();
    if (!currentWarningState)
        lastNotWarningStateTime = ros::Time::now();
    if (currentWarningState || currentCriticalState)
        lastNotGoodStateTime = ros::Time::now();
    if (currentPowerState)
        lastPowerTime = ros::Time::now();

    currentCriticalState = ((ros::Time::now() - lastNotCriticalStateTime).toSec() >= timeToCriticalState);
    currentWarningState = currentCriticalState || ((ros::Time::now() - lastNotWarningStateTime).toSec() >= timeToWarningState);
    bool currentGoodState = ((ros::Time::now() - lastNotGoodStateTime).toSec() >= timeToGoodState);
    currentPowerState = !((ros::Time::now() - lastPowerTime).toSec() >= timeToNotPowerState);

    if (currentPowerState){
        batteryState = BATTERY_STATE::CHARGE;
        lastNotCriticalStateTime = ros::Time::now();
        lastNotWarningStateTime = ros::Time::now();
    }
    else{
        if (currentCriticalState)
            batteryState = BATTERY_STATE::CRITICAL;
        else if(currentWarningState)
            batteryState = BATTERY_STATE::WARNING;
        else if(currentGoodState)
            batteryState = BATTERY_STATE::GOOD;
    }
}

void* watchDogFunction(void *arg){
    bool& killThread = *((bool*) arg);
    while(!killThread){
        static BATTERY_STATE batteryStateOld = batteryState;
        static ros::Time lastWarningCommandTime = ros::Time::now();
        static ros::Time lastCriticalCommandTime = ros::Time::now();

        if ((ros::Time::now() - lastBatteryStateUpdateTime).toSec() >= timeToFreeze) continue;
        switch (batteryState){
            case BATTERY_STATE::WARNING:
                if (batteryState != batteryStateOld || ((ros::Time::now() - lastWarningCommandTime).toSec() >= warningStateCommandInterval && warningStateCommandInterval >= 0)){
                    lastWarningCommandTime = ros::Time::now();
                    system(warningCommand.c_str());
                }
                break;

            case BATTERY_STATE::CRITICAL:
                if (batteryState != batteryStateOld || ((ros::Time::now() - lastCriticalCommandTime).toSec() >= criticalStateCommandInterval && criticalStateCommandInterval >= 0)){
                    lastCriticalCommandTime = ros::Time::now();
                    system(criticalCommand.c_str());
                }
                break;
        }

        batteryStateOld = batteryState;
        ros::Duration(0.001).sleep();
    }
    return NULL;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "youbot_battery_control");
    ros::NodeHandle nodeHandle("~");

    lastBatteryStateUpdateTime = ros::Time::now();
    lastNotGoodStateTime = ros::Time::now();
    lastNotWarningStateTime = ros::Time::now();
    lastNotCriticalStateTime = ros::Time::now();
    lastPowerTime = ros::Time::now();

    batteryState = BATTERY_STATE::GOOD;

    powerMinVoltage = nodeHandle.param<double>("charge_voltage", 12);
    cellCriticalMinVoltage = nodeHandle.param<double>("critical_cell_voltage", 11);
    cellWarningMinVoltage = nodeHandle.param<double>("waning_cell_voltage", 11.5);
    timeToFreeze = nodeHandle.param<double>("no_info_sleep_time", 3);
    timeToCriticalState = nodeHandle.param<double>("time_to_critical_state", 1);
    timeToWarningState = nodeHandle.param<double>("time_to_warning_state", 1);
    timeToGoodState = nodeHandle.param<double>("time_to_good_state", 1);
    timeToNotPowerState = nodeHandle.param<double>("time_to_not_power_state", 0.1);
    warningStateCommandInterval = nodeHandle.param<double>("warning_command_interval", 60);
    criticalStateCommandInterval = nodeHandle.param<double>("critical_command_interval", 0);
    warningCommand = nodeHandle.param<std::string>("warning_command", "/usr/local/etc/beep_sounds/ring");
    criticalCommand = nodeHandle.param<std::string>("critical_command", "/usr/local/etc/beep_sounds/alarm");

    ros::Subscriber batteryStateSubscriber = nodeHandle.subscribe("/youbot_dashboard/battery_state", 10, batteryStateCallback);

    pthread_t watchDogThread;
    bool killThread = false;
    pthread_create(&watchDogThread, NULL, watchDogFunction, &killThread);

    ros::spin();
    killThread = true;

    pthread_cancel(watchDogThread);
    pthread_join(watchDogThread, NULL);

    ros::Duration(0.1).sleep();
    ros::shutdown();

    return 0;
}
