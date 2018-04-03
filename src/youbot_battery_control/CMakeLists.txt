cmake_minimum_required(VERSION 2.8.3)
project(youbot_battery_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)

catkin_package(
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(youbot_battery_control
	src/youbot_battery_control.cpp
)

target_link_libraries(youbot_battery_control
	${catkin_LIBRARIES}
	pthread
)
