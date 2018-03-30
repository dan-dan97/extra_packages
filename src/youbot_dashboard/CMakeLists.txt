cmake_minimum_required(VERSION 2.8.3)
project(youbot_dashboard)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	message_generation
)

add_service_files(FILES
		SetDisplay.srv
		)

generate_messages(DEPENDENCIES
		std_msgs
		)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
	include
#	${CATKIN_DEVEL_PREFIX}/include
	${catkin_INCLUDE_DIRS}
)

add_library(dashboard_lib SHARED
	include/${PROJECT_NAME}/youbot_battery_monitor.h
	src/youbot_battery_monitor.cpp
)

target_link_libraries(dashboard_lib
  ${catkin_LIBRARIES}
)

add_executable(youbot_dashboard
	src/youbot_dashboard.cpp
)

add_dependencies(youbot_dashboard youbot_dashboard_generate_messages_cpp)

target_link_libraries(youbot_dashboard
  ${catkin_LIBRARIES}
	dashboard_lib
)
