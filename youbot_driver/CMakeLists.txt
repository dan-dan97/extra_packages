PROJECT(youbot_driver)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

#OPTION(USE_ROS "Enable ROS as compile tool" ON)

########### ROS ###########
#SET(ROS_ROOT_PATH $ENV{ROS_ROOT})
#IF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
#  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#  rosbuild_init()
#  ADD_DEFINITIONS(-DUSE_ROS_LOGGING)
#ENDIF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
)

ADD_DEFINITIONS(-DUSE_ROS_LOGGING)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES YouBotDriver soem
#  CATKIN_DEPENDS luh_youbot_msgs opencv2 roscpp tf
#  DEPENDS system_lib
)

########### cmake configuration and compiler flags ###########
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_C_FLAGS_RELEASE "-fPIC -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-fPIC -Wall")
#SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #static libraries
#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) #executables
#SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) #dynamic libraries (plugins)

# find required and optional dependencies

########### Boost ###########
#SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)
IF (Boost_FOUND)
  LINK_LIBRARIES(boost_thread boost_date_time boost_filesystem boost_system boost_regex)
  MESSAGE(STATUS "SUCCESSFUL: Boost found")
ELSE (Boost_FOUND)
  MESSAGE(STATUS "ERROR: Boost not found")
ENDIF (Boost_FOUND)

########### Doxygen ###########
#SET(DOXYFILE_LATEX "NO")
#SET(DOXYFILE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/doc)
#include(UseDoxygen)

########### Simple Open EtherCAT Master Library ###########
#add_subdirectory(soem)

INCLUDE_DIRECTORIES(
    include
	${catkin_INCLUDE_DIRS}
)

########### CppUnit and tests ###########
FIND_PACKAGE(CppUnit)
IF(CppUnit_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: CPP_UNIT found")
  add_subdirectory(testing)
#ELSE(CppUnit_FOUND)
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
ENDIF (CppUnit_FOUND)

########### youBot display example application ###########
add_subdirectory(display)

SET(YOUBOT_DRIVER_SRC
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMaster.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMasterWithThread.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMasterWithoutThread.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/Logger.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/ConfigFile.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/PidController.cpp
  ${PROJECT_SOURCE_DIR}/src/generic-joint/JointData.cpp
  ${PROJECT_SOURCE_DIR}/src/generic-joint/JointTrajectory.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJoint.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/JointLimitMonitor.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotBase.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotManipulator.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripper.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripperBar.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripperParameter.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/DataTrace.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/GripperDataTrace.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameter.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameterReadOnly.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameterPasswordProtected.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/JointTrajectoryController.cpp
  ${PROJECT_SOURCE_DIR}/src/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  ${PROJECT_SOURCE_DIR}/src/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)


SET(YOUBOT_DRIVER_HEADERS
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/EthercatMaster.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/EthercatMasterInterface.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/EthercatMasterWithThread.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/EthercatMasterWithoutThread.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotGripper.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotGripperBar.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotGripperParameter.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotBase.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotManipulator.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotJoint.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/JointLimitMonitor.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotJointParameterReadOnly.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotJointParameterPasswordProtected.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotJointParameter.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotSlaveMailboxMsg.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotSlaveMsg.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/ProtocolDefinitions.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/YouBotJointStorage.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/DataTrace.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/GripperDataTrace.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/youbot/JointTrajectoryController.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/base-kinematic/WheeledBaseKinematic.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/Exceptions.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/Logger.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/Time.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/Units.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/ConfigFile.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/PidController.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/DataObjectLockFree.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/target.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_arch.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_gcc/oro_arch.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_i386/oro_arch.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_msvc/oro_arch.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree//os/oro_powerpc/oro_arch.h  
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_powerpc/oro_atomic.h  
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_powerpc/oro_system.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-gripper/Gripper.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-gripper/GripperData.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-gripper/GripperParameter.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-joint/Joint.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-joint/JointData.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-joint/JointTrajectory.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/generic-joint/JointParameter.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/one-dof-gripper/OneDOFGripper.hpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/one-dof-gripper/OneDOFGripperData.hpp
)

SET(YOUBOT_DRIVER_CONFIG
  ${PROJECT_SOURCE_DIR}/config/youbot-ethercat.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-base.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-manipulator.cfg
)

ADD_LIBRARY(YouBotDriver ${YOUBOT_DRIVER_SRC})

ADD_LIBRARY(soem 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatbase.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatcoe.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatconfig.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatfoe.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatsoe.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatmain.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatprint.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/nicdrv.c 
  ${PROJECT_SOURCE_DIR}/src/soem/src/ethercatdc.c
)

TARGET_LINK_LIBRARIES(YouBotDriver soem)
ADD_LIBRARY(YouBotDriverShared SHARED "")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES OUTPUT_NAME "YouBotDriver")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(YouBotDriverShared -Wl,-whole-archive YouBotDriver soem -Wl,-no-whole-archive)

MACRO(COPY_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    FILE(COPY ${HEADER} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(COPY_HEADERS_WITH_DIRECTORY)
COPY_HEADERS_WITH_DIRECTORY(YOUBOT_DRIVER_HEADERS)

INSTALL(TARGETS YouBotDriver YouBotDriverShared RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

#ADD_EXECUTABLE(YouBot_test
#  ${PROJECT_SOURCE_DIR}/Test_commutation.cpp
#)

#target_link_libraries(YouBot_test YouBotDriver)

