cmake_minimum_required(VERSION 2.8.3)
project(robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES IR.msg motor.msg)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

## Declare a cpp executable
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(SerialCom ~/catkin_ws/src/robot/src/SerialCom.cpp)
target_link_libraries(SerialCom /usr/local/lib/libserial.so)

add_executable(Maestro src/Maestro.cpp)
target_link_libraries(Maestro ${catkin_LIBRARIES} SerialCom)

add_executable(Xmega src/Xmega.cpp)
target_link_libraries(Xmega ${catkin_LIBRARIES} SerialCom)

add_executable(XmegaTest src/XmegaTest.cpp)
target_link_libraries(XmegaTest ${catkin_LIBRARIES})

add_executable(SerialTest src/SerialTest.cpp)
target_link_libraries(SerialTest ${catkin_LIBRARIES} SerialCom)
