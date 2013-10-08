cmake_minimum_required(VERSION 2.8.3)
project(robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES IR.msg)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)



catkin_package(
	CATKIN_DEPENDS message_runtime
)

## Declare a cpp executable
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(SerialCom ~/catkin_ws/src/robot/src/SerialCom.cpp)


add_executable(Maestro src/Maestro.cpp)
target_link_libraries(Maestro ${catkin_LIBRARIES} /usr/local/lib/libserial.so SerialCom)

add_executable(SimpleServo src/SimpleServo.cpp)
target_link_libraries(SimpleServo ${catkin_LIBRARIES} /usr/local/lib/libserial.so) 

add_executable(Wander src/Wander.cpp)
target_link_libraries(Wander ${catkin_LIBRARIES})

add_executable(WanderTest src/WanderTest.cpp)
target_link_libraries(WanderTest ${catkin_LIBRARIES})

add_executable(XmegaTest src/XmegaTest.cpp)
target_link_libraries(XmegaTest ${catkin_LIBRARIES} /usr/local/lib/libserial.so SerialCom)
