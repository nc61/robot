cmake_minimum_required(VERSION 2.8.3)
project(robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES IR.msg motor.msg pile.msg)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} /home/nick/catkin_ws/src/robot/inc/)

#Main programs
add_executable(Maestro src/Maestro.cpp)
target_link_libraries(Maestro ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

add_executable(Xmega src/Xmega.cpp)
target_link_libraries(Xmega ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

add_executable(SenseReact src/SenseReact.cpp)
target_link_libraries(SenseReact ${catkin_LIBRARIES})

add_executable(RedDetect src/RedDetect.cpp)
target_link_libraries(RedDetect ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(VidSURF src/VidSURF.cpp)
target_link_libraries(VidSURF ${catkin_LIBRARIES} ${OpenCV_LIBS})


#Test programs
add_executable(SimpleServo test/SimpleServo.cpp)
target_link_libraries(SimpleServo ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

add_executable(CalibrateCam test/CalibrateCam.cpp)
target_link_libraries(CalibrateCam ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(XmegaTest test/XmegaTest.cpp)
target_link_libraries(XmegaTest ${catkin_LIBRARIES})

add_executable(TestMaestro test/TestMaestro.cpp)
target_link_libraries(TestMaestro ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

