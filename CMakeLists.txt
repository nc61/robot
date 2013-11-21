cmake_minimum_required(VERSION 2.8.3)
project(robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs
    message_generation)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES pile.msg sensors.msg motor.msg color.msg
    object.msg)

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



add_executable(Camera src/Camera.cpp)
target_link_libraries(Camera ${catkin_LIBRARIES} ${OpenCV_LIBS})



#Test programs
#add_executable(SimpleServo test/SimpleServo.cpp)
#target_link_libraries(SimpleServo ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

add_executable(VidSURF test/VidSURF.cpp)
target_link_libraries(VidSURF ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(CalibrateCam test/CalibrateCam.cpp)
target_link_libraries(CalibrateCam ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(XmegaTest test/XmegaTest.cpp)
#target_link_libraries(XmegaTest ${catkin_LIBRARIES})

#add_executable(TestMaestro test/TestMaestro.cpp)
#target_link_libraries(TestMaestro ${catkin_LIBRARIES} /usr/local/lib/libserial.so)

#add_executable(RedDetect test/RedDetect.cpp)
#target_link_libraries(RedDetect ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(FindColor test/FindColor.cpp)
#target_link_libraries(FindColor ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(SenseReact test/SenseReact.cpp)
target_link_libraries(SenseReact ${catkin_LIBRARIES})

add_executable(SURF test/SURF.cpp)
target_link_libraries(SURF ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(NavToColor test/NavToColor.cpp)
target_link_libraries(NavToColor ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(NavToObject test/NavToObject.cpp)
target_link_libraries(NavToObject ${catkin_LIBRARIES} ${OpenCV_LIBS})
