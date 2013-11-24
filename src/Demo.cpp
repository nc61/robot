#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/object.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "SenseReact.h"
#include "Demo.h"
#include "Common.h"

//IR information and states
uint8_t state = FIND_PILE;
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
uint8_t last_turn = RIGHT;
uint8_t FSR;
uint8_t xmega_feedback;
float xpos_pile, area_pile;
float xpos_bin, area_bin, y_0, y_1, y_2, y_3, y_avg_bin;

//Establish global scope for publisher
ros::Publisher motorPub;
ros::Publisher servoPub;
ros::Publisher statePub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Wander");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Rate init_delay(INIT_DELAY);
    //Publishers
    motorPub = nh.advertise<robot::motor>("motor_command", 1000);
    servoPub = nh.advertise<std_msgs::UInt8>("maestro_command", 1);
    statePub = nh.advertise<std_msgs::UInt8>("state_data", 1);
    init_delay.sleep();
    //Subscribers
    ros::Subscriber sensorSub = nh.subscribe<robot::sensors>("sensor_data", 2, processSensors);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);
    ros::Subscriber colorSub = nh.subscribe<robot::color>("color_data", 1, processColor);
    ros::Subscriber objectRecSub = nh.subscribe<robot::object>("object_data", 1, processObject);
    init_delay.sleep();

    while(ros::ok())
    {
        std_msgs::UInt8 msg;
        msg.data = state;
        statePub.publish(msg);

        if (state == FIND_PILE)
        {
            ROS_INFO("Finding pile");
            findPile();
        } else if (state == NAV_TO_PILE) {
            ROS_INFO("Navigating to pile");
            navToPile();
            state = APPROACH_PILE;
        } else if (state == APPROACH_PILE) {
            ROS_INFO("Approaching pile");
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            sendServoCommand(DIG_BUCKET);
            delayms(1000);
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (FSR < FSR_THRESHOLD){ ros::spinOnce(); loop_rate.sleep(); }
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            sendServoCommand(TILT_BACK_BUCKET);
            delayms(2000);
            sendServoCommand(LIFT_DIRT);
            delayms(2000);
            sendMotorCommand(GO_BACKWARD, NORMAL_SPEED);
            delayms(3000);
            stop();
            state = FIND_BIN;
        } else if (state == FIND_BIN) {
            ROS_INFO("Finding bin");
            findBin();
            state = NAV_TO_BIN;
        } else if (state == NAV_TO_BIN) {
            ROS_INFO("Navigating to bin");
            navToBin();
            delayms(3000);
            sendServoCommand(DUMP_DIRT);
            delayms(3000);
            smallMovement(BACKWARD, 1000, NO_AVOID);
            sendServoCommand(SERVO_INIT);
            smallMovement(LEFT, 1000, NO_AVOID);
            state = FIND_PILE;
        }

        loop_rate.sleep();
    }
}
void navToPile()
{
    sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
    while (area_pile < AREA_PILE_CLOSE && ros::ok())
    {
        ros::spinOnce();
        avoid_obstacle();
        if (xpos_pile < XPOS_PILE_LEFT_LIMIT || xpos_pile > XPOS_PILE_RIGHT_LIMIT || area_pile < AREA_PILE_THRESH)
        {
            ROS_INFO("Readjusting trajectory to pile");
            findPile();
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
        }
        ROS_INFO("Heading toward pile");
        delayms(75);
    }
}

void findPile()
{
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok())
    {
        if (area_pile > AREA_PILE_LOW)
        {
            if (xpos_pile < XPOS_PILE_CENTERED)
            {
                //"almost" centered
                if (xpos_pile > XPOS_PILE_LEFT_LIMIT)
                {
                    ROS_INFO("Pile close to aligned");
                    sendMotorCommand(PIVOT_LEFT, SLOW_PIVOT_TO_PILE);
                    while (xpos_pile < (XPOS_PILE_CENTERED) || area_pile < AREA_PILE_THRESH) { ros::spinOnce(); }
                    sendMotorCommand(STOP_IMMEDIATELY, 0);
                    state = NAV_TO_PILE;
                    return;
                } else {
                    ROS_INFO("Pivoting left looking for pile");
                    sendMotorCommand(PIVOT_LEFT, NORMAL_SPEED);
                }
            } else if (xpos_pile > XPOS_PILE_CENTERED) {
                //"almost" centered
                if (xpos_pile < XPOS_PILE_RIGHT_LIMIT)
                {
                    ROS_INFO("Pile close to aligned");
                    sendMotorCommand(PIVOT_LEFT, SLOW_PIVOT_TO_PILE);
                    while (xpos_pile > (XPOS_PILE_CENTERED) || area_pile < AREA_PILE_THRESH) { ros::spinOnce(); }
                    sendMotorCommand(STOP_IMMEDIATELY, 0);
                    state = NAV_TO_PILE;
                    return;
                } else {
                    ROS_INFO("Pivoting right looking for pile");
                    sendMotorCommand(PIVOT_RIGHT, NORMAL_SPEED);
                }
            } else {
                sendMotorCommand(STOP_IMMEDIATELY, 0);
                state = NAV_TO_PILE;
                return;
            }
        } else if (last_turn == RIGHT) {
            sendMotorCommand(PIVOT_LEFT, FAST);
            ROS_INFO("No sight of pile. pivoting left.");
        } else if (last_turn == LEFT) {
            sendMotorCommand(PIVOT_RIGHT, FAST);
            ROS_INFO("No sight of pile. Pivoting right.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void navToBin()
{
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok())
    {
        while (y_avg_bin < YPOS_BIN_CLOSE) 
        {
            ROS_INFO("Moving toward bin");
            smallMovement(FORWARD, 500, AVOID);
            for (int i = 0; i < NTB_FWD_CYCLES; i++) 
            {
                ros::spinOnce();
                ROS_INFO("y_avg: %f", y_avg_bin); 
                if (y_avg_bin >= YPOS_BIN_CLOSE)
                    break;
                loop_rate.sleep();
            }
        }
        sendMotorCommand(STOP_IMMEDIATELY, 0);
        if (y_3 - y_2 > ORIENTATION_THRESHOLD)
        {
            ROS_INFO("y3 > y2");
            while (xpos_bin > XPOS_BIN_ALIGN_RIGHT) { 
                smallMovement(RIGHT, 200, NO_AVOID);
                for (int i = 0; i < NTB_FWD_CYCLES; i++)
                {
                    ros::spinOnce();
                    ROS_INFO("xpos_bin: %f", xpos_bin);
                    if (xpos_bin <= XPOS_BIN_ALIGN_RIGHT)
                        break;
                    loop_rate.sleep();
                }
            }
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (leftIR < IR_BIN_LEFT) { ros::spinOnce(); }
            sendMotorCommand(PIVOT_LEFT, NORMAL_SPEED);
            while (midIR < IR_BIN_MID) { ros::spinOnce(); }
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (midIR < IR_BIN_DUMP);
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            return;
        } else if (y_2 - y_3 > ORIENTATION_THRESHOLD) {
            ROS_INFO("y2 > y3");
            while (xpos_bin < XPOS_BIN_ALIGN_LEFT) { 
                smallMovement(LEFT, 200, NO_AVOID);
                for (int i = 0; i < NTB_FWD_CYCLES; i++)
                {
                    ros::spinOnce();
                    ROS_INFO("xpos_bin: %f", xpos_bin);
                    if (xpos_bin >= XPOS_BIN_ALIGN_LEFT)
                        break;
                    loop_rate.sleep();
                }
            }
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (rightIR < IR_BIN_RIGHT) { ros::spinOnce(); }
            sendMotorCommand(PIVOT_RIGHT, NORMAL_SPEED);
            while (midIR < IR_BIN_MID) { ros::spinOnce(); }
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (midIR < IR_BIN_DUMP) { ros::spinOnce(); }
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            return;
        } else {
            sendMotorCommand(GO_FORWARD, NORMAL_SPEED);
            while (midIR < IR_BIN_DUMP) { ros::spinOnce(); }
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            return;
        }
    }
}

//Description: Pivots until the bin is found then stops
//Calls: delayms(), sendMotorCommand();
void findBin()
{
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("Looking for bin");
        if (area_bin > AREA_BIN_THRESH)
        {
            if (xpos_bin < XPOS_BIN_LEFT_LIMIT)
            {
                ROS_INFO("Bin found to left");
                smallMovement(LEFT, 75, NO_AVOID);
                for (int i = 0; i < 5; i++)
                {
                    ros::spinOnce();
                    if (xpos_bin >= XPOS_PILE_CENTERED && area_bin > AREA_BIN_THRESH)
                    {
                        state = NAV_TO_BIN;
                        return;
                    }
                    loop_rate.sleep();
                }
            } else if (xpos_bin > XPOS_BIN_RIGHT_LIMIT) {
                ROS_INFO("Bin found to right");
                smallMovement(RIGHT, 75, NO_AVOID);
                for (int i = 0; i < 5; i++)
                {
                    ros::spinOnce();
                    if (xpos_bin <= XPOS_PILE_CENTERED && area_bin > AREA_BIN_THRESH)
                    {
                        state = NAV_TO_BIN;
                        return;
                    }
                    loop_rate.sleep();
                }
            }
        } else if (last_turn == LEFT) {
            ROS_INFO("Pivoting right toward bin");
            smallMovement(RIGHT, 750, NO_AVOID);
            for (int i = 0; i < 5; i++)
            {
                ros::spinOnce();
                if (area_bin >= AREA_BIN_THRESH)
                    break;
                loop_rate.sleep();
            }
        } else {
            ROS_INFO("Pivoting right toward bin");
            smallMovement(LEFT, 750, NO_AVOID);
            for (int i = 0; i < 5; i++) 
            {
                ros::spinOnce();
                if (area_bin >= AREA_BIN_THRESH)
                    break;
                loop_rate.sleep();
            }
        }
        loop_rate.sleep();
    }

}
//Description: avoids an obstacle
//Calls: stop(), SendMotorCommand()
int avoid_obstacle()
{
    ros::Rate loop_rate(LOOP_RATE);
    if (midIR > MID_VN)
    {
        stop();
        ROS_WARN("Object in front");
        sendMotorCommand(GO_BACKWARD, FAST);
        while(midIR > MID_FAR) { ros::spinOnce(); loop_rate.sleep(); }
        if (leftIR > rightIR) 
        {
            sendMotorCommand(PIVOT_RIGHT, FAST) ;
            last_turn = RIGHT;
        } else {
            sendMotorCommand(PIVOT_LEFT, FAST);
            last_turn = LEFT; 
        }
        while (midIR > MID_VF){ ros::spinOnce(); }
        if (state == NAV_TO_PILE || state == NAV_TO_BIN)
        {
            smallMovement(FORWARD, 750, AVOID);
            if (state == NAV_TO_BIN) {
                state = FIND_BIN;
            } else {
                state = FIND_PILE;
            }
        }
        return 1;
    } else if (leftIR > LEFT_VN) {
        ROS_WARN("Object to left");
        sendMotorCommand(PIVOT_RIGHT, NORMAL_SPEED);
        while (leftIR > LEFT_FAR || rightIR > RIGHT_FAR) { ros::spinOnce(); loop_rate.sleep(); }
        last_turn = RIGHT;
        if (state == NAV_TO_PILE || state == NAV_TO_BIN)
        {
            smallMovement(FORWARD, 750, AVOID);
            if (state == NAV_TO_BIN) {
                state = FIND_BIN;
            } else {
                state = FIND_PILE;
            }
        }
        return 1;
    } else if (rightIR > RIGHT_VN) {
        ROS_WARN("Object to right");
        sendMotorCommand(PIVOT_LEFT, FAST);
        while (rightIR > RIGHT_FAR || leftIR > LEFT_FAR) { ros::spinOnce(); loop_rate.sleep(); }
        last_turn = LEFT;
        if (state == NAV_TO_PILE || state == NAV_TO_BIN)
        {
            smallMovement(FORWARD, 750, AVOID);
            if (state == NAV_TO_BIN) {
                ROS_INFO("Back to looking for bin");
                state = FIND_BIN;
            } else {
                ROS_INFO("Back to looking for pile");
                state = FIND_PILE;
            }
        }
        return 1;
    }
    return 0;
}

//Description: Tells motors to stop, then
//     waits for signal that motors have stopped
void stop()
{
    robot::motor msg;
    msg.command = STOP_MOTORS;
    msg.duty_cycle = 0;
    motorPub.publish(msg);
    
    xmega_feedback = 0;
    while (xmega_feedback != 1){ ros::spinOnce(); }
}

void smallMovement(uint8_t direction, uint16_t interval, uint8_t avoid)
{
    if (direction == LEFT)
        sendMotorCommand(PIVOT_LEFT_IMM, NORMAL_SPEED);
    else if (direction == RIGHT)
        sendMotorCommand(PIVOT_RIGHT_IMM, NORMAL_SPEED);
    else if (direction == FORWARD)
        sendMotorCommand(GO_FORWARD_IMM, NORMAL_SPEED);
    else if (direction == BACKWARD)
        sendMotorCommand(GO_BACKWARD, NORMAL_SPEED);
    else ROS_FATAL("Invalid argument to smallMovement");
    delayms(interval, avoid);
    sendMotorCommand(STOP_IMMEDIATELY, 0);
}

//Description: Sends a command to the motor controller
void sendMotorCommand(uint8_t command, uint8_t duty_cycle)
{
    robot::motor msg;
    msg.command = command;
    msg.duty_cycle = duty_cycle;
    motorPub.publish(msg);
}

//Description: Sends a command to the mini Maestro servo controller
void sendServoCommand(uint8_t command)
{
    std_msgs::UInt8 msg;
    msg.data = command;
    servoPub.publish(msg);
}

void delayms(uint16_t ms)
{
    ros::Rate millisecond(1000);
    for (int i = 0; i < ms; i++) {millisecond.sleep(); }
}

void delayms(uint16_t ms, uint8_t avoid)
{
    ros::Rate millisecond(1000);
    if (avoid)
    {
        for (int i = 0; i < ms; i++) 
        {
            if (avoid_obstacle() == 1) 
                break;
            millisecond.sleep(); 
        }
    } else for (int i = 0; i < ms; i++) { millisecond.sleep(); }
}

/**********************************************************
 * Callback functions
 **********************************************************/

//Description: updates global variables (sensor information)
//Called by: spinOnce()
void processSensors(const robot::sensors::ConstPtr &msg)
{
    leftIR = msg->leftIR;
    midIR = msg->midIR;
    rightIR = msg->rightIR;
    FSR = msg->FSR;
}

//Description: Updates global variable (feedback from xmega)
//Called by: spinOnce()
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg)
{
    xmega_feedback = msg->data;
}

//Description: Updates global variable (x position of pile)
//Called by: SpinOnce()
void processColor(const robot::color::ConstPtr &msg)
{
    xpos_pile = msg->xpos;
    area_pile = msg->area;
}

//Description: updates global variables (xpos_bin and area_bin)
//Called by: spinOnce()
void processObject(const robot::object::ConstPtr &msg)
{
    y_0 = msg->y0;
    y_1 = msg->y1;
    y_2 = msg->y2;
    y_3 = msg->y3;
    y_avg_bin = (y_2 + y_3)/2;

    xpos_bin = msg->xpos;
    area_bin = msg->area;
}
