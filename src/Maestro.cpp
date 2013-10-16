#include <string>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "robot/IR.h"
#include "ros/ros.h"
#include "Maestro.h"
#include <SerialStream.h>
#include "Common.h"
using namespace LibSerial;

/***************************************************************
 * Description: This node continously reads IR sensors at 10Hz while
 *      receiving and interpreting servo commands.
 *************************"*************************************/

SerialStream maestro;

int main(int argc, char** argv)
{
    //Initialize serial port
    std::string str = "/dev/ttyACM0";
    maestro.Open(str);
    maestro.SetBaudRate(SerialStreamBuf::BAUD_57600);
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    maestro.SetNumOfStopBits(1);
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!maestro.good())
    {
        std::cerr << "Error opening serial port"
                  << str
                  << std::endl;
        exit(1);
    }

    getError(0);
    
    ros::init(argc, argv, "Maestro");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    
    //Publisher
    ros::Publisher infraredSensor = nh.advertise<robot::IR>("sensor_data", 1000);
    loop_rate.sleep();
    //Subscriber
    ros::Subscriber servoControl = nh.subscribe<std_msgs::UInt8>("servo_command", 1000, servoCallback);
    
     //Initialize servos
    servoInit();
    
    while(ros::ok())
    {
        robot::IR msg;
        msg.leftIR = readPin(SENSOR_IR_L_PIN); 
        msg.midIR = readPin(SENSOR_IR_M_PIN);
        msg.rightIR = readPin(SENSOR_IR_R_PIN);
        ROS_INFO("Left IR: %d\tMiddle IR: %d\tRight IR: %d", msg.leftIR, msg.midIR, msg.rightIR);
        ROS_INFO("IR Reading: %d", msg.leftIR);
        infraredSensor.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

uint16_t readPin(uint8_t channel)
{
    char command[] = {0x90, channel};
    maestro.write(command, sizeof(command));
   
    char response[2];
    maestro.read(response, 2);

    uint16_t num_low = (uint16_t)(response[0]);
    uint16_t num_high = (uint16_t)(response[1]);
    
    ROS_INFO("num_low: %x\tnum_high: %x", num_low, num_high);
    return num_low + num_high*256;
}

void liftDirt()
{
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_LIFT_POS & 0x7F), 
                              ((SERVO_BUCKET_LIFT_POS << 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    ROS_INFO("Lifting dirt");
}

void raiseBucket()
{
    char cmd_pos_arms[] =     {0x9F, 2, SERVO_ARM_L_PIN, 
                              (SERVO_ARM_RAISE_POS & 0x7F), 
                              ((SERVO_ARM_RAISE_POS << 7) & 0x7F), 
                              (SERVO_ARM_RAISE_POS & 0x7F), 
                              ((SERVO_ARM_RAISE_POS << 7) & 0x7F), 
                              };
    
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    ROS_INFO("Raising bucket");
    
}

void servoInit()
{
    char cmd_speed_larm[] =   {0x87, SERVO_ARM_L_PIN, 
                              (SERVO_ARM_SPEED & 0x7F), 
                              ((SERVO_ARM_SPEED << 7) & 0x7F)
                              };
    
    maestro.write(cmd_speed_larm, sizeof(cmd_speed_larm));
    
    char cmd_speed_rarm[] =   {0x87, SERVO_ARM_R_PIN, 
                              (SERVO_ARM_SPEED & 0x7F), 
                              ((SERVO_ARM_SPEED << 7) & 0x7F)
                              };
    
    maestro.write(cmd_speed_rarm, sizeof(cmd_speed_rarm));
    
    char cmd_pos_arms[] =     {0x9F, 2, SERVO_ARM_L_PIN,
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS << 7) & 0x7F), 
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS << 7) & 0x7F), 
                              };
    
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    
    char cmd_speed_bucket[] = {0x87, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_SPEED & 0x7F), 
                              ((SERVO_BUCKET_SPEED << 7) & 0x7F) };
    
    maestro.write(cmd_speed_bucket, sizeof(cmd_speed_bucket));
    
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_BASE_POS & 0x7F), 
                              ((SERVO_BUCKET_BASE_POS << 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));

}

void getError(uint8_t location)
{
    char cmd_error[] = {0xA1};
    maestro.write(cmd_error, sizeof(cmd_error));

    char error_char[2];
    maestro.read(error_char, 2);

    uint8_t error_num[2];
    error_num[0] = error_char[0];
    error_num[1] = error_char[1];

    ROS_FATAL("Error code %x%x at location %d", error_num[0], error_num[1], location);
}

/******************************************************
 * Callback functions
 ******************************************************/

void servoCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t command = msg->data;
    ROS_INFO("Received data %d", command);
    
    if (command == 0){liftDirt();}
    else if (command == 1){raiseBucket();}
    else ROS_INFO("%d is not a valid command", command);
}
