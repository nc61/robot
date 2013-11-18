#include <string>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "ros/ros.h"
#include "Maestro.h"
#include <SerialStream.h>
#include "Common.h"
#include "robot/sensors.h"
using namespace LibSerial;

/***************************************************************
 * Description: This node continously reads IR and FSR sensors at 10Hz while
 *      receiving and interpreting servo commands.
 **************************************************************/

SerialStream maestro;
ros::Publisher maestroFeedback;
ros::Publisher sensorPub;

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
    sensorPub = nh.advertise<robot::sensors>("sensor_data", 1);
    maestroFeedback = nh.advertise<std_msgs::UInt8>("maestro_feedback", 1);
    loop_rate.sleep(); 
    //Subscriber
    ros::Subscriber servoControl = nh.subscribe<std_msgs::UInt8>("servo_command", 1, servoCallback);
    
    //Initialize servos
    while(ros::ok())
    {
        robot::sensors msg;
        msg.leftIR = readPin(SENSOR_IR_L_PIN); 
        msg.midIR = readPin(SENSOR_IR_M_PIN);
        msg.rightIR = readPin(SENSOR_IR_R_PIN);
        msg.FSR = readPin(SENSOR_FSR_PIN);
        
//        ROS_INFO("Left: %d\tMid: %d\tRight: %d", msg.leftIR, msg.midIR, msg.rightIR);
//        ROS_INFO("FSR: %d/n", msg.FSR);
        sensorPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    servoInit();
}

uint16_t readPin(uint8_t channel)
{
    char command[] = {0x90, channel};
    maestro.write(command, sizeof(command));
   
    char response[2];
    maestro.read(response, 2);

    uint16_t num_low = (unsigned char)response[0];
    uint16_t num_high = (unsigned char)response[1];
    
    uint16_t num = num_low + 256*num_high;
    return num;
}

void liftDirt()
{
    char cmd_pos_arms[] =     {0x84, SERVO_ARM_PIN, 
                              (SERVO_ARM_LIFT_POS & 0x7F), 
                              ((SERVO_ARM_LIFT_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    
    ros::Rate delay500ms(2); 
    delay500ms.sleep();

    
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_LIFT_POS & 0x7F), 
                              ((SERVO_BUCKET_LIFT_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    
    ROS_INFO("Lifting dirt");
    wait_until_position(SERVO_BUCKET_PIN, SERVO_BUCKET_LIFT_POS);
    wait_until_position(SERVO_ARM_PIN, SERVO_ARM_LIFT_POS);
    sendFeedback(DONE_LIFTING);

}

void dumpDirt()
{
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_DUMP_POS & 0x7F), 
                              ((SERVO_BUCKET_DUMP_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    
    char cmd_pos_arms[] =   {0x84, SERVO_ARM_PIN, 
                              (SERVO_ARM_DUMP_POS & 0x7F), 
                              ((SERVO_ARM_DUMP_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    
    ROS_INFO("Dumping dirt");
    wait_until_position(SERVO_BUCKET_PIN, SERVO_BUCKET_DUMP_POS);
    wait_until_position(SERVO_ARM_PIN, SERVO_ARM_DUMP_POS);
    sendFeedback(DONE_DUMPING);
}

void lowerBucket()
{
    
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_BASE_POS & 0x7F), 
                              ((SERVO_BUCKET_BASE_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    
    char cmd_pos_arms[] =     {0x84, SERVO_ARM_PIN, 
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS >> 7) & 0x7F) };
    
    
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    
    ROS_INFO("Lowering bucket");
    wait_until_position(SERVO_ARM_PIN, SERVO_ARM_BASE_POS);
    sendFeedback(DONE_LOWERING);
}

void digBucket()
{
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_DIG_POS & 0x7F), 
                              ((SERVO_BUCKET_DIG_POS >> 7) & 0x7F) };
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));

    ROS_INFO("Digging into pile");
}

void tiltBackBucket()
{
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_TILT_BACK_POS & 0x7F), 
                              ((SERVO_BUCKET_TILT_BACK_POS >> 7) & 0x7F) };
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));

    ROS_INFO("Tilting dirt into bucket");
}

void servoInit()
{
    ROS_INFO("Initializing Servos");
    char cmd_pos_arms[] =     {0x84, SERVO_ARM_PIN, 
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS >> 7) & 0x7F) }; 
                              
    maestro.write(cmd_pos_arms, sizeof(cmd_pos_arms));
    getError(1);
    ROS_INFO("%d",SERVO_ARM_BASE_POS);
    
    char cmd_speed_arm[] =    {0x87, SERVO_ARM_PIN, 
                              (SERVO_ARM_SPEED_FAST & 0x7F), 
                              ((SERVO_ARM_SPEED_FAST >> 7) & 0x7F)
                              };
    
    maestro.write(cmd_speed_arm, sizeof(cmd_speed_arm));
    getError(2);
    
    
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_BASE_POS & 0x7F), 
                              ((SERVO_BUCKET_BASE_POS >> 7) & 0x7F) };
    
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    getError(3);
    
    char cmd_speed_bucket[] = {0x87, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_SPEED_FAST & 0x7F), 
                              ((SERVO_BUCKET_SPEED_FAST >> 7) & 0x7F) };
    
    maestro.write(cmd_speed_bucket, sizeof(cmd_speed_bucket));
    getError(4);
    wait_until_position(SERVO_BUCKET_PIN, SERVO_BUCKET_BASE_POS);
    wait_until_position(SERVO_ARM_PIN, SERVO_ARM_BASE_POS);
}

void wait_until_position(uint8_t channel, uint16_t target)
{
    char cmd[] = {0x90, channel};
    char response[2];
    uint16_t pulse_time = 0;
    ros::Rate loop_rate(LOOP_RATE);
    ROS_INFO("target: %d\tchannel: %d\n", target, channel);
    
    while(abs(pulse_time - target) > 5)
    {
        maestro.write(cmd, sizeof(cmd));
        maestro.read(response, sizeof(response));
        pulse_time = readPin(channel);
        ROS_INFO("Pulse time: %d\n", pulse_time);
        loop_rate.sleep();
    }
    ROS_INFO("Channel %d is at position %d\n", channel, pulse_time);
}

void sendFeedback(uint8_t data)
{
    std_msgs::UInt8 msg;
    msg.data = data;
    maestroFeedback.publish(msg);
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

    if (error_num[0] | error_num[1])
    {
        ROS_FATAL("Error code %x%x at location %d", error_num[0], error_num[1], location);
    }
}

/******************************************************
 * Callback functions
 ******************************************************/

void servoCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t command = msg->data;
    ROS_INFO("Received data %d", command);
    
    if (command == LIFT_DIRT){ liftDirt(); }
    else if (command == DUMP_DIRT){ dumpDirt(); }
    else if (command == LOWER_BUCKET){ lowerBucket(); }
    else if (command == SERVO_INIT){ servoInit(); }
    else if (command == DIG_BUCKET){ digBucket(); }
    else if (command == TILT_BACK_BUCKET){ tiltBackBucket(); } 

    else ROS_INFO("%d is not a valid command", command);

}
