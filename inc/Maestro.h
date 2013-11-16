#ifndef MAESTRO_H
#define MAESTRO_h

#define SENSOR_IR_L_PIN  3
#define SENSOR_IR_M_PIN  2
#define SENSOR_IR_R_PIN  4
#define SERVO_ARM_PIN    1
#define SERVO_BUCKET_PIN 0
#define SENSOR_FSR_PIN   5

#define SERVO_ARM_RAISE_POS 0x3E40
#define SERVO_ARM_LIFT_POS 3095
#define SERVO_ARM_BASE_POS 8020
#define SERVO_ARM_DUMP_POS 3336
#define SERVO_ARM_SPEED 5
#define SERVO_ARM_SPEED_FAST 20

#define SERVO_BUCKET_RAISE_POS 6480
#define SERVO_BUCKET_LIFT_POS 5848
#define SERVO_BUCKET_BASE_POS 9324 
#define SERVO_BUCKET_DUMP_POS 3336
#define SERVO_BUCKET_SPEED 5
#define SERVO_BUCKET_SPEED_FAST 20


uint16_t readPin(uint8_t channel);
void servoCallback(const std_msgs::UInt8::ConstPtr &msg);
void liftDirt();
void servoInit();
void raiseBucket();
void serialInit();
void getError(uint8_t location);
void dumpDirt();
void lowerBucket();
void wait_until_position(uint8_t channel, uint16_t target);
void sendFeedback(uint8_t data);

#endif
