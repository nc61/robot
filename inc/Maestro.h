#ifndef MAESTRO_H
#define MAESTRO_h

#define SERVO_BUCKET_PIN 0
#define SERVO_ARM_PIN    1
#define SENSOR_FSR_PIN   8
#define SENSOR_IR_L_PIN  9
#define SENSOR_IR_M_PIN 10 
#define SENSOR_IR_R_PIN 11 

#define SERVO_ARM_RAISE_POS 0x3E40
#define SERVO_ARM_LIFT_POS 3095
#define SERVO_ARM_BASE_POS 7900
#define SERVO_ARM_DUMP_POS 3336
#define SERVO_ARM_DIG_POS 8200
#define SERVO_ARM_SPEED 5
#define SERVO_ARM_SPEED_FAST 20

#define SERVO_BUCKET_RAISE_POS 6480
#define SERVO_BUCKET_LIFT_POS 6700
#define SERVO_BUCKET_BASE_POS 9324 
#define SERVO_BUCKET_DUMP_POS 3336
#define SERVO_BUCKET_SPEED 5
#define SERVO_BUCKET_SPEED_FAST 20
#define SERVO_BUCKET_DIG_POS 8300
#define SERVO_BUCKET_TILT_BACK_POS 10000


uint16_t readPin(uint8_t channel);
void servoCallback(const std_msgs::UInt8::ConstPtr &msg);
void liftDirt();
void servoInit();
void raiseBucket();
void serialInit();
void getError(uint8_t location);
void dumpDirt();
void digBucket();
void tiltBackBucket();
void lowerBucket();
void wait_until_position(uint8_t channel, uint16_t target);
void sendFeedback(uint8_t data);

#endif
