#define SENSOR_IR_L_PIN  0
#define SENSOR_IR_M_PIN  1
#define SENSOR_IR_R_PIN  2
#define SERVO_ARM_L_PIN  3
#define SERVO_ARM_R_PIN  4
#define SERVO_BUCKET_PIN 5

#define SERVO_ARM_RAISE_POS 0x3E40
#define SERVO_ARM_LIFT_POS 0x3E40
#define SERVO_ARM_BASE_POS 0x2E70
#define SERVO_ARM_DUMP_POS 0x900
#define SERVO_ARM_SPEED 0x010C

#define SERVO_BUCKET_RAISE_POS 0x2E70
#define SERVO_BUCKET_LIFT_POS 0x3E40
#define SERVO_BUCKET_BASE_POS 0x2E70
#define SERVO_BUCKET_DUMP_POS 0x2E70
#define SERVO_BUCKET_SPEED 0x010C


uint16_t readPin(unsigned int channel);
void servoCallback(const std_msgs::UInt8::ConstPtr &msg);
void liftDirt();
void servoInit();
void raiseBucket();
void serialInit();
