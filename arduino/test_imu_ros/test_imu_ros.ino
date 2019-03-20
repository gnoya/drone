#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>

int hertz = 200;
int baudrate = 115200;

int min_throttle_read = 986;
int max_throttle_read = 1684;
int min_yaw_read = 986;
int max_yaw_read = 1684;
int min_pitch_read = 986;
int max_pitch_read = 1684;
int min_roll_read = 986;
int max_roll_read = 1684;

int throttle_pin = 14;
int yaw_pin = 15;
int roll_pin = 16;
int pitch_pin = 17;

volatile uint16_t rc_throttle;
volatile uint16_t rc_yaw;
volatile uint16_t rc_pitch;
volatile uint16_t rc_roll;

uint16_t rc_throttle_timer;
uint16_t rc_yaw_timer;
uint16_t rc_pitch_timer;
uint16_t rc_roll_timer;

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];
float yaw, pitch, roll = 0;
unsigned long timer = 0;
unsigned long timer_2 = 0;
float timeStep = 1 / (float)hertz;

geometry_msgs::Vector3 angles;
std_msgs::Int8 pwm;
ros::NodeHandle arduino_node;
ros::Publisher chatter("imu", &angles);
ros::Publisher pub_pwm("pwm", &pwm);

void throttle_handler(){
  if(digitalRead(throttle_pin) == HIGH){
    rc_throttle_timer = micros();
    pwm.data = 2;
    pub_pwm.publish(&pwm);
  }
  else {
    rc_throttle = (uint16_t)(micros() - rc_throttle_timer);
    if (rc_throttle < min_throttle_read) rc_throttle = min_throttle_read;
    else if (rc_throttle > max_throttle_read) rc_throttle = max_throttle_read;
    rc_throttle = (uint16_t)map(rc_throttle, min_throttle_read, max_throttle_read, 1000, 2000);
  }
  
}

void yaw_handler(){
  if(digitalRead(yaw_pin) == HIGH){
    rc_yaw_timer = micros();
  }
  else {
    rc_yaw = (uint16_t)(micros() - rc_yaw_timer);
    if (rc_yaw < min_yaw_read) rc_yaw = min_yaw_read;
    else if (rc_yaw > max_yaw_read) rc_yaw = max_yaw_read;
    rc_yaw = (uint16_t)map(rc_yaw, min_yaw_read, max_yaw_read, 1000, 2000);
  }
}

void pitch_handler(){
  if(digitalRead(pitch_pin) == HIGH){
    rc_pitch_timer = micros();
  }
  else {
    rc_pitch = (uint16_t)(micros() - rc_pitch_timer);
    if (rc_pitch < min_pitch_read) rc_pitch = min_pitch_read;
    else if (rc_pitch > max_pitch_read) rc_pitch = max_pitch_read;
    rc_pitch = (uint16_t)map(rc_pitch, min_pitch_read, max_pitch_read, 1000, 2000);
  }
}

void roll_handler(){
  if(digitalRead(roll_pin) == HIGH){
    rc_roll_timer = micros();
  }
  else {
    rc_roll = (uint16_t)(micros() - rc_roll_timer);
    if (rc_roll < min_roll_read) rc_roll = min_roll_read;
    else if (rc_roll > max_roll_read) rc_roll = max_roll_read;
    rc_roll = (uint16_t)map(rc_roll, min_roll_read, max_roll_read, 1000, 2000);
  }
}

void setup() {
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(439);
  mpu.setYAccelOffset(429);
  mpu.setZAccelOffset(902);
  mpu.setXGyroOffset(-11);
  mpu.setYGyroOffset(72);
  mpu.setZGyroOffset(48);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();

  arduino_node.getHardware()->setBaud(baudrate);
  arduino_node.initNode();
  arduino_node.advertise(chatter);
  arduino_node.advertise(pub_pwm);

  pinMode(throttle_pin, INPUT);
  pinMode(yaw_pin, INPUT);
  pinMode(roll_pin, INPUT);
  pinMode(pitch_pin, INPUT);
  attachInterrupt(throttle_pin, throttle_handler, CHANGE);
  attachInterrupt(yaw_pin, yaw_handler, CHANGE);
  attachInterrupt(roll_pin, roll_handler, CHANGE);
  attachInterrupt(pitch_pin, pitch_handler, CHANGE);
}

void loop() {
  timer = millis();
  if (fifoCount < packetSize) {
    angles.x = pitch;
    angles.y = roll;
    angles.z = yaw;
    chatter.publish(&angles);
    arduino_node.spinOnce();
  }
  else {
    if (fifoCount == 1024) {
      mpu.resetFIFO();
    }
    else{
      if (fifoCount % packetSize != 0) {
        mpu.resetFIFO();
      } 
      else {
        while (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
        }
      
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180/PI;
        pitch = ypr[1] * 180/PI;
        roll = ypr[2] * 180/PI;
      }
    }
  }

  fifoCount = mpu.getFIFOCount();
  
  if((timeStep * 1000) - (millis() - timer) > 0){
    delay((timeStep * 1000) - (millis() - timer));
  }

}
