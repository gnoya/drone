#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>

#define YAW 0
#define PITCH 1
#define ROLL 2
#define THROTTLE 3

int hertz = 200;
int baudrate = 115200;

int min_read_rc[4] = {986, 986, 986, 986};
int max_read_rc[4] = {1684, 1684, 1684, 1684};
int rc_pins[4] = {14, 15, 16, 17};

volatile uint16_t rc_values[4];
uint16_t rc_timers[4];

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float angles[3];
unsigned long timer = 0;
float timeStep = 1 / (float)hertz;

geometry_msgs::Vector3 ros_angles;
std_msgs::Int8 pwm;
ros::NodeHandle arduino_node;
ros::Publisher chatter("imu", &ros_angles);
ros::Publisher pub_pwm("pwm", &pwm);

void setup()
{
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

  pinMode(rc_pins[YAW], INPUT);
  pinMode(rc_pins[PITCH], INPUT);
  pinMode(rc_pins[ROLL], INPUT);
  pinMode(rc_pins[THROTTLE], INPUT);

  attachInterrupt(rc_pins[YAW], yaw_handler, CHANGE);
  attachInterrupt(rc_pins[PITCH], roll_handler, CHANGE);
  attachInterrupt(rc_pins[ROLL], pitch_handler, CHANGE);
  attachInterrupt(rc_pins[THROTTLE], throttle_handler, CHANGE);
}

void loop()
{
  timer = millis();
  check_mpu();
  ros_publish();

  check_delay();
}

void check_mpu()
{
  // este fifo count estaba casi al final del loop.
  fifoCount = mpu.getFIFOCount();

  if (fifoCount >= packetSize)
  {
    if (fifoCount == 1024)
    {
      mpu.resetFIFO();
    }
    else
    {
      if (fifoCount % packetSize != 0)
      {
        mpu.resetFIFO();
      }
      else
      {
        while (fifoCount >= packetSize)
        {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(angles, &q, &gravity);
        // Cuidado si esto altera los valores, antes no se sobreescribia a si mismo.
        angles[YAW] = angles[YAW] * 180 / PI;
        angles[PITCH] = angles[PITCH] * 180 / PI;
        angles[ROLL] = angles[ROLL] * 180 / PI;
      }
    }
  }
}

void ros_publish()
{
  ros_angles.x = angles[ROLL];
  ros_angles.y = angles[PITCH];
  ros_angles.z = angles[YAW];
  chatter.publish(&ros_angles);
  arduino_node.spinOnce();
}

void check_delay()
{
  unsigned long time_left = 1000 * timeStep - (millis() - timer);
  if (time_left > 0)
  {
    delay(time_left);
  }
}

void yaw_handler()
{
  rc_timing(YAW);
}

void pitch_handler()
{
  rc_timing(PITCH);
}

void roll_handler()
{
  rc_timing(ROLL);
}

void throttle_handler()
{
  rc_timing(THROTTLE);
}

void rc_timing(int channel)
{
  if (digitalRead(rc_pins[channel]) == HIGH)
  {
    rc_timers[channel] = micros();
  }
  else
  {
    rc_values[channel] = rc_map(rc_timers[channel], min_read_rc[channel], max_read_rc[channel]);
  }
}

uint16_t rc_map(uint16_t timer, int min, int max)
{
  uint16_t value = (uint16_t)(micros() - timer);
  if (value < min)
    value = min;
  else if (value > max)
    value = max;
  return (uint16_t)map(value, min, max, 1000, 2000);
}