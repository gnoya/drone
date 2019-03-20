#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
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

int esc_pins[4] = {3, 4, 5, 6};

float kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll

/*

  Front
  (1) (2)     x
    \ /     z ↑
    X       \|
    / \       +----→ y
  (3) (4)

  Motor 1 : front left - clockwise
  Motor 2 : front right - counter-clockwise
  Motor 3 : rear left - clockwise
  Motor 4 : rear left - counter-clockwise

  */

volatile uint16_t rc_values[4];
uint16_t rc_timers[4];

float angles[3];
float setpoints[4];
float errors[3];
float error_sum[3] = {0, 0, 0};
float previous_error[3] = {0, 0, 0};

unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

unsigned long timer = 0;
float timeStep = 1 / (float)hertz;

geometry_msgs::Vector3 ros_angles;
std_msgs::Int8 pwm;
ros::NodeHandle arduino_node;
ros::Publisher chatter("imu", &ros_angles);
ros::Publisher pub_pwm("pwm", &pwm);

Servo first_esc, second_esc, third_esc, fourth_esc;

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

  first_esc.attach(esc_pins[0]);
  second_esc.attach(esc_pins[1]);
  third_esc.attach(esc_pins[2]);
  fourth_esc.attach(esc_pins[3]);

  while (!arduino_node.connected())
  {
    arduino_node.spinOnce();
  }

  if (! nh.getParam("kp", kp, 3){
    //default values
    kp[YAW] = 0;
    kp[PITCH] = 0;
    kp[ROLL] = 0; 
  }

  if (! nh.getParam("kd", kd, 3){
    //default values
    kd[YAW] = 0;
    kd[PITCH] = 0;
    kd[ROLL] = 0; 
  }

  if (! nh.getParam("ki", ki, 3){
    //default values
    ki[YAW] = 0;
    ki[PITCH] = 0;
    ki[ROLL] = 0; 
  }
}

void loop()
{
  timer = millis();

  measure_mpu();
  /*
  calculate_setpoints();
  calculate_errors();
  pid_controller();
  apply_motors();
  */

  ros_publish();
  check_delay();
}

void measure_mpu()
{
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

        angles[YAW] = angles[YAW] * 180 / PI;
        angles[PITCH] = angles[PITCH] * 180 / PI;
        angles[ROLL] = angles[ROLL] * 180 / PI;
      }
    }
  }
}

void calculate_setpoints()
{
  setpoints[YAW] = map(rc_values[YAW], 1000, 2000, -180, 180);
  setpoints[PITCH] = map(rc_values[PITCH], 1000, 2000, 33, -33);
  setpoints[ROLL] = map(rc_values[ROLL], 1000, 2000, -33, 33);
  setpoints[THROTTLE] = map(rc_values[THROTTLE], 1000, 2000, 1000, 1800);
}

void calculate_errors()
{
  errors[YAW] = setpoints[YAW] - angles[YAW];
  errors[PITCH] = setpoints[PITCH] - angles[PITCH];
  errors[ROLL] = setpoints[ROLL] - angles[ROLL];
}

void pid_controller()
{
  float delta_err[3] = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
  float yaw_pid = 0;
  float pitch_pid = 0;
  float roll_pid = 0;

  // Initialize motor commands with throttle
  pulse_length_esc1 = setpoints[THROTTLE];
  pulse_length_esc2 = setpoints[THROTTLE];
  pulse_length_esc3 = setpoints[THROTTLE];
  pulse_length_esc4 = setpoints[THROTTLE];

  // Do not calculate anything if throttle is 0
  if (setpoints[THROTTLE] >= 1012)
  {
    // Calculate sum of errors : Integral coefficients
    error_sum[YAW] += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL] += errors[ROLL];

    // Calculate error delta : Derivative coefficients
    delta_err[YAW] = errors[YAW] - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL] = errors[ROLL] - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW] = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL] = errors[ROLL];

    // PID = e.Kp + ∫e.Ki + Δe.Kd
    yaw_pid = (errors[YAW] * kp[YAW]) + (error_sum[YAW] * ki[YAW]) + (delta_err[YAW] * kd[YAW]);
    pitch_pid = (errors[PITCH] * kp[PITCH]) + (error_sum[PITCH] * ki[PITCH]) + (delta_err[PITCH] * kd[PITCH]);
    roll_pid = (errors[ROLL] * kp[ROLL]) + (error_sum[ROLL] * ki[ROLL]) + (delta_err[ROLL] * kd[ROLL]);

    // Calculate pulse duration for each ESC
    pulse_length_esc1 = setpoints[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
    pulse_length_esc2 = setpoints[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
    pulse_length_esc3 = setpoints[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
    pulse_length_esc4 = setpoints[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
  }

  pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
  pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
  pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
  pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

void apply_motors()
{
  first_esc.writeMicroseconds(pulse_length_esc1);
  second_esc.writeMicroseconds(pulse_length_esc2);
  third_esc.writeMicroseconds(pulse_length_esc3);
  fourth_esc.writeMicroseconds(pulse_length_esc4);
}

void ros_publish()
{
  /*
  ros_angles.x = angles[ROLL];
  ros_angles.y = angles[PITCH];
  ros_angles.z = angles[YAW];
  */
  ros_angles.x = kp[YAW];
  ros_angles.y = ki[YAW];
  ros_angles.z = kd[YAW];
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

float minMax(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    value = max_value;
  }
  else if (value < min_value)
  {
    value = min_value;
  }

  return value;
}

void reset_pid()
{
  errors[YAW] = 0;
  errors[PITCH] = 0;
  errors[ROLL] = 0;

  error_sum[YAW] = 0;
  error_sum[PITCH] = 0;
  error_sum[ROLL] = 0;

  previous_error[YAW] = 0;
  previous_error[PITCH] = 0;
  previous_error[ROLL] = 0;
}

void stop_all()
{
  pulse_length_esc1 = 1000;
  pulse_length_esc2 = 1000;
  pulse_length_esc3 = 1000;
  pulse_length_esc4 = 1000;
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