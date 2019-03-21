#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include <ros.h>
#include <drone_control/imu_values.h>
#include <drone_control/rc_controller.h>
#define YAW 0
#define PITCH 1
#define ROLL 2
#define THROTTLE 3

int hertz = 100;
int baudrate = 115200;

int min_read_rc[4] = {996, 1048, 982, 983};
int max_read_rc[4] = {1837, 1952, 1824, 1837};
int rc_pins[4] = {14, 15, 16, 17};

int esc_pins[4] = {8, 9, 10, 11};

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

float angles[3] = {0};
float setpoints[4];
float errors[3];
float error_sum[3] = {0, 0, 0};
float previous_error[3] = {0, 0, 0};
float last_yaw = 180;
float yaw_velocity = 0;

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

drone_control::imu_values ros_angles;
drone_control::imu_values imu_errors;

drone_control::rc_controller rc_setpoints;
drone_control::rc_controller pid_values;

ros::NodeHandle arduino_node;
ros::Publisher chatter("imu", &ros_angles);
ros::Publisher pub_setpoints("rc_setpoints", &rc_setpoints);

ros::Publisher pub_errors("imu_errors", &imu_errors);
ros::Publisher pub_pid("pid_values", &pid_values);

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
  arduino_node.advertise(pub_setpoints);
  arduino_node.advertise(pub_pid); 
  
  arduino_node.advertise(pub_errors);

  pinMode(rc_pins[YAW], INPUT);
  pinMode(rc_pins[PITCH], INPUT);
  pinMode(rc_pins[ROLL], INPUT);
  pinMode(rc_pins[THROTTLE], INPUT);

  attachInterrupt(rc_pins[YAW], yaw_handler, CHANGE);
  attachInterrupt(rc_pins[PITCH], pitch_handler, CHANGE);
  attachInterrupt(rc_pins[ROLL], roll_handler, CHANGE);
  attachInterrupt(rc_pins[THROTTLE], throttle_handler, CHANGE);

  first_esc.attach(esc_pins[0]);
  second_esc.attach(esc_pins[1]);
  third_esc.attach(esc_pins[2]);
  fourth_esc.attach(esc_pins[3]);
  
  first_esc.writeMicroseconds(1000);
  second_esc.writeMicroseconds(1000);
  third_esc.writeMicroseconds(1000);
  fourth_esc.writeMicroseconds(1000);


  while (!arduino_node.connected())
  {
    arduino_node.spinOnce();
  }

  if (! arduino_node.getParam("kp", kp, 3)){
    //default values
    kp[YAW] = 0;
    kp[PITCH] = 0;
    kp[ROLL] = 0; 
  }

  if (! arduino_node.getParam("kd", kd, 3)){
    //default values
    kd[YAW] = 0;
    kd[PITCH] = 0;
    kd[ROLL] = 0; 
  }

  if (! arduino_node.getParam("ki", ki, 3)){
    //default values
    ki[YAW] = 0;
    ki[PITCH] = 0;
    ki[ROLL] = 0; 
  }
}

void loop()
{
  timer = millis();

  if(measure_mpu());
  {
    calculate_setpoints();
    calculate_errors();
    pid_controller();
    apply_motors();
    ros_publish();
  }
  
  rc_setpoints.set_throttle = rc_values[THROTTLE];
  rc_setpoints.set_yaw = rc_values[YAW];
  rc_setpoints.set_pitch = rc_values[PITCH];
  rc_setpoints.set_roll = rc_values[ROLL];


  pid_values.set_yaw = (float)pulse_length_esc1;
  pid_values.set_pitch = (float)pulse_length_esc2;
  pid_values.set_roll = (float)pulse_length_esc3;
  pid_values.set_throttle = (float)pulse_length_esc4;

  pub_setpoints.publish(&rc_setpoints);
  pub_pid.publish(&pid_values);
  
  
  arduino_node.spinOnce();
  
  check_delay();
}

bool measure_mpu()
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
        
        float new_yaw = angles[YAW] * 180 / PI + 180;
        
        angles[PITCH] = angles[PITCH] * 180 / PI;
        angles[ROLL] = angles[ROLL] * 180 / PI;
        angles[YAW] = new_yaw;
        yaw_velocity =  1000 * (new_yaw - last_yaw) / (millis() - timer);
        last_yaw = new_yaw;
        return (true);
      }
    }
  }
  return (false);
}

void calculate_setpoints()
{
  setpoints[YAW] = map(rc_values[YAW], 1000, 2000, 90, -90);
  setpoints[PITCH] = map(rc_values[PITCH], 1000, 2000, 33, -33);
  setpoints[ROLL] = map(rc_values[ROLL], 1000, 2000, 33, -33);
  setpoints[THROTTLE] = map(rc_values[THROTTLE], 1000, 2000, 1000, 1800);
}

void calculate_errors()
{
  errors[YAW] = setpoints[YAW] - yaw_velocity;
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
  if (setpoints[THROTTLE] >= 1030)
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

    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 1800);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 1800);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 1800);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 1800);
  }
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
  ros_angles.yaw = yaw_velocity;
  ros_angles.pitch = angles[PITCH];
  ros_angles.roll = angles[ROLL];

  imu_errors.yaw = errors[YAW];
  imu_errors.pitch = errors[PITCH];
  imu_errors.roll = errors[ROLL];
  
  pub_errors.publish(&imu_errors);
  chatter.publish(&ros_angles);
  
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
    uint16_t rc_value = rc_map(rc_timers[channel], min_read_rc[channel], max_read_rc[channel]);
    if(rc_value > 1420 && rc_value < 1580) rc_value = 1500;
    rc_values[channel] = rc_value;
  }
}

uint16_t rc_map(uint16_t timer, int min, int max)
{
  uint16_t value = (uint16_t)(micros() - timer);
  //return value;
  
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
