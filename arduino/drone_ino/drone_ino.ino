#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <drone/Errors.h>
#include <drone/ImuMeasures.h>
#include <drone/Motors.h>
#include <drone/Setpoints.h>
#include <drone/RCValues.h>
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

float kp[3];    // P coefficients in that order : Yaw, Pitch, Roll
float ki[3]; // I coefficients in that order : Yaw, Pitch, Roll
float kd[3];          // D coefficients in that order : Yaw, Pitch, Roll

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
unsigned long yaw_timer = 0;
float timeStep = 1 / (float)hertz;
bool enable_motors = true;

drone::ImuMeasures imu_measures;
drone::Errors imu_errors;

drone::Setpoints ros_setpoints;
drone::RCValues ros_rc_values;
drone::Motors motor_value;

ros::NodeHandle arduino_node;
ros::Publisher pub_measures("imu_measures", &imu_measures);
ros::Publisher pub_errors("imu_errors", &imu_errors);
ros::Publisher pub_setpoints("setpoints", &ros_setpoints);
ros::Publisher pub_rc_values("rc_values", &ros_rc_values);
ros::Publisher pub_pid("motors", &motor_value);
void reboot_drone(const std_msgs::Empty& toggle_msg);
ros::Subscriber<std_msgs::Empty> sub_reboot("reboot", &reboot_drone);
void update_params(const std_msgs::Empty& toggle_msg);
ros::Subscriber<std_msgs::Empty> sub_params("update_params", &update_params);
void enable_motors_function(const std_msgs::Bool& msg);
ros::Subscriber<std_msgs::Bool> sub_motors("enable_motors", &enable_motors_function);

Servo first_esc, second_esc, third_esc, fourth_esc;

void setup()
{  
  arduino_node.getHardware()->setBaud(baudrate);
  arduino_node.initNode();
  
  arduino_node.advertise(pub_measures);
  arduino_node.advertise(pub_errors);
  arduino_node.advertise(pub_setpoints);
  arduino_node.advertise(pub_rc_values);
  arduino_node.advertise(pub_pid); 
  arduino_node.subscribe(sub_reboot);
  arduino_node.subscribe(sub_params);
  arduino_node.subscribe(sub_motors);
  
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
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
  
  //stop_all();

  while (!arduino_node.connected())
  {
    first_esc.writeMicroseconds(rc_values[THROTTLE]);
    second_esc.writeMicroseconds(rc_values[THROTTLE]);
    third_esc.writeMicroseconds(rc_values[THROTTLE]);
    fourth_esc.writeMicroseconds(rc_values[THROTTLE]);
    arduino_node.spinOnce();
  }

  get_ros_params();
  init_mpu();
  yaw_timer = millis();
}

void loop()
{
  timer = millis();

  if(measure_mpu());
  {
    calculate_setpoints();
    calculate_errors();
    pid_controller();
    if(enable_motors) {
      apply_motors();
    }
    imu_publish();
  }
  
  ros_publish();
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
        
        angles[PITCH] = - angles[PITCH] * 180 / PI;
        angles[ROLL] = angles[ROLL] * 180 / PI;
        angles[YAW] = new_yaw;
        yaw_velocity =  1000 * (new_yaw - last_yaw) / (millis() - yaw_timer);
        yaw_timer = millis();
        last_yaw = new_yaw;     
        return (true);
      }
    }
  }
  return (false);
}

void calculate_setpoints()
{
  if(rc_values[YAW] == 0) setpoints[YAW] = 0;
  else if(rc_values[YAW] == 1500) setpoints[YAW] = 0;
  else setpoints[YAW] = map(rc_values[YAW], 1000, 2000, 90, -90) - 1;
  
  if(rc_values[PITCH] == 0) setpoints[PITCH] = 0;
  else if(rc_values[PITCH] == 1500) setpoints[PITCH] = 0;
  else setpoints[PITCH] = map(rc_values[PITCH], 1000, 2000, 33, -33) - 1;
  
  if(rc_values[ROLL] == 0) setpoints[ROLL] = 0;
  else if(rc_values[ROLL] == 1500) setpoints[ROLL] = 0;
  else setpoints[ROLL] = map(rc_values[ROLL], 1000, 2000, 33, -33) - 1;
  
  if(rc_values[THROTTLE] <= 1000) setpoints[THROTTLE] = 1000;
  else setpoints[THROTTLE] = map(rc_values[THROTTLE], 1000, 2000, 1000, 1800);
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

void imu_publish()
{
  imu_measures.yaw = yaw_velocity;
  imu_measures.pitch = angles[PITCH];
  imu_measures.roll = angles[ROLL];

  imu_errors.yaw_error = errors[YAW];
  imu_errors.pitch_error = errors[PITCH];
  imu_errors.roll_error = errors[ROLL];
  
  pub_errors.publish(&imu_errors);
  pub_measures.publish(&imu_measures);
}

void ros_publish(){
  ros_rc_values.rc_throttle = (uint16_t)rc_values[THROTTLE];
  ros_rc_values.rc_yaw = (uint16_t)rc_values[YAW];
  ros_rc_values.rc_pitch = (uint16_t)rc_values[PITCH];
  ros_rc_values.rc_roll = (uint16_t)rc_values[ROLL];
  
  ros_setpoints.set_throttle = setpoints[THROTTLE];
  ros_setpoints.set_yaw = setpoints[YAW];
  ros_setpoints.set_pitch = setpoints[PITCH];
  ros_setpoints.set_roll = setpoints[ROLL];

  motor_value.front_left = (uint16_t)pulse_length_esc1;
  motor_value.front_right = (uint16_t)pulse_length_esc2;
  motor_value.back_left = (uint16_t)pulse_length_esc3;
  motor_value.back_right = (uint16_t)pulse_length_esc4;

  pub_rc_values.publish(&ros_rc_values);
  pub_setpoints.publish(&ros_setpoints);
  pub_pid.publish(&motor_value);
}

void check_delay()
{
  unsigned long time_left = 1000 * timeStep - (millis() - timer);
  if (time_left > 0)
  {
    delay(time_left);
  }
}

void init_mpu(){
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
}

void reboot_drone(const std_msgs::Empty& toggle_msg){
  digitalWrite(7, HIGH);
  reset_pid();
  get_ros_params();
  init_mpu();
  delay(500);
  digitalWrite(7, LOW);
}


void update_params(const std_msgs::Empty& toggle_msg){
  digitalWrite(7, HIGH);
  reset_pid();
  get_ros_params();
  delay(150);
  digitalWrite(7, LOW);
}

void enable_motors_function(const std_msgs::Bool& msg){
  stop_all();
  enable_motors = msg.data;
}

void get_ros_params(){
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
void rc_timing(int channel)
{
  if (digitalRead(rc_pins[channel]) == HIGH)
  {
    rc_timers[channel] = micros();
  }
  else
  {
    uint16_t rc_value = rc_map(rc_timers[channel], min_read_rc[channel], max_read_rc[channel]);
    if(rc_value > 1460 && rc_value < 1540) rc_value = 1500;
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
  first_esc.writeMicroseconds(1000);
  second_esc.writeMicroseconds(1000);
  third_esc.writeMicroseconds(1000);
  fourth_esc.writeMicroseconds(1000);
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
