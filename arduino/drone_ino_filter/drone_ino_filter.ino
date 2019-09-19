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

int hertz = 500;
int baudrate = 115200;
// Yaw, pitch, roll, throttle
int min_read_rc[4] = {1260, 1235, 1238, 984};
int max_read_rc[4] = {1700, 1670, 1662, 1630};
int rc_pins[4] = {14, 15, 16, 17};
int esc_pins[4] = {8, 9, 10, 11};

int max_yaw_setpoint = 10;
int max_pitch_setpoint = 12;
int max_roll_setpoint = 12;

int max_i_yaw = 150;
int max_i_pitch = 150;
int max_i_roll = 150;

// Complementary filter value
const float alpha = 0.96;

/*
    Front
    (1) (2)     y
        \ /     r ↑
        X       \|
        / \       +----→ p
    (3) (4)
    
    Front: Blue
    Motor 1 : front left - clockwise
    Motor 2 : front right - counter-clockwise
    Motor 3 : rear left - clockwise
    Motor 4 : rear left - counter-clockwise
*/

float kp[3]; // P coefficients in that order : Yaw, Pitch, Roll
float ki[3]; // I coefficients in that order : Yaw, Pitch, Roll
float kd[3]; // D coefficients in that order : Yaw, Pitch, Roll

volatile uint16_t rc_values[4];
uint16_t rc_timers[4];

float angles[3] = {0};
float setpoints[4];
float errors[3];
float error_sum[3] = {0, 0, 0};
float previous_error[3] = {0, 0, 0};

// FIX: this 180 could lead to issues ?
float last_yaw = 180;
float yaw_velocity = 0;

unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

MPU6050 mpu;

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
void reboot_drone(const std_msgs::Empty &toggle_msg);
ros::Subscriber<std_msgs::Empty> sub_reboot("reboot", &reboot_drone);
void update_params(const std_msgs::Empty &toggle_msg);
ros::Subscriber<std_msgs::Empty> sub_params("update_params", &update_params);
void enable_motors_function(const std_msgs::Bool &msg);
ros::Subscriber<std_msgs::Bool> sub_motors("enable_motors", &enable_motors_function);

Servo first_esc, second_esc, third_esc, fourth_esc;

// This global variable tells us how to scale gyroscope data
float GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float ACCEL_FACTOR;

const float RADIANS_TO_DEGREES = 57.2958; // 180 / 3.14159

unsigned long last_read_time;
float last_x_angle; // These are the filtered angles
float last_y_angle;
float last_z_angle;
float last_gyro_x_angle; // Store the gyro angles to compare drift
float last_gyro_y_angle;
float last_gyro_z_angle;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Use the following global variables
// to calibrate the gyroscope sensor and accelerometer readings
float base_x_gyro = 0;
float base_y_gyro = 0;
float base_z_gyro = 0;
float base_x_accel = 0;
float base_y_accel = 0;
float base_z_accel = 0;

inline unsigned long get_last_time() { return last_read_time; }
inline float get_last_x_angle() { return last_x_angle; }
inline float get_last_y_angle() { return last_y_angle; }
inline float get_last_z_angle() { return last_z_angle; }
inline float get_last_gyro_x_angle() { return last_gyro_x_angle; }
inline float get_last_gyro_y_angle() { return last_gyro_y_angle; }
inline float get_last_gyro_z_angle() { return last_gyro_z_angle; }

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro)
{
    last_read_time = time;
    last_x_angle = x;
    last_y_angle = y;
    last_z_angle = z;
    last_gyro_x_angle = x_gyro;
    last_gyro_y_angle = y_gyro;
    last_gyro_z_angle = z_gyro;
}

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

    if (measure_mpu())
    {
        calculate_setpoints();
        calculate_errors();
        pid_controller();

        if (enable_motors)
        {
            apply_motors();
        }

        imu_publish();
    }
    else
    {
        stop_all();
    }

    ros_publish();
    arduino_node.spinOnce();
    check_delay();
}

bool measure_mpu()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Remove offsets and scale gyro data
    float gyro_x = (gx - base_x_gyro) / GYRO_FACTOR;
    float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;
    float gyro_z = (gz - base_z_gyro) / GYRO_FACTOR;
    float accel_x = ax; // - base_x_accel;
    float accel_y = ay; // - base_y_accel;
    float accel_z = az; // - base_z_accel;

    float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_z = 0;

    unsigned long t_now = millis();
    // Compute the (filtered) gyro angles
    float dt = (t_now - get_last_time()) / 1000.0;
    float gyro_angle_x = gyro_x * dt + get_last_x_angle();
    float gyro_angle_y = gyro_y * dt + get_last_y_angle();
    float gyro_angle_z = gyro_z * dt + get_last_z_angle();

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    float angle_z = gyro_angle_z; // Accelerometer doesn't give z-angle

    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    // float new_yaw = angles[YAW] * 180 / PI + 180;
    // Inverted on purpose
    angles[PITCH] = angle_x;
    angles[ROLL] = angle_y;
    angles[YAW] = angle_z;
    yaw_velocity = 1000 * (angle_z - last_yaw) / (millis() - yaw_timer);
    yaw_timer = millis();
    last_yaw = angle_z;
}

void calculate_setpoints()
{
    if (rc_values[YAW] == 0)
        setpoints[YAW] = 0;
    else if (rc_values[YAW] == 1500)
        setpoints[YAW] = 0;
    else
        setpoints[YAW] = map(rc_values[YAW], 1000, 2000, max_yaw_setpoint, -max_yaw_setpoint);

    if (rc_values[PITCH] == 0)
        setpoints[PITCH] = 0;
    else if (rc_values[PITCH] == 1500)
        setpoints[PITCH] = 0;
    else
        setpoints[PITCH] = map(rc_values[PITCH], 1000, 2000, -max_pitch_setpoint, max_pitch_setpoint);

    if (rc_values[ROLL] == 0)
        setpoints[ROLL] = 0;
    else if (rc_values[ROLL] == 1500)
        setpoints[ROLL] = 0;
    else
        setpoints[ROLL] = map(rc_values[ROLL], 1000, 2000, max_roll_setpoint, -max_roll_setpoint);

    if (rc_values[THROTTLE] <= 1000)
        setpoints[THROTTLE] = 1000;
    else
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
    float delta_err[3] = {0, 0, 0}; // Error deltas in that order: Yaw, Pitch, Roll
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
        // Calculate sum of errors : Integral coefficients. Cap if needed.
        error_sum[YAW] = minMax(error_sum[YAW] + errors[YAW], -max_i_yaw, max_i_yaw);
        error_sum[PITCH] = minMax(error_sum[PITCH] + errors[PITCH], -max_i_pitch, max_i_pitch);
        error_sum[ROLL] = minMax(error_sum[ROLL] + errors[ROLL], -max_i_roll, max_i_roll);

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
        pulse_length_esc1 = setpoints[THROTTLE] - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc2 = setpoints[THROTTLE] + roll_pid + pitch_pid + yaw_pid;
        pulse_length_esc3 = setpoints[THROTTLE] - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc4 = setpoints[THROTTLE] + roll_pid - pitch_pid - yaw_pid;

        pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 1800);
        pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 1800);
        pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 1800);
        pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 1800);
    }
    else
    {
        reset_pid();
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

void ros_publish()
{
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

void init_mpu()
{
    Wire.begin();
    mpu.initialize();
    mpu.setDMPEnabled(false);
    uint8_t devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        uint8_t FS_SEL = 0;
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        // FIX: the commented line was before, but it maked no sense.
        // GYRO_FACTOR = 131.0 / (FS_SEL + 1);
        GYRO_FACTOR = 131.0 / (READ_FS_SEL + 1);
    }
    calibrate_sensors();
    set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

void calibrate_sensors()
{
    int num_readings = 100;

    // Discard the first reading (don't know if this is needed or
    // not, however, it won't hurt.)
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Read and average the raw values
    for (int i = 0; i < num_readings; i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        base_x_gyro += gx;
        base_y_gyro += gy;
        base_z_gyro += gz;
        base_x_accel += ax;
        base_y_accel += ay;
        base_y_accel += az;
    }

    base_x_gyro /= num_readings;
    base_y_gyro /= num_readings;
    base_z_gyro /= num_readings;
    base_x_accel /= num_readings;
    base_y_accel /= num_readings;
    base_z_accel /= num_readings;
}

void reboot_drone(const std_msgs::Empty &toggle_msg)
{
    digitalWrite(7, HIGH);
    reset_pid();
    get_ros_params();
    init_mpu();
    delay(500);
    digitalWrite(7, LOW);
}

void update_params(const std_msgs::Empty &toggle_msg)
{
    digitalWrite(7, HIGH);
    reset_pid();
    get_ros_params();
    delay(150);
    digitalWrite(7, LOW);
}

void enable_motors_function(const std_msgs::Bool &msg)
{
    stop_all();
    enable_motors = msg.data;
}

void get_ros_params()
{
    if (!arduino_node.getParam("kp", kp, 3))
    {
        // Default values
        kp[YAW] = 0;
        kp[PITCH] = 0;
        kp[ROLL] = 0;
    }
    if (!arduino_node.getParam("kd", kd, 3))
    {
        // Default values
        kd[YAW] = 0;
        kd[PITCH] = 0;
        kd[ROLL] = 0;
    }
    if (!arduino_node.getParam("ki", ki, 3))
    {
        // Default values
        ki[YAW] = 0;
        ki[PITCH] = 0;
        ki[ROLL] = 0;
    }
     if (! arduino_node.getParam("min_read_rc", min_read_rc, 4)){
    //default values
    min_read_rc[YAW] = 1250;
    min_read_rc[PITCH] = 1250;
    min_read_rc[ROLL] = 1250; 
    min_read_rc[THROTTLE] = 1250;
  }
  if (! arduino_node.getParam("max_read_rc", max_read_rc, 4)){
    //default values
    max_read_rc[YAW] = 1800;
    max_read_rc[PITCH] = 1800;
    max_read_rc[ROLL] = 1800; 
    max_read_rc[THROTTLE] = 1800;
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
        if (rc_value > 1450 && rc_value < 1550)
            rc_value = 1500;
        rc_values[channel] = rc_value;
    }
}

uint16_t rc_map(uint16_t timer, int min, int max)
{
    uint16_t value = (uint16_t)(micros() - timer);
    //
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
