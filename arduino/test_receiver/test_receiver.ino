#include <PinChangeInt.h>
#include <Servo.h>
#define MY_PIN 5 // we could choose any pin
 
volatile int pwm_value = 0;
volatile int prev_time = 0;
uint8_t latest_interrupted_pin;

int value = 1000;
Servo firstESC;

void rising()
{
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
  pwm_value = micros()-prev_time;
  //Serial.println(pwm_value);
  if (pwm_value < 984) pwm_value = 984;
  else if (pwm_value > 1628) pwm_value = 1628;
  value = map(pwm_value, 984, 1628, 1000, 2000);
}
 
void setup() {
  pinMode(MY_PIN, INPUT); digitalWrite(MY_PIN, HIGH);
  firstESC.attach(3);
  Serial.begin(115200);
  PCintPort::attachInterrupt(MY_PIN, &rising, RISING);
}

void loop() {
  firstESC.writeMicroseconds(value);
  Serial.println(value);
}
