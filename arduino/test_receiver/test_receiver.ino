#include <PinChangeInt.h>
#include <Servo.h>
#define THROTTLE_PIN 10 // we could choose any pin
 
volatile int pwm_value = 0;
volatile int prev_time = 0;
uint8_t latest_interrupted_pin;

int value = 1000;
Servo firstESC, secondESC, thirdESC, fourthESC;

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
  pinMode(THROTTLE_PIN, INPUT); digitalWrite(THROTTLE_PIN, HIGH);
  firstESC.attach(4);
  secondESC.attach(5);
  thirdESC.attach(6);
  fourthESC.attach(7);
  Serial.begin(115200);
  PCintPort::attachInterrupt(THROTTLE_PIN, &rising, RISING);
}

void loop() {
  firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  thirdESC.writeMicroseconds(value);
  fourthESC.writeMicroseconds(value);
  Serial.println(value);
}
