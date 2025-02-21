#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
void setup() {
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  // put your main code here, to run repeatedly:
  pwm.setPWM(15, 0, 4095);
  delay(3000);
  pwm.setPWM(15,0, 0);
  delay(3000);
}
