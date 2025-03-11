#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define PI 3.14159263

void setup() {
  Serial.begin(115200);
  pinMode(12, OUTPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.writeMicroseconds(0, 950);
  pwm.writeMicroseconds(1, 950);
  pwm.writeMicroseconds(2, 950);
  delay(500);
  pwm.writeMicroseconds(0, 1350);
  pwm.writeMicroseconds(1, 1350);
  pwm.writeMicroseconds(2, 1350);
  delay(500);
  pwm.writeMicroseconds(0, 1500);
  pwm.writeMicroseconds(1, 1500);
  pwm.writeMicroseconds(2, 1500);
  delay(500);

}


void loop() {

  if (Serial.available() > 0) {
    // Read incoming data until newline
    String data = Serial.readStringUntil('\n');
    
    // Split the string into individual float values using commas
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    // Extract the three float values
    float value1 = data.substring(0, firstComma).toFloat();
    float value2 = data.substring(firstComma + 1, secondComma).toFloat();
    float value3 = data.substring(secondComma + 1).toFloat();

    if(value1 >= 85){value1 = 85;}
    if(value2 >= 85){value2 = 85;}
    if(value3 >= 85){value3 = 85;}

    if(value1 <= 36){value1 = 35;}
    if(value2 <= 36){value2 = 35;}
    if(value3 <= 36){value3 = 35;}

    pwm.writeMicroseconds(0, map(value1, 0, 180, 600, 2400));
    pwm.writeMicroseconds(1, map(value2, 0, 180, 600, 2400));
    pwm.writeMicroseconds(2, map(value3, 0, 180, 600, 2400));
    
  }

}