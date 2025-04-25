#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

#define SER0  0   // Servo Motor 0 on connector 0
#define SER1  1   // Servo Motor 1 on connector 1
#define SER2  2   // Servo Motor 2 on connector 2
#define SER3  3   // Servo Motor 3 on connector 3
#define SER4  4   // Servo Motor 0 on connector 4
#define SER5  5   // Servo Motor 1 on connector 5
#define SER6  6   // Servo Motor 2 on connector 6
#define SER7  7   // Servo Motor 3 on connector 7

// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;
int pwm2;
int pwm3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Servo test!");

  board1.begin();
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
  // pwm0 = map(0, 0, 180, SERVOMIN, SERVOMAX);
  // board1.setPWM(SER0, 0, pwm0);

  // pwm1 = map(0, 0, 180, SERVOMIN, SERVOMAX);
  // board1.setPWM(SER1, 0, pwm1);

  // put your main code here, to run repeatedly:
  for(int angle=0; angle<=180; angle+=10){
    Serial.print("Motor Angle = ");
    Serial.println(angle);

    pwm0 = map(0, 0, 180, SERVOMIN, SERVOMAX); // Hip
    pwm1 = map(0, -180, 0, SERVOMIN, SERVOMAX); // Knee
    pwm2 = map(angle, 0, 180, SERVOMIN, SERVOMAX); 
    pwm3 = map(-angle, -180, 0, SERVOMIN, SERVOMAX); 

    int pwm90 = map(90, 0, 180, SERVOMIN, SERVOMAX); 
    int pwmm90 = map(-90, -180, 0, SERVOMIN, SERVOMAX);


    board1.setPWM(SER0, 0, pwm0);
    board1.setPWM(SER1, 0, pwm0);
    board1.setPWM(SER2, 0, pwm0);
    board1.setPWM(SER3, 0, pwm0);
    board1.setPWM(SER4, 0, pwm1);
    board1.setPWM(SER5, 0, pwm1);
    board1.setPWM(SER6, 0, pwm1);
    board1.setPWM(SER7, 0, pwm1);
    delay(500);
  }

  // for(int angle=0; angle<=90; angle+=10){
  //   pwm1 = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  //   board1.setPWM(SER1, 0, pwm1);
  //   delay(500);
  // }

  delay(500);
}

