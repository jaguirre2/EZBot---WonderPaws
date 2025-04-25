#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// Servo channels
#define HIP_SERVO_CHANNEL_FRONT_LEG_LEFT  0
#define KNEE_SERVO_CHANNEL_FRONT_LEG_LEFT   1
#define HIP_SERVO_CHANNEL_BACK_LEG_LEFT  2
#define KNEE_SERVO_CHANNEL_BACK_LEG_LEFT   3

#define HIP_SERVO_CHANNEL_FRONT_LEG_RIGHT  4
#define KNEE_SERVO_CHANNEL_FRONT_LEG_RIGHT   5
#define HIP_SERVO_CHANNEL_BACK_LEG_RIGHT  6
#define KNEE_SERVO_CHANNEL_BACK_LEG_RIGHT   7

int angle = 0;
int step = 10;
const float stepSize = 1;

float hipAngle = 0;
float kneeAngle = 0;

float targetX = -6;
float targetY = -6;

float angles[8] = {0};

// Leg constants (in cm)
#define FEMUR_LEN   5.91912456
#define TIBIA_LEN   6.8813172

// Function to convert angle to pulse length
int angleToPulse(float angle, String dir) {
  if (dir == "left") {
    return map(constrain(angle, 0, 180), 0, 180, SERVOMIN, SERVOMAX);
  }

  else if (dir == "right") {
    return map(constrain(-angle, -180, 0), -180, 0, SERVOMIN, SERVOMAX);
  }
}

void calculateJointAngles(float X_given, float Y_given, float &hipAngle, float &kneeAngle) {
  // Make X & Y their absolute value counterparts
  float X = abs(X_given);
  float Y = abs(Y_given);
  
  float Z = sqrt((X * X) + (Y * Y));
  if (Z > (FEMUR_LEN+TIBIA_LEN)) return;

  if (Y_given > 0) {
    float alpha1 = degrees(acos(((Z * Z) + (FEMUR_LEN * FEMUR_LEN) - (TIBIA_LEN * TIBIA_LEN)) / (2 * Z * FEMUR_LEN)));
    float beta1 = degrees(acos(((X * X) + (Z * Z) - (Y * Y)) / (2 * X * Z)));
    hipAngle = alpha1 - beta1;
  }

  else if (Y_given <= 0) {
    if (X_given < 0) {
      float alpha1 = degrees(acos(((Z * Z) + (FEMUR_LEN * FEMUR_LEN) - (TIBIA_LEN * TIBIA_LEN)) / (2 * Z * FEMUR_LEN)));
      float beta1 = degrees(acos(((X * X) + (Z * Z) - (Y * Y)) / (2 * X * Z)));
      hipAngle = alpha1 + beta1;
    }

    if (X_given >= 0) {
      float alpha1 = degrees(acos(((Y * Y) + (Z * Z) - (X * X)) / (2 * Y * Z)));
      float beta1 = degrees(acos(((Z * Z) + (FEMUR_LEN * FEMUR_LEN) - (TIBIA_LEN * TIBIA_LEN)) / (2 * Z * FEMUR_LEN)));
      hipAngle = alpha1 + beta1 + 90;
    }
  }

  else {
    Serial.println("Error with calculateJointAngles...");
  }

  kneeAngle = degrees(acos(((FEMUR_LEN * FEMUR_LEN) + (TIBIA_LEN * TIBIA_LEN) - (Z * Z)) / (2 * FEMUR_LEN * TIBIA_LEN)));

  Serial.print("hipAngle = ");
  Serial.println(hipAngle);
  Serial.print("kneeAngle = ");
  Serial.println(kneeAngle);
}

void moveAllServosToAngles(float targetAngles[8], int duration = 500) {
  int steps = 50;
  float startAngles[8];
  float stepSizes[8];

  for (int i = 0; i < 8; i++) {
    startAngles[i] = angles[i];
    stepSizes[i] = (targetAngles[i] - startAngles[i]) / steps;
  }

  for (int s = 0; s <= steps; s++) {
    for (int i = 0; i < 8; i++) {
      float a = startAngles[i] + stepSizes[i] * s;
      angles[i] = a;
      int pwm = angleToPulse(a, i >= 4 ? "right" : "left"); // ← NEW PWM conversion
      board1.setPWM(i, 0, pwm); // ← Replaces .write() call
    }
    delay(duration / steps);
  }
}

void moveSingleLegToPose(int leg, float x, float y, int duration = 500) {
  float hip, knee;
  calculateJointAngles(x, y, hip, knee);  // Updated variable names

  float targetAngles[8];
  for (int i = 0; i < 8; i++) targetAngles[i] = angles[i]; // copy current angles

  switch (leg) {
    case 0: targetAngles[0] = hip; targetAngles[1] = knee; break; // Front Left
    case 1: targetAngles[4] = hip; targetAngles[5] = knee; break; // Front Right
    case 2: targetAngles[2] = hip; targetAngles[3] = knee; break; // Back Left
    case 3: targetAngles[6] = hip; targetAngles[7] = knee; break; // Back Right
    default: Serial.println("Invalid leg number (0-3)."); return;
  }

  moveAllServosToAngles(targetAngles, duration);
}

void moveSelectedLegsToPose(int leg1, int leg2, float x1, float y1, int duration = 500, float x2 = NAN, float y2 = NAN) {
  // Use x1, y1 for both legs if x2, y2 are not provided
  if (isnan(x2) || isnan(y2)) {
    x2 = x1;
    y2 = y1;
  }

  float hip1, knee1, hip2, knee2;
  calculateJointAngles(x1, y1, hip1, knee1); // Joint angles for first leg
  calculateJointAngles(x2, y2, hip2, knee2); // Joint angles for second leg

  float targetAngles[8];
  for (int i = 0; i < 8; i++) targetAngles[i] = angles[i]; // copy current angles

  // Apply joint angles for leg1
  switch (leg1) {
    case 0: targetAngles[0] = hip1; targetAngles[1] = knee1; break; // Front Left
    case 1: targetAngles[4] = hip1; targetAngles[5] = knee1; break; // Front Right
    case 2: targetAngles[2] = hip1; targetAngles[3] = knee1; break; // Back Left
    case 3: targetAngles[6] = hip1; targetAngles[7] = knee1; break; // Back Right
    default: Serial.println("Invalid leg1 number (0-3)."); return;
  }

  // Apply joint angles for leg2
  switch (leg2) {
    case 0: targetAngles[0] = hip2; targetAngles[1] = knee2; break;
    case 1: targetAngles[4] = hip2; targetAngles[5] = knee2; break;
    case 2: targetAngles[2] = hip2; targetAngles[3] = knee2; break;
    case 3: targetAngles[6] = hip2; targetAngles[7] = knee2; break;
    default: Serial.println("Invalid leg2 number (0-3)."); return;
  }

  moveAllServosToAngles(targetAngles, duration);
}


// void moveSelectedLegsToPose(int leg1, int leg2, float x, float y, int duration = 500) {
//   float hip, knee;
//   calculateJointAngles(x, y, hip, knee);  // Updated variable names

//   float targetAngles[8];
//   for (int i = 0; i < 8; i++) targetAngles[i] = angles[i]; // copy current angles

//   int legs[2] = {leg1, leg2};
//   for (int i = 0; i < 2; i++) {
//     int leg = legs[i];
//     switch (leg) {
//       case 0: targetAngles[0] = hip; targetAngles[1] = knee; break; // Front Left
//       case 1: targetAngles[4] = hip; targetAngles[5] = knee; break; // Front Right
//       case 2: targetAngles[2] = hip; targetAngles[3] = knee; break; // Back Left
//       case 3: targetAngles[6] = hip; targetAngles[7] = knee; break; // Back Right
//       default: Serial.println("Invalid leg number (0-3)."); return;
//     }
//   }

//   moveAllServosToAngles(targetAngles, duration);
// }

void setup() {
  Serial.begin(9600);
  Serial.println("\nLEFT Half Leg Test...");

  board1.begin();
  board1.setPWMFreq(60);  // Set frequency to 50/60 Hz for analog servos

  moveSelectedLegsToPose(2, 3, -6, -6, 800);
  moveSelectedLegsToPose(0, 1, -6, -6, 800);

  Serial.println("Standing up (2 legs at a time)");
  moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Front Legs up
  delay(10);
  moveSelectedLegsToPose(0, 1, 3, -9, 50); // Front Legs Down

  moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Back Legs up
  delay(10);
  moveSelectedLegsToPose(2, 3, 5, -9, 1); // Back Legs Down

  // float hipAngle = 0.0;
  // float kneeAngle = 0.0;
  // calculateJointAngles(targetX, targetY, hipAngle, kneeAngle);
  
  // // Calibration (set all servos to 0)
  // int pwm0 = map(98.55, 0, 180, SERVOMIN, SERVOMAX);
  // int pwm1 = map(82.67, 0, 180, SERVOMIN, SERVOMAX);

  // int pwm2 = map(98.55, -180, 0, SERVOMIN, SERVOMAX);
  // int pwm3 = map(82.67, -180, 0, SERVOMIN, SERVOMAX);

  // board1.setPWM(HIP_SERVO_CHANNEL_FRONT_LEG_LEFT, 0, pwm0);
  // board1.setPWM(KNEE_SERVO_CHANNEL_FRONT_LEG_LEFT, 0, pwm1);
  // board1.setPWM(HIP_SERVO_CHANNEL_BACK_LEG_LEFT, 0, pwm0);
  // board1.setPWM(KNEE_SERVO_CHANNEL_BACK_LEG_LEFT, 0, pwm1);

  // board1.setPWM(HIP_SERVO_CHANNEL_FRONT_LEG_RIGHT, 0, pwm2);
  // board1.setPWM(KNEE_SERVO_CHANNEL_FRONT_LEG_RIGHT, 0, pwm3);
  // board1.setPWM(HIP_SERVO_CHANNEL_BACK_LEG_RIGHT, 0, pwm2);
  // board1.setPWM(KNEE_SERVO_CHANNEL_BACK_LEG_RIGHT, 0, pwm3);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "w") {
      targetY += stepSize;
    }
    else if (input == "s") {
      targetY -= stepSize;
    }
    else if (input == "a") {
      targetX -= stepSize;
    }
    else if (input == "d") {
      targetX += stepSize;
    }

    else if (input == "9") {
      moveSelectedLegsToPose(2, 3, -6, -6, 800);
      moveSelectedLegsToPose(0, 1, -6, -6, 800);

      moveSingleLegToPose(0, -5, 5, 200);
      moveSingleLegToPose(0, -6, -6, 200);
      moveSingleLegToPose(0, -5, 5, 200);
      moveSingleLegToPose(0, -6, -6, 200);
    }

    else if (input == "0") {
      Serial.println("Resetting to rest pose");
      moveSelectedLegsToPose(2, 3, -6, -6, 800);
      moveSelectedLegsToPose(0, 1, -6, -6, 800);
    }

    else if (input == "1") {
      Serial.println("Standing up (2 legs at a time)");
      moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Front Legs up
      delay(10);
      moveSelectedLegsToPose(0, 1, 3, -9, 50); // Front Legs Down

      moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Back Legs up
      delay(10);
      moveSelectedLegsToPose(2, 3, 5, -9, 1); // Back Legs Down
    }

    else if (input == "2") {
      moveSingleLegToPose(1, 3, -5, 1); // Front Right
      delay(1);
      moveSingleLegToPose(1, 3, -9, 1);

      moveSingleLegToPose(2, 5, -5, 1); // Back Left
      delay(1);
      moveSingleLegToPose(2, 5, -9, 1);

      moveSingleLegToPose(0, 3, -5, 1); // Front Left
      delay(1);
      moveSingleLegToPose(0, 3, -9, 1);

      moveSingleLegToPose(3, 5, -5, 1); // Back Right
      delay(1);
      moveSingleLegToPose(3, 5, -9, 1);
    }

    else if (input == "3") {
      // Move to position where legs will lift it up
      moveSelectedLegsToPose(0, 3, 5, -9, 1, 5, -9);

      // Lift up legs
      moveSelectedLegsToPose(1, 2, -1.5, -5, 1, 0.5, -5);
      moveSelectedLegsToPose(1, 2, -3, -8, 1, -1, -9);
      moveSelectedLegsToPose(1, 2, 6, -7, 1, 7, -8);

      moveSelectedLegsToPose(0, 3, -1.5, -5, 1, 0.5, -5);
      
      
      // moveSelectedLegsToPose(1, 2, -1.5, -5, 1, 0.5, -5);
      // delay(10);
      // moveSelectedLegsToPose(1, 2, -3, -9, 1, -1, -9);
      // delay(10);
      // moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
    }

    else if (input == "4") {
      moveSelectedLegsToPose(0, 3, -1.5, -5, 1, 0.5, -5);
      delay(10);
      moveSelectedLegsToPose(0, 3, -3, -9, 1, -1, -9);
      delay(10);
      moveSelectedLegsToPose(0, 3, 3, -9, 1, 5, -9);
    }

    else if (input == "5") {
      moveSelectedLegsToPose(1, 2, -1.5, -5, 1, 0.5, -5);
      moveSelectedLegsToPose(1, 2, -1, -9, 1, 1, -9);
      moveSelectedLegsToPose(0, 3, 5, -9, 1, 7, -9);
      
      // delay(10);

      moveSelectedLegsToPose(0, 3, -1.5, -5, 1, 0.5, -5);
      moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
      moveSelectedLegsToPose(0, 3, -1, -9, 1, 1, -9);
      delay(10);


      moveSelectedLegsToPose(0, 1, 3, -9, 1); // Front Legs Down
      moveSelectedLegsToPose(2, 3, 5, -9, 1); // Back Legs Down
    }

    else if (input == "6") {
      // Lift 1
      moveSelectedLegsToPose(1, 2, 3, -5, 1, 5, -5);
      delay(1);

      // Down 1 & Lift 2
      moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
      moveSelectedLegsToPose(0, 3, 3, -5, 1, 5, -5);
      delay(1);

      // Lift 1 & Down 2
      moveSelectedLegsToPose(1, 2, 3, -5, 1, 5, -5);
      moveSelectedLegsToPose(0, 3, 3, -9, 1, 5, -9);
      delay(1);

      // Down 1 & Lift 2
      moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
      moveSelectedLegsToPose(0, 3, 3, -5, 1, 5, -5);
      delay(1);

      // Lift 1 & Down 2
      moveSelectedLegsToPose(1, 2, 3, -5, 1, 5, -5);
      moveSelectedLegsToPose(0, 3, 3, -9, 1, 5, -9);
      delay(1);

      // WALK CYCLE START

      // Forward 1 & Lift 2
      moveSelectedLegsToPose(1, 2, -3, -9, 1, -1, -9);
      moveSelectedLegsToPose(0, 3, -1.5, -5, 1, 0.5, -5);
      delay(1);

      // Forward 2 & Back 1
      moveSelectedLegsToPose(0, 3, -3, -9, 1, -1, -9);
      moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
      delay(1);

      // Lift 1 & Back 2
      moveSelectedLegsToPose(1, 2, -1.5, -5, 1, 0.5, -5);
      moveSelectedLegsToPose(0, 3, 3, -9, 1, 5, -9);
      delay(1);

      // Forward 1 & Lift 2
      moveSelectedLegsToPose(1, 2, -3, -9, 1, -1, -9);
      moveSelectedLegsToPose(0, 3, -1.5, -5, 1, 0.5, -5);
      delay(1);

      // Forward 2 & Back 1
      moveSelectedLegsToPose(0, 3, -3, -9, 1, -1, -9);
      moveSelectedLegsToPose(1, 2, 3, -9, 1, 5, -9);
      delay(1);

      // Lift 1 & Back 2
      moveSelectedLegsToPose(1, 2, -1.5, -5, 1, 0.5, -5);
      moveSelectedLegsToPose(0, 3, 3, -9, 1, 5, -9);
      delay(1);
    }

    // float hipAngle = 0.0;
    // float kneeAngle = 0.0;
    // calculateJointAngles(targetX, targetY, hipAngle, kneeAngle);
    
    // // Convert angles to PWM signals
    // int hipPulseLeft = angleToPulse(hipAngle, "left");
    // int kneePulseLeft = angleToPulse(kneeAngle, "left");
    // int hipPulseRight = angleToPulse(hipAngle, "right");
    // int kneePulseRight = angleToPulse(kneeAngle, "right");

    // // Move servos
    // board1.setPWM(HIP_SERVO_CHANNEL_FRONT_LEG_LEFT, 0, hipPulseLeft);
    // board1.setPWM(KNEE_SERVO_CHANNEL_FRONT_LEG_LEFT, 0, kneePulseLeft);
    // board1.setPWM(HIP_SERVO_CHANNEL_BACK_LEG_LEFT, 0, hipPulseLeft);
    // board1.setPWM(KNEE_SERVO_CHANNEL_BACK_LEG_LEFT, 0, kneePulseLeft);
    // board1.setPWM(HIP_SERVO_CHANNEL_FRONT_LEG_RIGHT, 0, hipPulseRight);
    // board1.setPWM(KNEE_SERVO_CHANNEL_FRONT_LEG_RIGHT, 0, kneePulseRight);
    // board1.setPWM(HIP_SERVO_CHANNEL_BACK_LEG_RIGHT, 0, hipPulseRight);
    // board1.setPWM(KNEE_SERVO_CHANNEL_BACK_LEG_RIGHT, 0, kneePulseRight);

    // // Output current position and angles for debugging
    // Serial.print("Current Position: X = ");
    // Serial.print(targetX);
    // Serial.print(" cm, Y = ");
    // Serial.println(targetY);
    // Serial.print("Hip Angle: ");
    // Serial.print(hipAngle);
    // Serial.print(" degrees, Knee Angle: ");
    // Serial.println(kneeAngle);
  }

  delay(100);  // Short delay to allow for serial communication
}
