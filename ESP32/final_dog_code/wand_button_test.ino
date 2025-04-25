#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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

// Pin definitions
#define ULTRASONIC_TRIG_PIN 2
#define ULTRASONIC_ECHO_PIN 4
#define LDR_DIGITAL_PIN 15

// WAND SETUP
uint8_t wandMAC[] = {0xD4, 0x8C, 0x49, 0x57, 0xEF, 0x48};

struct WandData {
  float accelX, accelY, accelZ;
  int button1State;
  int button2State;
  int button3State;
  int button4State;
};

WandData receivedWandData;
float accelX_offset = 0.0, accelY_offset = 0.0, accelZ_offset = 0.0;

long readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  //if (duration == 0) {
    //Serial.println("Ultrasonic sensor timeout");
    //return -1; // Indicate no reading
 
  return (duration * 0.034) / 2;
}

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

  // Serial.print("hipAngle = ");
  // Serial.println(hipAngle);
  // Serial.print("kneeAngle = ");
  // Serial.println(kneeAngle);
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

// ESP-NOW: Receive Button States
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len == sizeof(WandData)) {
    memcpy(&receivedWandData, data, sizeof(receivedWandData));
    Serial.print("Button States: ");
    Serial.print(receivedWandData.button1State);
    Serial.print(", ");
    Serial.print(receivedWandData.button2State);
    Serial.print(", ");
    Serial.print(receivedWandData.button3State);
    Serial.print(", ");
    Serial.println(receivedWandData.button4State);
    
    Serial.print("receivedWandData.accelX: ");
    Serial.println(receivedWandData.accelX);

    Serial.print("receivedWandData.accelY: ");
    Serial.println(receivedWandData.accelY);

    Serial.print("receivedWandData.accelZ: ");
    Serial.println(receivedWandData.accelZ);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nLEFT Half Leg Test...");

  board1.begin();
  board1.setPWMFreq(60);  // Set frequency to 50/60 Hz for analog servos

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LDR_DIGITAL_PIN, INPUT);


  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, wandMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    while (true);
  }

  esp_now_register_recv_cb(onReceive);

  moveSelectedLegsToPose(2, 3, -6, -6, 800);
  moveSelectedLegsToPose(0, 1, -6, -6, 800);
}


void loop() {
  if (receivedWandData.accelX < -4 && receivedWandData.accelZ > 1) { // DOWN
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, -11, -4, 800);
  }

  else if (receivedWandData.accelX > 5 && receivedWandData.accelZ > 1) { // UP
    Serial.println("Standing up (2 legs at a time)");
    moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Front Legs up
    delay(10);
    moveSelectedLegsToPose(0, 1, 3, -9, 50); // Front Legs Down

    moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Back Legs up
    delay(10);
    moveSelectedLegsToPose(2, 3, 5, -9, 1); // Back Legs Down
  }


  long distance = readUltrasonicDistance();
  // Serial.print("Distance: ");
  // Serial.println(distance);

  if (distance <= 7 && distance != 0) {
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, -6, 8, 800);
  }
  
  // Test LDR
  int ldrValue = digitalRead(LDR_DIGITAL_PIN);
  // Serial.print("LDR Value: ");
  // Serial.println(ldrValue == LOW ? "Bright" : "Dark");  // Flip logic here
  //delay(1000);

  if (ldrValue != LOW && ldrValue != 0) {
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, -11, -4, 800);
  }

  // Button 1 -> Sitting
  if (receivedWandData.button1State == 0) {
    Serial.println("Sitting pose");
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, 0, -4, 200);
    moveSelectedLegsToPose(0, 1, 0, -11, 200);
  }

  else if (receivedWandData.button4State == 0) {

  }

  // Button 2 -> Wave
  else if (receivedWandData.button2State == 0) {
    Serial.println("Waving");
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, -6, -6, 800);

    moveSingleLegToPose(0, -5, 5, 200);
    moveSingleLegToPose(0, -6, -6, 200);
    moveSingleLegToPose(0, -5, 5, 200);
    moveSingleLegToPose(0, -6, -6, 200);
  }

  // Button 3 -> Stand Up
  else if (receivedWandData.button3State == 0) {
    Serial.println("Standing up (2 legs at a time)");
    moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Front Legs up
    delay(10);
    moveSelectedLegsToPose(0, 1, 3, -9, 50); // Front Legs Down

    moveSelectedLegsToPose(2, 3, 5, -5, 1); // Move Back Legs up
    delay(10);
    moveSelectedLegsToPose(2, 3, 5, -9, 1); // Back Legs Down
  }

  else {
    moveSelectedLegsToPose(2, 3, -6, -6, 800);
    moveSelectedLegsToPose(0, 1, -6, -6, 800);
  }
}