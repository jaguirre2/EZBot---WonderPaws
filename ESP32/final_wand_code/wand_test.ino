#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <SEMU_SSD1331.h>

#include "idle.h"

// Button pin definitions
#define BUTTON1_PIN 15 // red
#define BUTTON2_PIN 13 // blue
#define BUTTON3_PIN 5  // yellow
#define BUTTON4_PIN 14  // green
#define LED_PIN 18      // D2 pin for LED control

// You can use any (4 or) 5 pins
#define sclk 18   // marked SCL or CK on OLED board
#define mosi 23   // marked SDA or SI on OLED board
#define cs   17    // marked CS or OC on OLED board
#define rst  4   // marked RES or R on OLED board
#define dc   16   // marked DC or sometimes (confusingly) RS on OLED board

Adafruit_MPU6050 mpu;

SEMU_SSD1331 display = SEMU_SSD1331(cs, dc, mosi, sclk, rst);

unsigned long t0, t1, t2;
uint8_t c, r;
bool forward;

// Dog's MAC Address
uint8_t dogMAC[] = {0xA0, 0xB7, 0x65, 0x0D, 0x73, 0x50}; //A0:B7:65:0D:73:50

// Structure to hold Wand's sensor and button data
struct WandData {
  float accelX, accelY, accelZ;
  int button1State;
  int button2State;
  int button3State;
  int button4State;
};

// Variables to store accelerometer offsets
float accelX_offset = 0.0, accelY_offset = 0.0, accelZ_offset = 0.0;

// Function to calibrate accelerometer
void calibrateAccelerometer() {
  float sumX = 0, sumY = 0, sumZ = 0;
  const int numSamples = 100;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    sumX += accel.acceleration.x;
    sumY += accel.acceleration.y;
    sumZ += accel.acceleration.z;
    delay(10);
  }

  accelX_offset = sumX / numSamples;
  accelY_offset = sumY / numSamples;
  accelZ_offset = sumZ / numSamples;

  Serial.println("Accelerometer Calibrated:");
  Serial.printf("Offsets - X: %.2f, Y: %.2f, Z: %.2f\n", accelX_offset, accelY_offset, accelZ_offset);
}

// Variables to hold Dog's received data
struct DogSensorData {
  float accelX, accelY, accelZ;
  long distance;
  int lightDigital;
} receivedData;

// Callback for receiving data from the Dog
void onReceive(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(DogSensorData)) {
    memcpy(&receivedData, data, sizeof(DogSensorData));
    Serial.println("Received Dog Sensor Data:");
    Serial.printf("Acceleration: X=%.2f, Y=%.2f, Z=%.2f\n",
                  receivedData.accelX, receivedData.accelY, receivedData.accelZ);
    Serial.printf("Distance: %ld cm\n", receivedData.distance);
    Serial.printf("Light: %s\n", receivedData.lightDigital ? "Dark" : "Bright");

    // Check any condition to turn on the LED on pin D18
    // if (receivedData.distance < 5 ) {
    //   digitalWrite(LED_PIN, HIGH); // Turn on LED on pin D2
    // } else {
    //   digitalWrite(LED_PIN, LOW); // Turn off LED on pin D2
    // }
    // if (receivedData.lightDigital == 1 ) {
    //   digitalWrite(LED_PIN, HIGH); // Turn on LED on pin D2
    // } else {
    //   digitalWrite(LED_PIN, LOW); // Turn off LED on pin D2
    // }
    // if (receivedData.accelX > 2.0 || receivedData.accelX < -2.0) {
    //   digitalWrite(LED_PIN, HIGH); // Turn on LED on pin D2
    // } else {
    //   digitalWrite(LED_PIN, LOW); // Turn off LED on pin D2
    // Unified LED Control Logic
    if (receivedData.distance < 5 || receivedData.lightDigital == 1 ||
        receivedData.accelX > 2.0 || receivedData.accelX < -2.0) {
      digitalWrite(LED_PIN, HIGH); // Turn on LED if any condition is met
    } else {
      digitalWrite(LED_PIN, LOW); // Otherwise, keep LED off
    }
   
  }
}

// Send Wand's sensor and button data to the Dog
void sendSensorData() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  WandData wandData = {
    accel.acceleration.x - accelX_offset,
    accel.acceleration.y - accelY_offset,
    accel.acceleration.z - accelZ_offset,
    digitalRead(BUTTON1_PIN),
    digitalRead(BUTTON2_PIN),
    digitalRead(BUTTON3_PIN),
    digitalRead(BUTTON4_PIN)
  };

  esp_now_send(dogMAC, (uint8_t *)&wandData, sizeof(wandData));
}

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
 
  calibrateAccelerometer();

  // Initialize button pins
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is initially off

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (1);
  }

  // Add Dog as a peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, dogMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }

  // Register receive callback
  esp_now_register_recv_cb(onReceive);

  Serial.println("Setup complete");

  display.begin();
  display.clearWindow();
  Serial.println("Testing Dog Animations...");
  Serial.println("Idle Animation");
  forward = true;
}

void loop() {
  int button1State = digitalRead(BUTTON1_PIN);
  int button2State = digitalRead(BUTTON2_PIN);
  int button3State = digitalRead(BUTTON3_PIN);
  int button4State = digitalRead(BUTTON4_PIN);

  if (button4State == 0) { // Button 4 pressed
    Serial.println("Button 4 pressed - Recalibrating & Sending Data");
    //calibrateAccelerometer(); // Recalibrate offsets
    sendSensorData(); // Send data only when Button 4 is pressed
  }

  else if (button1State == 0) {
    sendSensorData();
  }

  else if (button2State == 0) {
    sendSensorData();
  }

  else if (button3State == 0) {
    sendSensorData();
  }

  else
  {
    calibrateAccelerometer(); // Recalibrate offsets
    sendSensorData();
  }

  for (int i = 0; i < 3; i++) {
    display.drawImage(&DogIdleAnim[i]);
    // delay(PAUSE);
  }

  for (int i = 1; i > 0; i--) {
    display.drawImage(&DogIdleAnim[i]);
    // delay(PAUSE);
  }

  delay(500);
}