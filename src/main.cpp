// ---- RP2040 Flight Controller ----
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "VectorMath.h"
#include "IMU.h"

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20
const int Button_PIN = 21; // GP21

// ---- IMU constants ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

// ---- Arduino Setup and Loop ----
void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(Button_PIN, INPUT_PULLUP);

  delay(100); // Wait for hardware to stabilize

  writeRegister(PWR_MGMT_1, 0x00); // Väck IMU!
  delay(100);
}

void loop() {
  // Loop-time (dt) calculation
  static unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  if (lastTime == 0) lastTime = currentTime; 
  float dt = (currentTime - lastTime) / 1000000.0f; // us to s
  lastTime = currentTime;

  // Make IMU offsets static to survive loop iterations
  static CalibrationOffsetsIMU IMU_Offsets;


  // Read IMU data
  IMUstruct data = readIMU(IMU_Offsets);
  Vector3 accel = data.accel;
  Vector3 gyro = data.gyro;
  Vector3 orientation = IMUyprOrientation(accel, gyro, dt);
  Vector3 translation = IMUxyzTranslation(accel, gyro, dt); // INTE KLAR

  // Print accelerometer and gyroscope data to the serial monitor
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Gyro:  (%.4f,  %.4f,  %.4f)\n", gyro.x, gyro.y, gyro.z);
  //Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Accel: (%.4f,  %.4f,  %.4f)\n", accel.x, accel.y, accel.z);
  //Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Pitch: %.4f   Roll: %.4f\n", orientation.y, orientation.z);
  Serial.print(buffer);

  // 
  if (accel.z < 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  if (digitalRead(Button_PIN) == LOW) {
    Serial.println("Callibrating IMU...");
    callibIMU(IMU_Offsets);
  }

  //delay(500);
}