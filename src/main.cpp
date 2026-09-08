// ---- RP2040 Flight Controller ----
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "VectorMath.h"
#include "IMU.h"

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20
const int motor_PIN = 22; // GP22

// ---- IMU constants (memory addresses) ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

int worthless_integer = 0;

// ---- Arduino Setup and Loop ----
void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(motor_PIN, OUTPUT);
  digitalWrite(motor_PIN, LOW);

  delay(100); // Wait for hardware to stabilize

  writeRegister(PWR_MGMT_1, 0x00); // Väck IMU!
  
  Serial.println("--- Setup complete ---");
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Loop-time (dt) calculation
  static unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  if (lastTime == 0) lastTime = currentTime; 
  float dt = (currentTime - lastTime) / 1000000.0f; // us to s
  lastTime = currentTime;

  // Make IMU offsets static to survive loop iterations
  static IMUCalibrationData calibrationData;
  if (worthless_integer == 0) {
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    Serial.print("Callibrating IMU...\n");
    callibIMU(calibrationData);

    worthless_integer = 1;
    Serial.print("--- Setup complete ---\n");
    digitalWrite(LED_PIN, LOW);
  }

  // Read IMU data
  IMUstruct data = readIMU(calibrationData);
  Vector3 accel = data.accel;
  Vector3 gyro = data.gyro;
  Vector3 orientation = IMUyprOrientation(accel, gyro, dt);
  Vector3 translation = IMUxyzTranslation(accel, gyro, dt); // INTE KLAR

  // Print accelerometer and gyroscope data to the serial monitor
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Gyro:  (%.4f,  %.4f,  %.4f)\n", gyro.x, gyro.y, gyro.z);
  //Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Accel: (%.4f,  %.4f,  %.4f)\n", accel.x, accel.y, accel.z);
  Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Pitch: %.4f   Roll: %.4f\n", orientation.y, orientation.z);
  //Serial.print(buffer);

  // Allert when upside down
  if (accel.z > 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }


  delay(1000);
}