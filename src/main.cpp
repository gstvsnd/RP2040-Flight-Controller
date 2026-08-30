#include <Arduino.h>

#include "FlightController.h"

#include <SPI.h>
#include <math.h>
#include <time.h>
#include <stdio.h>

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20
const int Button_PIN = 21; // GP21

const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

const float ACCEL_SCALE = 16384.0 / 9.82; // 1g = 9.82 m/s^2, 16384 LSB/g for ±2g range
const float GYRO_SCALE = 131.0 / (PI / 180.0); // 1°/s = π/180 rad/s, 131 LSB/(°/s) for ±250°/s range
const float TEMP_SCALE = 340.0;


// ---- Vector and Matrix structures for 3D transformations ----
// 3D vector structure
struct Vector3 {
  float x, y, z;
};
// 3x3 Matrix for 3D transformations
struct Matrix3x3 {
  float m[3][3]; 
};
// 3D vector - 3D vector:
// 3D vector subtraction
Vector3 operator-(const Vector3& a, const Vector3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
// 3D vector addition
Vector3 operator+(const Vector3& a, const Vector3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
// 3D vector - scalar (float):
// 3D vector multiplied by a scalar (float)
Vector3 operator*(const Vector3& vec, float scalar) {
  return {vec.x * scalar, vec.y * scalar, vec.z * scalar};
}
// 3D vector - 3x3 matrix:
// 3D vector multiplication with a 3x3 matrix
Vector3 operator*(const Matrix3x3& mat, const Vector3& vec) {
  Vector3 result;
  // row1 * (x, y, z)
  result.x = (mat.m[0][0] * vec.x) + (mat.m[0][1] * vec.y) + (mat.m[0][2] * vec.z);
  // row2 * (x, y, z)
  result.y = (mat.m[1][0] * vec.x) + (mat.m[1][1] * vec.y) + (mat.m[1][2] * vec.z);
  // row3 * (x, y, z)
  result.z = (mat.m[2][0] * vec.x) + (mat.m[2][1] * vec.y) + (mat.m[2][2] * vec.z);
  return result;
}
// Common operations for 3D vectors:
// Dot product (How much vector_a goes in the direction of vector_b)
float dot(const Vector3& a, const Vector3& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}
// Cross product (Vector orthogonal to vector_a and vector_b)
Vector3 cross(const Vector3& a, const Vector3& b) {
  return {
    (a.y * b.z) - (a.z * b.y),
    (a.z * b.x) - (a.x * b.z),
    (a.x * b.y) - (a.y * b.x)
  };
}
// Distance between two 3D positonal vectors (Euclidean distance)
float eucledianDistance(const Vector3& a, const Vector3& b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
// Magnitude/Length
float magnitude(const Vector3& vec) {
  return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

// ---- MPU Data Structures ----
struct MPUstruct {
  Vector3 accel;
  Vector3 gyro;
};
struct CalibrationOffsetsMPU {
  Vector3 accel;
  Vector3 gyro;
};

// Send data to register magic
void writeRegister(byte reg, byte data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

MPUstruct readMPU(CalibrationOffsetsMPU &MPU_Offsets) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(ACCEL_XOUT_H | 0x80); // Läs ut accelerometerdata (MSB först)

  int16_t rawAccelX = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawAccelY = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawAccelZ = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);

  int16_t readRawTemp = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00); // 

  int16_t rawGyroX = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawGyroY = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawGyroZ = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  // Translate raw values to actual values (float)
  float accelX = (rawAccelX / ACCEL_SCALE) - MPU_Offsets.accel.x;
  float accelY = (rawAccelY / ACCEL_SCALE) - MPU_Offsets.accel.y;
  float accelZ = (rawAccelZ / ACCEL_SCALE) - MPU_Offsets.accel.z;
  float gyroX = (rawGyroX / GYRO_SCALE) - MPU_Offsets.gyro.x;
  float gyroY = (rawGyroY / GYRO_SCALE) - MPU_Offsets.gyro.y;
  float gyroZ = (rawGyroZ / GYRO_SCALE) - MPU_Offsets.gyro.z;

  return {Vector3{accelX, accelY, accelZ}, Vector3{gyroX, gyroY, gyroZ}};
}

void callibMPU(CalibrationOffsetsMPU &offsets) {
  // Kalibrera accelerometern
  offsets = {Vector3{0, 0, 0}, Vector3{0, 0, 0}};
  delay(500); // Debounce delay (prevents calibrating for vibrations when the button is pressed)

  // Rolling average - more stable measurements
  MPUstruct base = {Vector3{0,0,0}, Vector3{0,0,0}}; // Zero basevalues
  int numSamples = 200;
  for (int i = 0; i < numSamples; i++) {
    MPUstruct temp = readMPU(offsets);
    base.accel = base.accel + temp.accel;
    base.gyro = base.gyro + temp.gyro;
    delay(10);
  }
  base.accel = base.accel * (1.0 / numSamples);
  base.gyro = base.gyro * (1.0 / numSamples);

  // Spara ner de nya värdena som offsets.
  float gravity = 9.82; // Assume 9.82m/s for gravity
  offsets.accel.x = base.accel.x;
  offsets.accel.y = base.accel.y;
  offsets.accel.z = base.accel.z - gravity;
  
  offsets.gyro.x = base.gyro.x;
  offsets.gyro.y = base.gyro.y;
  offsets.gyro.z = base.gyro.z;
}

// ---- Orientation and Translation Vectors ----
// (yaw, pitch, roll) from accelerometer and gyroscope data
float MCUAccRoll(const Vector3& accel) {
  float roll = atan2(accel.y, accel.z);
  return roll;
}
float MCUAccPitch(const Vector3& accel) {
  float pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
  return pitch;
}
Vector3 MCUyprOrientation(const Vector3& accel, const Vector3& gyro) {
  // Calculate roll and pitch from accelerometer
  float yaw = 0; // Placeholder for yaw calculation
  float pitch = MCUAccPitch(accel);
  float roll = MCUAccRoll(accel);

  // Integrate gyroscope data to get orientation
  //roll += gyro.x * dt;
  //pitch += gyro.y * dt;

  return {roll, pitch, 0}; // Yaw is not calculated here
}

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

  writeRegister(PWR_MGMT_1, 0x00); // Väck MCU!
  delay(100);
}

void loop() {
  // Make MCU offsets static to survive loop iterations
  static CalibrationOffsetsMPU MPU_Offsets;

  // Read MCU data
  MPUstruct data = readMPU(MPU_Offsets);
  Vector3 accel = data.accel;
  Vector3 gyro = data.gyro;
  Vector3 orientation = MCUyprOrientation(accel, gyro); // NÄSTAN KLAR
  Vector3 translation = {0, 0, 0}; // INTE KLAR

  // Print accelerometer and gyroscope data to the serial monitor
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Gyro:  (%.4f,  %.4f,  %.4f)\n", gyro.x, gyro.y, gyro.z);
  Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Accel: (%.4f,  %.4f,  %.4f)\n", accel.x, accel.y, accel.z);
  Serial.print(buffer);

  // 
  if (accel.z < 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  if (digitalRead(Button_PIN) == LOW) {
    Serial.println("Callibrating MPU...");
    callibMPU(MPU_Offsets);
  }

  delay(250);
}