// IMU.cpp
#include <SPI.h>
#include <math.h>
#include "VectorMath.h"
#include "IMU.h"

// ---- IMU constants ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

// Send data to register magic
void writeRegister(byte reg, byte data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}
// Read IMU data and apply calibration offsets
IMUstruct readIMU(IMUCalibrationData &calibrationData) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(ACCEL_XOUT_H | 0x80); // Read IMU

  int16_t rawAccelX = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawAccelY = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawAccelZ = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t readRawTemp = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00); // 
  int16_t rawGyroX = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawGyroY = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t rawGyroZ = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  // Translate raw values to actual values (float)
  float accelX = (rawAccelX / ACCEL_SCALE) - calibrationData.accel.x;
  float accelY = (rawAccelY / ACCEL_SCALE) - calibrationData.accel.y;
  float accelZ = (rawAccelZ / ACCEL_SCALE) - calibrationData.accel.z;

  // Low-Pass Filter Gyro-data
  static float filteredGyroX = 0;
  static float filteredGyroY = 0;
  static float filteredGyroZ = 0;
  float filterFactor = 0.7f;
  filteredGyroX = (filterFactor * filteredGyroX) + ((1.0f - filterFactor) * rawGyroX);
  filteredGyroY = (filterFactor * filteredGyroY) + ((1.0f - filterFactor) * rawGyroY);
  filteredGyroZ = (filterFactor * filteredGyroZ) + ((1.0f - filterFactor) * rawGyroZ);

  float gyroX = (filteredGyroX / GYRO_SCALE) - calibrationData.gyro.x;
  float gyroY = (filteredGyroY / GYRO_SCALE) - calibrationData.gyro.y;
  float gyroZ = (filteredGyroZ / GYRO_SCALE) - calibrationData.gyro.z;

  Vector3 accel = {accelX, accelY, accelZ};
  Vector3 gyro = {gyroX, gyroY, gyroZ};

  // Alow drone developer to mount IMU in any orientation:
  accel = calibrationData.R * accel;
  gyro = calibrationData.R * gyro;

  return {accel, gyro};
}
void callibIMU(IMUCalibrationData &calibrationData) {
  // Callibrate IMU
  calibrationData = {Vector3{0, 0, 0}, Vector3{0, 0, 0}, Matrix3x3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}}; // Reset offsets for new callibration
  
  // Estimate current accelereration - Rolling average
  writeRegister(0x1A, 0x03); // (Digital Low Pass Filter - DLPF) ish 42 Hz, 
  IMUstruct base = {Vector3{0,0,0}, Vector3{0,0,0}}; // Zero basevalues
  int numSamples = 200;
  for (int i = 0; i < numSamples; i++) {
    IMUstruct temp = readIMU(calibrationData);
    base.accel = base.accel + temp.accel;
    base.gyro = base.gyro + temp.gyro;
    delay(10);
  }
  base.accel = base.accel * (1.0 / numSamples);
  base.gyro = base.gyro * (1.0 / numSamples);
  
  const float g = 9.82; // Gravity constant (m/s^2)

  Matrix3x3 R;
  Vector3 ideal_accel = {0.0f, 0.0f, -g};
  Vector3 norm_base = normalize(base.accel);
  Vector3 norm_ideal = normalize(ideal_accel);
  Vector3 axis = normalize(cross(norm_base, norm_ideal));

  float cosTheta = dot(norm_base, norm_ideal);
  float sinTheta = sqrt(1.0f - (cosTheta * cosTheta));
  float v = 1.0f - cosTheta;

  R.m[0][0] = cosTheta + (axis.x * axis.x * v);
  R.m[0][1] = (axis.x * axis.y * v) - (axis.z * sinTheta);
  R.m[0][2] = (axis.x * axis.z * v) + (axis.y * sinTheta);
  
  R.m[1][0] = (axis.y * axis.x * v) + (axis.z * sinTheta);
  R.m[1][1] = cosTheta + (axis.y * axis.y * v);
  R.m[1][2] = (axis.y * axis.z * v) - (axis.x * sinTheta);
  
  R.m[2][0] = (axis.z * axis.x * v) - (axis.y * sinTheta);
  R.m[2][1] = (axis.z * axis.y * v) + (axis.x * sinTheta);
  R.m[2][2] = cosTheta + (axis.z * axis.z * v);

  // Rotational correction:
  calibrationData.R = R; 

  // Accelerometer drift compensation (with rotational correction):
  Vector3 accel_offset = (R * base.accel) - ideal_accel;
  calibrationData.accel = transpose(R) * accel_offset;

  // Angular drift compensation:
  calibrationData.gyro.x = base.gyro.x;
  calibrationData.gyro.y = base.gyro.y;
  calibrationData.gyro.z = base.gyro.z;
}
// ---- Orientation and Translation Vectors ----
// (-, pitch, roll) from accelerometer and gyroscope data
Vector3 IMUyprOrientation(const Vector3& accel, const Vector3& gyro, float dt) {
  // Calculate roll and pitch from accelerometer
  //static float yaw = 0; // PLACEHOLDER
  static float pitch = 0;
  static float roll = 0;

  // Accelerometer angles (rad)
  float accPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
  float accRoll = atan2(accel.y, accel.z);

  // Gyroscope rates (rad/s)
  float gyroRoll = gyro.x;
  float gyroPitch = gyro.y;

  // <3 komplementärfiltret <3
  float TrustFactor = 0.05; // Accelerometer weight (0.0 -> 1.0)
  pitch = (1 - TrustFactor) * (pitch + (gyroPitch * dt)) + TrustFactor * accPitch;
  roll  = (1 - TrustFactor) * (roll + (gyroRoll * dt)) + TrustFactor * accRoll;

  return {0.0, pitch, roll}; // Yaw is not included
}
Vector3 IMUxyzTranslation(const Vector3& accel, const Vector3& gyro, float dt) {
  // Placeholder function for translation vector calculation
  // This would typically involve integrating the accelerometer data over time
  // and applying any necessary corrections for drift and orientation.
  return {0.0, 0.0, 0.0}; // Placeholder return value
}
