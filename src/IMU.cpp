// IMU.cpp
#include "IMU.h"

// ---- Pin Definitions ----
extern const int CS_PIN = 17; // GP17

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
IMUstruct readIMU(CalibrationOffsetsIMU &IMU_Offsets) {
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
  float accelX = (rawAccelX / ACCEL_SCALE) - IMU_Offsets.accel.x;
  float accelY = (rawAccelY / ACCEL_SCALE) - IMU_Offsets.accel.y;
  float accelZ = (rawAccelZ / ACCEL_SCALE) - IMU_Offsets.accel.z;
  float gyroX = (rawGyroX / GYRO_SCALE) - IMU_Offsets.gyro.x;
  float gyroY = (rawGyroY / GYRO_SCALE) - IMU_Offsets.gyro.y;
  float gyroZ = (rawGyroZ / GYRO_SCALE) - IMU_Offsets.gyro.z;

  return {Vector3{accelX, accelY, accelZ}, Vector3{gyroX, gyroY, gyroZ}};
}
void callibIMU(CalibrationOffsetsIMU &offsets) {
  // Kalibrera accelerometern
  offsets = {Vector3{0, 0, 0}, Vector3{0, 0, 0}};
  delay(500); // Debounce delay (prevents calibrating for vibrations when the button is pressed)

  // Rolling average - more stable measurements
  IMUstruct base = {Vector3{0,0,0}, Vector3{0,0,0}}; // Zero basevalues
  int numSamples = 200;
  for (int i = 0; i < numSamples; i++) {
    IMUstruct temp = readIMU(offsets);
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
