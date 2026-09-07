// IMU.h
#ifndef IMU_H
#define IMU_H

// ---- Pin Definitions ----
extern const int CS_PIN; // GP17

// IMU scalars
const float ACCEL_SCALE = 16384.0 / 9.82; // 1g = 9.82 m/s^2, 16384 LSB/g for ±2g range
const float GYRO_SCALE = 131.0 / (PI / 180.0); // 1°/s = π/180 rad/s, 131 LSB/(°/s) for ±250°/s range
const float TEMP_SCALE = 340.0;

// ---- IMU Data Structures ----
struct IMUstruct {
  Vector3 accel;
  Vector3 gyro;
};
struct CalibrationOffsetsIMU {
  Vector3 accel;
  Vector3 gyro;
};

// ---- IMU Functions ----
// Send data to register magic
void writeRegister(byte reg, byte data);
// Read IMU data and apply calibration offsets
IMUstruct readIMU(CalibrationOffsetsIMU &IMU_Offsets);
// Find calibration offsets for IMU
void callibIMU(CalibrationOffsetsIMU &offsets);
// ---- Orientation and Translation Vectors ----
// (-, pitch, roll) from accelerometer and gyroscope data
Vector3 IMUyprOrientation(const Vector3& accel, const Vector3& gyro, float dt);
Vector3 IMUxyzTranslation(const Vector3& accel, const Vector3& gyro, float dt);

#endif // IMU_H