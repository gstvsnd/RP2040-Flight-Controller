// VectorMath.cpp
#include <math.h>
#include "VectorMath.h"

// ---- Vector and Matrix operations ----
// 3D vector - 3D vector:
Vector3 operator-(const Vector3& a, const Vector3& b) {
// 3D vector subtraction
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vector3 operator+(const Vector3& a, const Vector3& b) {
// 3D vector addition
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
// 3D vector - scalar (float):
Vector3 operator*(const Vector3& vec, float scalar) {
// 3D vector multiplied by a scalar (float)
  return {vec.x * scalar, vec.y * scalar, vec.z * scalar};
}
// 3D vector - 3x3 matrix:
Vector3 operator*(const Matrix3x3& matrix, const Vector3& vec) {
// 3D vector multiplication with a 3x3 matrix (Matrix multiplication)
  Vector3 result;
  // row1 * (x, y, z)
  result.x = (matrix.m[0][0] * vec.x) + (matrix.m[0][1] * vec.y) + (matrix.m[0][2] * vec.z);
  // row2 * (x, y, z)
  result.y = (matrix.m[1][0] * vec.x) + (matrix.m[1][1] * vec.y) + (matrix.m[1][2] * vec.z);
  // row3 * (x, y, z)
  result.z = (matrix.m[2][0] * vec.x) + (matrix.m[2][1] * vec.y) + (matrix.m[2][2] * vec.z);
  return result;
}
// Common operations for 3D vectors:
float dot(const Vector3& a, const Vector3& b) {
// Dot product (How much vector_a goes in the direction of vector_b)
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}
Vector3 cross(const Vector3& a, const Vector3& b) {
  // Cross product (3D vector x 3D vector)
  return {
    (a.y * b.z) - (a.z * b.y),
    (a.z * b.x) - (a.x * b.z),
    (a.x * b.y) - (a.y * b.x)
  };
}
float eucledianDistance(const Vector3& a, const Vector3& b) {
// Distance between two 3D positonal vectors (Euclidean distance)
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
float magnitude(const Vector3& vec) {
// Magnitude/Length
  return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}
Vector3 normalize(const Vector3& vec) {
// Normalize vector
  float mag = magnitude(vec);
  if (mag == 0) {
    return {0, 0, 0}; // Division by zero = weird
  }
  return {vec.x / mag, vec.y / mag, vec.z / mag};
}
Matrix3x3 transpose(const Matrix3x3& matrix) {
  Matrix3x3 inverse;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      inverse.m[i][j] = matrix.m[j][i];
    }
  }
  return inverse;
}

