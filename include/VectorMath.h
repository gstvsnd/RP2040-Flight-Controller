// VectorMath.h
#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

// ---- Vector and Matrix structures for 3D transformations ----
// 3D vector structure
struct Vector3 {
  float x, y, z;
};
// 3x3 Matrix for 3D transformations
struct Matrix3x3 {
  float m[3][3]; 
};

// ---- Vector and Matrix operations ----
// 3D vector - 3D vector:
Vector3 operator-(const Vector3& a, const Vector3& b);
Vector3 operator+(const Vector3& a, const Vector3& b);
// 3D vector - scalar (float):
Vector3 operator*(const Vector3& vec, float scalar);
// 3D vector - 3x3 matrix:
Vector3 operator*(const Matrix3x3& mat, const Vector3& vec);
// Common operations for 3D vectors:
float dot(const Vector3& a, const Vector3& b);
Vector3 cross(const Vector3& a, const Vector3& b);
float eucledianDistance(const Vector3& a, const Vector3& b);
float magnitude(const Vector3& vec);
Vector3 normalize(const Vector3& vec);
Matrix3x3 transpose(const Matrix3x3& matrix);


#endif // VECTOR_MATH_H