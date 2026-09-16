// MotorController.cpp
#include <Arduino.h>

extern const int motor1_PIN = 4; // LB
extern const int motor2_PIN = 5; // RB
extern const int motor3_PIN = 6; // LF
extern const int motor4_PIN = 7; // RF

void mixMotors(float throttle, float yaw, float pitch, float roll) {
  // Quad X-mix
  // Coordinate System: (X=North, Y=East, Z=Down)
  // - Roll: Roll right down
  // - Pitch: Nose down
  // - Yaw: Rotate CW

  float m0 = throttle + roll - pitch - yaw; // LB CW
  float m1 = throttle - roll - pitch + yaw; // RB CCW
  float m2 = throttle + roll + pitch + yaw; // LF CCW
  float m3 = throttle - roll + pitch - yaw; // RF CW

  // convert to PWM (0 - 255) and constrain to valid ranges
  float powerFactor = 0.99f;
  int pwm0 = constrain((int)(m0 * 255.0f * powerFactor), 0, 255);
  int pwm1 = constrain((int)(m1 * 255.0f * powerFactor), 0, 255);
  int pwm2 = constrain((int)(m2 * 255.0f * powerFactor), 0, 255);
  int pwm3 = constrain((int)(m3 * 255.0f * powerFactor), 0, 255);

  // Write PWM values to motors
  analogWrite(motor1_PIN, pwm0); // LB
  analogWrite(motor2_PIN, pwm1); // RB
  analogWrite(motor3_PIN, pwm2); // LF
  analogWrite(motor4_PIN, pwm3); // RF
}
void killMotors() {
  analogWrite(motor1_PIN, 0);
  analogWrite(motor2_PIN, 0);
  analogWrite(motor3_PIN, 0);
  analogWrite(motor4_PIN, 0);
}
float computePID(float setpoint, float measured, float dt, float kp, float ki, float kd, float maxOutput, float &integrator, float &prevError) {
    // error
    float error = setpoint - measured;

    // Proportional-error
    float pOut = kp * error;

    // integral
    integrator += error * dt;
    float iOut = ki * integrator;
    if (iOut > 0.2f) {
        iOut = 0.2f;
        integrator = iOut / ki;
    }
    else if (iOut < -0.2f) {
        iOut = -0.2f;
        integrator = iOut / ki;
    }

    // derivative
    float derivative = (dt > 0) ? (error - prevError) / dt : 0;
    float dOut = kd * derivative;

    prevError = error;
    float output = pOut + iOut + dOut;

    // max out:
    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;


    return output / (2 * 3.14f);
}