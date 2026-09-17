// FlightController.cpp
#include <Arduino.h>
#include "FlightController.h"
#include "RC_Controlls.h"
#include "IMU.h"

extern const int motor1_PIN = 4; // LB
extern const int motor2_PIN = 5; // RB
extern const int motor3_PIN = 6; // LF
extern const int motor4_PIN = 7; // RF

// Motor controller
void killMotors() {
  analogWrite(motor1_PIN, 0);
  analogWrite(motor2_PIN, 0);
  analogWrite(motor3_PIN, 0);
  analogWrite(motor4_PIN, 0);
}
void mixMotors(float throttle, float yaw, float pitch, float roll) {
  // Quad X-mix
  // Coordinate System: (X=North, Y=East, Z=Down)
  // - Roll: Roll right down
  // - Pitch: Nose down
  // - Yaw: Rotate CW

  // motor mixer
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
float angularVelocityPID(float setpoint, float measured, float dt, float kp, float ki, float kd, float max_radians_second, float &integrator, float &prevError) {
    // error
    float error = (setpoint * max_radians_second) - measured; // Users goal rotation - Measured rotation

    // Proportional-error
    float pOut = kp * error;

    // integral
    integrator += error * dt;
    float iOut = ki * integrator;
    if (iOut > 0.5f) {
        iOut = 0.5f;
        integrator = iOut / ki;
    }
    else if (iOut < -0.5f) {
        iOut = -0.5f;
        integrator = iOut / ki;
    }

    // derivative
    float derivative = (dt > 0) ? (error - prevError) / dt : 0;
    float dOut = kd * derivative;

    prevError = error;
    float output = pOut + iOut + dOut;

    // max out:
    float maxOutput = 2 * 3.1415;
    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;
    output = output / (2 * 3.1415f);

    return output;
}

// FLight modes
void flyAcroMode(ControllerInput input, IMUstruct data, float dt) {
  // PID tuning
  static float rollInt = 0, rollPrevErr = 0;
  static float pitchInt = 0, pitchPrevErr = 0;
  static float yawInt = 0, yawPrevErr = 0;

  static float kp = 0.275f * input.S1; 
  static float ki = 0.002f * input.S2;
  static float kd = 0.005f;

  float max_radians_second = 3.1415 * (2.0 / 1.0);

  // Compute corrections for wished angular velocity
  // input (normalized), measured(rad/sec), dt, proportional, integrator, derivative, maximum_rotation (rad/sec), integrator, prevError
  float rollCorrection  = angularVelocityPID(input.roll, data.gyro.x, dt, kp, ki, kd, max_radians_second, rollInt, rollPrevErr); 
  float pitchCorrection = angularVelocityPID(-input.pitch, data.gyro.y, dt, kp, ki, kd, max_radians_second, pitchInt, pitchPrevErr);
  float yawCorrection = angularVelocityPID(input.yaw, data.gyro.z, dt, kp, ki, kd, max_radians_second, yawInt, yawPrevErr);

  // Debugg
  /*
  Serial.print("PID corrections: ");
  Serial.print(rollCorrection, 3);
  Serial.print(", ");
  Serial.print(pitchCorrection, 3);
  Serial.print(", ");
  Serial.print(yawCorrection, 3);
  Serial.println();
  Serial.print("Roll-stick: "); 
  Serial.print(input.roll);
  Serial.print(" | Correction: "); 
  Serial.println(rollCorrection, 4);
  Serial.print("Pitch-stick: "); 
  Serial.print(input.pitch);
  Serial.print(" | Correction: "); 
  Serial.println(pitchCorrection, 4);
  Serial.print("Tuning sticks - S1: ");
  Serial.print(input.S1, 5);
  Serial.print(" | S2: ");
  Serial.println(input.S2, 5);*/

  // Fly
  mixMotors(input.throttle, yawCorrection, pitchCorrection, rollCorrection);
}
void flyAngleMode(ControllerInput input, IMUstruct data, float dt) {
  killMotors(); //placeholder
}
void flyOtherMode() {
  killMotors(); //placeholderplaceholder
}
