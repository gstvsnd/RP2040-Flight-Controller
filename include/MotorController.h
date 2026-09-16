// MotorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

// ---- Hardware Pins ----

// ---- PWM motor control ----
const uint32_t PWM_FREQUENCY = 20000; // Hz

//---- Functions ----
void mixMotors(float throttle, float yaw, float pitch, float roll);
void killMotors();
float computePID(float setpoint, float measured, float dt, float kp, float ki, float kd, float maxOutput, float &integrator, float &prevError);


#endif // MOTORCONTROLLER_H