// FlightController.h
#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "RadioController.h"
#include "IMU.h"

// ---- PWM motor control ----
const uint32_t PWM_FREQUENCY = 20000; // Hz

//---- Functions ----
void killMotors();
float angularVelocityPID(float setpoint, float measured, float dt, float kp, float ki, float kd, float max_radians_second, float &integrator, float &prevError);
void mixMotors(float throttle, float yaw, float pitch, float roll);
void runMotors(float m1, float m2, float m3, float m4); 


void flyAcroMode(ControllerInput input, IMUstruct data, float dt);
void flyAngleMode(ControllerInput input, IMUstruct data, float dt);
void flyOtherMode(); //placeholder

#endif // FLIGHTCONTROLLER_H