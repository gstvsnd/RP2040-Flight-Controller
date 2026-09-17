// FlightController.h
#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "RC_Controlls.h"
#include "IMU.h"

// ---- PWM motor control ----
const uint32_t PWM_FREQUENCY = 20000; // Hz

//---- Functions ----
void killMotors();
void mixMotors(float throttle, float yaw, float pitch, float roll);
float angularVelocityPID(float setpoint, float measured, float dt, float kp, float ki, float kd, float max_radians_second, float &integrator, float &prevError);
 
void flyAcroMode(ControllerInput input, IMUstruct data, float dt);
void flyAngleMode(ControllerInput input, IMUstruct data, float dt);
void flyOtherMode(); //placeholder

#endif // FLIGHTCONTROLLER_H