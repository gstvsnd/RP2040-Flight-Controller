# Raspberry-Pi-Pico-Flight-Controller drone
Nu ska vi försöka bygga en drönare med en egen flight controller <!-- Jättekul! -->

## Project idea (Updated 2026-08-31)
A small radio controlled quadrocopter drone based of a Paspberry Pi Pico running the flight controller. A IMU "Inertial Measurement Unit" accounts for spatial context and the motors is controlled by PWM "Pulse Width Modulation" in a PID loop for stable flight. The rest will just work out.

Coordinate system: (x, y, z) <=> (North, East, Down) - directions, (roll, pitch yaw) - rotations
General units: SI units & radians
Rotational speed unit: radians/second
Acceleration unit: meters/second*2

> WIP - Work In Progress

> TBC - To Be Continued

> TBD - To Be Done

> TND - To Not Do

> Done - Works (But can be improved)

## Main Features
### Orientational Awareness
    IMU Sensor Readings - Done
    Roll/Pitch Estimation - Done
    Accelerometer/Gyroscope Complimentary Filter - Done
### Callibration
    Rotational callibration matrix - Done
### Stable Flight
    Motor control (PWM) - Done
    PID Controlls - TBC
### Flight Modes
    Acro mode - TBC
    Angle mode - TBD
### Remote Controll
    ExpressLRS Radio Communication - Done

## Main Hardware (BOM-ish)
### MCU
* Raspberry Pi Pico 2040
### IMU
* MPU 6500/9250/9255 - TBC <!-- Det är vad det står på sensorn... -->
### Motors 
* 4x8520 brushed motors
### Motor controlls
* 4x AO3400A N-Channel Mosfets for PWM
* 4x 100ohm resistors (serial with PWM signals)
* 4x 4.6kohm resistors pull-down
* 4x SS34 Flyback-Diodes
### Battery
<!-- * 400mah LiPo battery (from a vape I found on the golf course) - TBD: Battery delivers less current than needed to give full gas and supply PICO. -->
* 680mah 95C (high current) 1S LiPo
### Radio Transmitter 
* ExpressLRS 2.4GHz reciever
### Quadrocopter Frame
* 3D printed PETG model
