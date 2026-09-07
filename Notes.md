# Raspberry-Pi-Pico-Flight-Controller drone
Nu ska vi försöka bygga en drönare med en egen flight controller <!-- Jättekul! -->

## Project idea (Updated 2026-08-31)
A small radio controlled quadrocopter drone based of a Paspberry Pi Pico running the flight controller. A IMU "Inertial Measurement Unit" accounts for spatial context and the motors is controlled by PWM "Pulse Width Modulation" in a PID loop for stable flight. The rest will just work out.

> WIP - Work In Progress

> TBC - To Be Continued

> TBD - To Be Done

> IGU - I Gave Up


## Features
### Orientational Awareness
    Roll/Pitch Estimation - WIP
    * IMU Sensor Readings - Done  
    * Accelerometer/Gyroscope Complimentary Filter (Pitch/Roll) - Done
### Rotational Callibration
    Rotational callibration matrix - TBD
### Stable Flight
    PID Controlls - TBD
### Remote Controll
    ExpressLRS Radio Communication - TBD

## Hardware (BOM-ish)
### MCU
* Raspberry Pi Pico 2040 - TBC
### IMU
* MPU 6500/9250/9255 - TBC <!-- Det är vad det står på sensorn... -->
### Motors 
* 4x8520 brushed motors - TBC
### Motor controlls
* 4x 2N2222A PNP Transistors for PWM - TBC
### Battery
* 400mah LiPo battery (from a vape I found on the golf course) - TBD
### Radio Transmitter 
* ExpressLRS 2.4GHz - TBD
### Quadrocopter Frame
* 3D printed model - TBD
