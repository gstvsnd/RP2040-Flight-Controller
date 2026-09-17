// ---- RP2040 Flight Controller ----
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <AlfredoCRSF.h>
#include "VectorMath.h"
#include "IMU.h"
#include "RC_Controlls.h"
#include "FlightController.h"

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20
const int Battery_PIN = 28; // GP28

const int motor1_PIN = 4; // LB
const int motor2_PIN = 5; // RB
const int motor3_PIN = 6; // LF
const int motor4_PIN = 7; // RF

// ---- IMU constants (memory addresses) ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

int worthless_integer = 0;

// ---- RC Controller stuff ----
AlfredoCRSF crsf;

// ---- Arduino Setup and Loop ----
void setup() {
  Serial.begin(115200);
  
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(Battery_PIN, INPUT);
  pinMode(motor1_PIN, OUTPUT);
  pinMode(motor2_PIN, OUTPUT);
  pinMode(motor3_PIN, OUTPUT);
  pinMode(motor4_PIN, OUTPUT);
  digitalWrite(motor1_PIN, LOW);
  digitalWrite(motor2_PIN, LOW);
  digitalWrite(motor3_PIN, LOW);
  digitalWrite(motor4_PIN, LOW);

  delay(100); // Wait for hardware to stabilize

  writeRegister(PWR_MGMT_1, 0x00); // IMU

  Serial.println("--- Setup complete ---");
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Loop-time (dt) calculation
  unsigned long currentTime = micros();
  static unsigned long lastTime = 0;
  if (lastTime == 0) lastTime = currentTime; 
  float dt = (currentTime - lastTime) / 1000000.0f; // us to s
  lastTime = currentTime;

  // Timers
  unsigned long currentMillis = millis();
  static unsigned long lastPrintTime = 0;
  static unsigned long lastBlinkTime = 0;
  static bool ledBlinkState = false;

  // Read IMU data
  static IMUCalibrationData calibrationData;
  IMUstruct data = readIMU(calibrationData);
  if (worthless_integer == 0) {
    Serial.println("Calibrating IMU...");
    digitalWrite(LED_PIN, HIGH);
    delay(3000);
    
    callibIMU(calibrationData);

    Serial.println("Calibration Complete!\n");
    digitalWrite(LED_PIN, LOW);
    worthless_integer = 1;
  }

  float battery_voltage = analogRead(Battery_PIN) * (2.0f * (3.3f / 1023.0f)); // Voltage divider with equal resistors 1023 for 10-bit ADC (arduino analogRead returns 0-1023 for 0-3.3V)

  // Read RC controller input
  ControllerInput input = listen_channels(input, 3, 4, 2, 1, 6, 7, 8, 9, 5, 10, 11, 12);
  /* Controller Structure: 
  Sticks: throttle(0 to 1), yaw(-1 to 1), pitch(-1 to 1), roll(-1 to 1), 
  Switshes SA(0, 1, 2), SB(1, 2, 3), SC(0, 1, 2), SD(0, 1, 2), SE(0, 2), SF(0, 2), 
  Tuning Pots S1(0 to 1), S2(0 to 1)
  on my transmitter... (RM TX15)*/


  // ARM drone with SE switch
  if (input.SE == 2 && crsf.isLinkUp() && battery_voltage > 3.0f) {
    digitalWrite(LED_PIN, HIGH);
    // Decide Flight mode
    if (input.SD == 0) {
      flyAcroMode(input, data, dt);
    }
    if (input.SD == 1) {
      flyAngleMode(input, data, dt); //placeholder
    }
    if (input.SD == 2) {
      flyOtherMode(); //placeholderplaceholder
    }
    if (battery_voltage < 3.3f) { // Battery warning
      Serial.println("Battery low!");
      if (currentMillis - lastBlinkTime >= 300) {
        lastBlinkTime = currentMillis;
        ledBlinkState = !ledBlinkState;
        digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW); // magic
      }
    }
  }
  else if (battery_voltage < 3.0f) { // Battery warning
    killMotors();
    Serial.println("Battery critical!");
    if (currentMillis - lastBlinkTime >= 150) {
      lastBlinkTime = currentMillis;
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW); // magic
    }
  }
  else if (input.SF == 2) { // Callibrate IMU
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Calibrating IMU...");
    callibIMU(calibrationData);
    Serial.println("Calibration Complete!\n");
    digitalWrite(LED_PIN, LOW);
  }
  else {
    killMotors();
    digitalWrite(LED_PIN, LOW);
  }

  ///*
  if (currentMillis - lastPrintTime >= 2000) { // prints every 2 sec
    lastPrintTime = currentMillis;
    Serial.println("\nDebug data:");
    Serial.print("Gyro: ");
    Serial.print(data.gyro.x, 3);
    Serial.print(", ");
    Serial.print(data.gyro.y, 3);
    Serial.print(", ");
    Serial.print(data.gyro.z, 3);
    Serial.println();
  }//*/
}