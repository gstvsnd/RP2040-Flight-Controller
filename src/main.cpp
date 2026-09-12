// ---- RP2040 Flight Controller ----
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <AlfredoCRSF.h>
#include "VectorMath.h"
#include "IMU.h"

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20

const int motor1_PIN = 4; // LB
const int motor2_PIN = 5; // RB
const int motor3_PIN = 6; // LF
const int motor4_PIN = 7; // RF

AlfredoCRSF crsf;

// ---- IMU constants (memory addresses) ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

int worthless_integer = 0;

// ---- Visiting Functions ----
int getSwitchPosition(int channelValue) {
  if (channelValue < 1250) {
    return 0; // Läge ner
  } 
  else if (channelValue < 1750) {
    return 1; // Läge mitten
  } 
  else {
    return 2; // Läge upp
  }
}

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
  static unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  if (lastTime == 0) lastTime = currentTime; 
  float dt = (currentTime - lastTime) / 1000000.0f; // us to s
  lastTime = currentTime;

  // Make IMU offsets static to survive loop iterations
  static IMUCalibrationData calibrationData;
  if (worthless_integer == 0) {
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    Serial.print("Callibrating IMU...\n");
    callibIMU(calibrationData);

    worthless_integer = 1;
    Serial.print("--- Setup complete ---\n");
    digitalWrite(LED_PIN, LOW);
  }

  // Read IMU data
  IMUstruct data = readIMU(calibrationData);
  Vector3 accel = data.accel;
  Vector3 gyro = data.gyro;
  Vector3 orientation = IMUyprOrientation(accel, gyro, dt);
  Vector3 translation = IMUxyzTranslation(accel, gyro, dt); // INTE KLAR

  // Print accelerometer and gyroscope data to the serial monitor
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Gyro:  (%.4f,  %.4f,  %.4f)\n", gyro.x, gyro.y, gyro.z);
  //Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Accel: (%.4f,  %.4f,  %.4f)\n", accel.x, accel.y, accel.z);
  //Serial.print(buffer);
  snprintf(buffer, sizeof(buffer), "Pitch: %.4f   Roll: %.4f\n", orientation.y, orientation.z);
  //Serial.print(buffer);

  crsf.update();

  if (crsf.isLinkUp()) {
    // Radio channels definitions:
    int ch3 = crsf.getChannel(1); // Throttle
    int ch4 = crsf.getChannel(2); // Yaw
    int ch2 = crsf.getChannel(3); // Pitch
    int ch1 = crsf.getChannel(4); // Roll

    int ch5 = crsf.getChannel(5); // SE
    int ch6 = crsf.getChannel(6); // SF
    int ch7 = crsf.getChannel(7); // SA
    int ch8 = crsf.getChannel(8); // SB
    int ch9 = crsf.getChannel(9); // SC
    int ch10 = crsf.getChannel(10); // SD


    // Manual controll callibration:
    float throttle = ((float)ch3 / 1000.0f) - 1; // Normalize to [0, 1]
    float yaw = ((float)ch4 / 500.0f) - 3.0f; // Normalize to [-1, 1]
    float pitch = ((float)ch2 / 500.0f) - 3.0f; // Normalize to [-1, 1]
    float roll = ((float)ch1 / 500.0f) - 3.0f; // Normalize to [-1, 1]

    // Switch translation:
    int SA = getSwitchPosition(ch7);
    int SB = getSwitchPosition(ch8);
    int SC = getSwitchPosition(ch9);
    int SD = getSwitchPosition(ch10);
    int SE = getSwitchPosition(ch5);
    int SF = getSwitchPosition(ch6);

    Serial.print("Josticks: \n");
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print(" | Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.print(roll);

    Serial.print("Switches: \n");
    Serial.print("SA: ");
    Serial.print(SA);
    Serial.print(" | SB: ");
    Serial.print(SB);
    Serial.print(" | SC: ");
    Serial.print(SC);
    Serial.print(" | SD: ");
    Serial.print(SD);
    Serial.print(" | SE: ");
    Serial.print(SE);
    Serial.print(" | SF: ");
    Serial.print(SF);
  }
  else {
    Serial.println("No radiolink");
  }

  delay(500);
}