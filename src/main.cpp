// ---- RP2040 Flight Controller ----
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "VectorMath.h"
#include "IMU.h"
#include "RC_Controlls.h"
#include <AlfredoCRSF.h>

// ---- Pin Definitions ----
const int CS_PIN = 17; // GP17
const int LED_PIN = 20; // GP20

const int motor1_PIN = 4; // LB
const int motor2_PIN = 5; // RB
const int motor3_PIN = 6; // LF
const int motor4_PIN = 7; // RF

// ---- IMU constants (memory addresses) ----
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;

int worthless_integer = 0;

// ---- PWM motor control ----
const uint32_t PWM_FREQUENCY = 30000; // Hz

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

  // Read RC controller input
  ControllerInput input = listen_channels(input, 1, 2, 3, 4, 7, 8, 9, 10, 5, 6);
  

  // PWM
  // 2. Om länk finns, koppla gaspinnen direkt till PWM-pinnen
  // rcData.throttle är mellan 0.0 och 1.0 baserat på din funktion
  int pwmValue = (int)(input.throttle * 255.0f);
  
  // Säkerställ att värdet håller sig mellan 0 och 255
  pwmValue = constrain(pwmValue, 0, 255);

  // Skicka ut PWM till motorn/MOSFETen
  analogWrite(motor1_PIN, pwmValue);
  analogWrite(motor2_PIN, pwmValue);
  analogWrite(motor3_PIN, pwmValue);
  analogWrite(motor4_PIN, pwmValue);
}