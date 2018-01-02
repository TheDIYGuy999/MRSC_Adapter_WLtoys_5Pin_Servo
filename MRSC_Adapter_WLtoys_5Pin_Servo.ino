/* Standalone MRSC "Micro Rc Stability Control" converter for 5pin WLtoys steering servo
    MPU: Atmega 32U4 3.3V, 8MHz for test or 328P 3.3V, 8MHz for vehicle
    Board: Pro Micro or Pro Mini
    MPU-6050 board: GY-521

   Pins:
   - Steering input (red wire from Servo) A0
   - Steering output (red wire to 5 pin servo connector on the Receiver, via RC-filter 10kOhm, 470nF) 10
   - MPU-6050 SDA 2 (A4 on Pro Mini)
   - MPU-6050 SCL 3 (A5 on Pro Mini)

   Supply pins:
   - GND spliced into black servo wire
   - VCC (3.3V) spliced into white servo wire

*/

const float codeVersion = 0.2; // Software revision

//
// =======================================================================================================
// LIBRARIES
// =======================================================================================================
//

#include <Wire.h> // I2C library (for the MPU-6050 gyro /accelerometer)
#include <PWMFrequency.h> // https://github.com/TheDIYGuy999/PWMFrequency

#include "mpu.h" // MPU-6050 handling (.h file in sketch folder)

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// MRSC gain
byte mrscGain = 20; // Gain in %

// Configuration
boolean mpuInversed = true;

// Pin definition (don't change servo imputs, interrupt routine is hardcoded)
#define INPUT_STEERING A0

#define OUTPUT_STEERING 10

#define GAIN_POT A2
#define INVERSE_MPU_DIRECTION 9

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  // Configure inputs
  pinMode(INVERSE_MPU_DIRECTION, INPUT_PULLUP);
  pinMode(GAIN_POT, INPUT);
  pinMode(INPUT_STEERING, INPUT);

  // Configure outputs
  pinMode(OUTPUT_STEERING, OUTPUT);

  // PWM frequency: 32 = 984Hz (default), 8 = 3936Hz, 1 = 31488Hz
  setPWMPrescaler(OUTPUT_STEERING, 1);

  // Center servo before MPU-6050 calibration, which takes time
  analogWrite(OUTPUT_STEERING, 142); // 127 in theory. Adjust, until your servo does not move during MPU-6050 initialization!!

  // MPU 6050 accelerometer / gyro setup
  setupMpu6050();
}

//
// =======================================================================================================
// READ INPUTS
// =======================================================================================================
//

void readInputs() {
  mrscGain = map(analogRead(GAIN_POT), 0, 255, 0, 100);
  mpuInversed = !digitalRead(INVERSE_MPU_DIRECTION);
}

//
// =======================================================================================================
// MRSC (MICRO RC STABILITY CONTROL) CALCULATIONS
// =======================================================================================================
// For cars with stability control (steering overlay depending on gyro yaw rate)

void mrsc() {

  int steeringAngle;

  // Read sensor data
  readMpu6050Data();

  // Compute steering compensation overlay
  int turnRateSetPoint = map(analogRead(INPUT_STEERING), 0, 1023, -255, 255);  // turnRateSetPoint = servo pot. angle (0 to 1023) = -50 to 50
  int turnRateMeasured = (yaw_rate * 127) * mrscGain / 100; // degrees/s * speed (we have no speed input, so just 25% speed = 127)
  turnRateMeasured = constrain (turnRateMeasured, -75, 75); // Limit turn rate (adjust to match your max. steering throw)
  if (mpuInversed) steeringAngle = turnRateSetPoint - turnRateMeasured;  // Compensation of steering angle
  else steeringAngle = turnRateSetPoint + turnRateMeasured;

  // Control steering servo
  steeringAngle = map(steeringAngle, -255, 255, 0, 255);
  analogWrite(OUTPUT_STEERING, steeringAngle);
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {
  //readInputs(); // Read pots and switches
  mrsc(); // Do stability control calculations
}

