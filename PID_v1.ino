// Include AS5600 and Wire libraries
#include "AS5600.h"
#include <Wire.h>

// Define I2C address for AS5600
#define AS5600_ADDRESS 0x36

// Create AS5600 encoder object
AS5600 encoder;

// Define motor control pins
const int PulsePinPlus = 9;        // Pulse Pin for stepper motor
const int DirectionPinPlus = 8;    // Direction Pin for stepper motor
const int fullRevolution = 400;    // Number of pulses for one full revolution

// Function to rotate the motor by one full revolution
void rotation() {
  for (int i = 0; i < fullRevolution; i++) {
    digitalWrite(PulsePinPlus, HIGH);
    delay(1);
    digitalWrite(PulsePinPlus, LOW);
    delay(1);
  }
}

// Function to rotate the motor by a custom number of steps (based on 'rev')
void customRotation(float rev) {
  for (int i = 0; i < int(rev); i++) {
    digitalWrite(PulsePinPlus, HIGH);
    delay(1);
    digitalWrite(PulsePinPlus, LOW);
    delay(1);
  }
}

void setup() {
  delay(1000); // This is because the encoder takes time to start and because of that the motor rotates extra.

  // Initialize motor control pins
  pinMode(PulsePinPlus, OUTPUT);
  pinMode(DirectionPinPlus, OUTPUT);

  // Initialize Serial and I2C communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize the AS5600 encoder
  if (!encoder.begin()) {
    Serial.println("Check connections.");
    while (1); // Halt if encoder not found
  }

  Serial.println("AS5600 initialized.");

  // Set motor direction to anticlockwise
  digitalWrite(DirectionPinPlus, HIGH);

  // Read initial angle from encoder
  float angle1 = encoder.readAngle();
  Serial.print("Angle: ");
  Serial.print(angle1 * 360.0 / 4096.0);
  Serial.println(" degrees");
  delay(100); // Delay for stability

  // Perform full rotation
  rotation();

  // Read angle after rotation
  float angle2 = encoder.readAngle();
  Serial.print("Angle: ");
  Serial.print(angle2 * 360.0 / 4096.0);
  Serial.println(" degrees");
  delay(100); // Delay for stability

  // If angle has changed, perform correction
  if (angle1 != angle2) {
    float rev = (angle2 * 360.0 / 4096.0) - (angle1 * 360.0 / 4096.0);
    Serial.print(rev);
    customRotation(int(rev));
  }

  // Read final angle after correction
  float angle3 = encoder.readAngle();
  Serial.print("Angle: ");
  Serial.print(angle3 * 360.0 / 4096.0);
  Serial.println(" degrees");
  delay(100); // Final delay
}

void loop() {
  // Main loop left empty
}
