/**
 * @file tle493d_w2b6_corrected.cpp
 * @brief Corrected example code for the Infineon TLE493D-W2B6 (A0) 3D Magnetic Sensor on an ESP8266.
 *
 */

#include <Wire.h>
#include "TLx493D_inc.hpp" // From the Infineon TLx493D library

using namespace ifx::tlx493d;

// ESP8266 pins (e.g., for a Wemos D1 Mini or NodeMCU)
const uint8_t SDA_PIN = 4; // D2 This was only for 
const uint8_t SCL_PIN = 5; // D1

TLx493D_W2B6 *mag = nullptr;

void setup() {
  Serial.begin(115200);
  // Add a small delay to allow the serial monitor to connect.
  delay(500);
  Serial.println("\n--- TLE493D-W2B6 Sensor Initializing ---");

  // Initialize the I2C bus with custom ESP8266 pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Set I2C to 400 kHz (Fast Mode)

  // --- FIX 1 (Continued): Instantiate the sensor object NOW ---
  // Now that Wire is initialized, we can safely create the sensor object.
  mag = new TLx493D_W2B6(Wire, TLx493D_IIC_ADDR_A0_e);

  // --- FIX 2: Initialize sensor with interrupts disabled for stability ---
  // The 'true' argument disables interrupts during initialization, which is
  // recommended for this sensor generation to prevent I2C bus stalls.
  if (!mag->begin(true)) {
    Serial.println("Error: Sensor initialization failed!");
    Serial.println("Please check your wiring and the I2C address.");
    // Halt execution if the sensor can't be found.
    while (1) {
      delay(1000);
    }
  }

  // Set the magnetic field measurement range to the full ±160 mT range.
  mag->setSensitivity(TLx493D_FULL_RANGE_e);

  Serial.println("Sensor TLE493D-W2B6 (A1) is ready.");
  Serial.println("------------------------------------");
}

void loop() {
  double bx, by, bz;

  // The getMagneticField function triggers a measurement and reads the result.
  // We use the pointer access operator '->' instead of '.'
  if (mag->getMagneticField(&bx, &by, &bz)) {
    Serial.print("Bx: ");
    Serial.print(bx, 3); // Print with 3 decimal places
    Serial.print(" mT  |  ");
    Serial.print("By: ");
    Serial.print(by, 3);
    Serial.print(" mT  |  ");
    Serial.print("Bz: ");
    Serial.print(bz, 3);
    Serial.println(" mT");
  } else {
    // This error indicates a problem during an ongoing read operation.
    Serial.println("Error: Failed to read from sensor during loop.");
  }

  // Wait before the next reading
  delay(200);
}
