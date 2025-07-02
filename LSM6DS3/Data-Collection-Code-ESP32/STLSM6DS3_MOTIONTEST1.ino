// STLSM6DS3 6-DoF IMU initial motion data sampling code
// Made by Isaac Medina
// Date: 24 June 2025
// Purpose: This code collects data at ~60 Hz from the IMU during motion.
//          Motion is sampled for 3-5 seconds and then copy/pasted to CSV.


// Include libraries:
#include <Wire.h>

// IMU i2c addresses
#define LSM6DS3_ADDR  0x6A
#define OUTX_L_XL     0x28
#define CTRL1_XL      0x10
#define CTRL2_G       0x11
#define OUTX_L_G      0x22

// writeRegister
// Input: An 8-bit unsigned register address (listed above), and an 8-bit unsigned value for that register.
// Desciption: Writes a value to a register on the LSM6DS3 IMU.
// Output: Void function -> no epxlicit output. However, it updates register values.
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// read16bitRegister
// Input: An 8-bit unsigned register address.
// Description: Reads the value of the address in question.
// Output: Returns the 16-bit signed integer value in the input register.
int16_t read16bitRegister(uint8_t reg) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDR, 2);

  int16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}

void setup() {

  // Configure serial monitor and I2C
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  delay(1000);

  // Configure accelerometer: 104Hz, ±2g, BW = 50Hz
  writeRegister(CTRL1_XL, 0x40);
  delay(1000);

  // Configure gyroscope: 104Hz, ±245 dps
  writeRegister(CTRL2_G, 0x40);
  delay(1000);

}

// Main data collection loop
void loop() {

  // Declare acceleration and gyroscope data variables
  // Accelerometer:
  int16_t ax = read16bitRegister(OUTX_L_XL);
  int16_t ay = read16bitRegister(OUTX_L_XL + 2);
  int16_t az = read16bitRegister(OUTX_L_XL + 4);

  // Gyroscope:
  int16_t gx = read16bitRegister(OUTX_L_G);
  int16_t gy = read16bitRegister(OUTX_L_G + 2);
  int16_t gz = read16bitRegister(OUTX_L_G + 4);

  // Print the data in the desired format for the CSV file
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); Serial.print(", ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az); 

  // ~60 Hz sampling rate
  delay(16.67);

}
