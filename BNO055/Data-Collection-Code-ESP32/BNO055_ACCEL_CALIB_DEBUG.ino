/* BNO055 Accelerometer Calibration Debugging
*  Made by Isaac Medina
*  Date: 26 June 2025
*/

// Include libraries
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// I2C device address. Addr can be 0x28 or 0x29, switch value if fail on test.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void setup() {

  // Initialize serial monitor
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C
  Wire.begin();
  delay(1000);

  // Initialize the BNO055
  if (!bno.begin()) {
    Serial.println("Could not find BNO055! Check wiring and I2C address.");
    while (1);
  }

  bno.setExtCrystalUse(true); // Use external crystal for better timing accuracy
  bno.setMode(OPERATION_MODE_NDOF); // Use full fusion with accel+gyro+mag
  delay(100);
  Serial.println("Begin accelerometer calibration. Hold sensor in 6 distinct, stable orientations.");
  Serial.println("Place flat, then on each side (±X, ±Y, ±Z), holding each for 2-3 seconds.");
  Serial.println("===============================================================");
}

void loop() {
  // Get current calibration levels
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // Get raw accelerometer data
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Print readings and calibration state
  Serial.print("Accel: ");
  Serial.print(acc.x(), 2); Serial.print(", ");
  Serial.print(acc.y(), 2); Serial.print(", ");
  Serial.print(acc.z(), 2); Serial.print(" [m/s^2]   ");

  Serial.print(" | ACC Calib: "); Serial.print(accel);
  Serial.print(" | SYS: "); Serial.print(sys);
  Serial.print(" | GYR: "); Serial.print(gyro);
  Serial.print(" | MAG: "); Serial.println(mag);

  delay(250);
}
