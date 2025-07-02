/* BNO055 IMU initial calibration and data collection code
*  Made by Isaac Medina
*  Date: 25 June 2025
*  Purpose: Initialy callibration and data collection testing. 
*           This code utilizes Adafruit's BNO055 library for initial testing. 
*           Eventually, for the custom PCB, need to integrate a custom library for programming the nRF52832.
*           SRL 2025
*/

// Include libraries:
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// I2C device address. Addr can be 0x28 or 0x29, switch value if fail on test.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {

  // Initialize serial monitor
  Serial.begin(115200);
  delay(1000);

  // Initialize wire for i2c
  Wire.begin();
  delay(1000);

  // Initialize the sensor
  if(!bno.begin()){

    // There was a problem detecting the IMU
    Serial.print("There was an issue connecting to the BNO055, check wiring or I2C address...");
    while(1);
  }
  else if(bno.begin()){
    Serial.print("BNO055 found!");
  }

  delay(1000);
  bno.setExtCrystalUse(true); // Use external crystal for better timing accuracy
  bno.setMode(OPERATION_MODE_NDOF); // 9-DoF mode with magnetometer callibration


  // Callibration format and prompts
  Serial.println("==== BNO055 Calibration Required ====");
  Serial.println(" - Accelerometer: 6 stable orientations with slight pitch (±X, ±Y, ±Z)");
  Serial.println(" - Gyroscope: flat and still");
  Serial.println(" - Magnetometer: figure-8 motions in air");
  Serial.println();
  Serial.println("Calibration progress: SYS, GYR, ACC, MAG");

}


void loop() {
 uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CALIB:");
  Serial.print(sys); Serial.print(", ");
  Serial.print(gyro); Serial.print(", ");
  Serial.print(accel); Serial.print(", ");
  Serial.println(mag);

  // Once fully calibrated, begin logging data
  if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
    Serial.println("=== Fully Calibrated! ===");
    Serial.println("Data capture beginning soon...");
    delay(1000);  // pause before logging

    Serial.println("timestamp_us, acc_x, acc_y, acc_z, quat_w, quat_x, quat_y, quat_z"); // Data output format
    delay(1000); // pause before logging

    // Begin data collection loop after calibration is complete
    while (1) {
      logData();
    }
  }
  delay(500);
}


// logData
// Input: None.
// Description: Captures the data from the BNO055 IMU with timestamps
// Output: Outputs linear acceleration and quaternion data to the serial terminal
void logData(){
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= 10000){ // ~100 Hz = sample rate of BNO055
    lastMicros = now;

    // Read linear acceleration and quaternion
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = bno.getQuat();

    // Output for CSV
    Serial.print(now); Serial.print(", "); // Print timestamp

    // Linear acceleration
    Serial.print(acc.x(), 6); Serial.print(", ");
    Serial.print(acc.y(), 6); Serial.print(", ");
    Serial.print(acc.z(), 6); Serial.print(", ");

    // Quaternion
    Serial.print(quat.w(), 6); Serial.print(", ");
    Serial.print(quat.x(), 6); Serial.print(", ");
    Serial.print(quat.y(), 6); Serial.print(", ");
    Serial.println(quat.z(), 6); 
  }
}
