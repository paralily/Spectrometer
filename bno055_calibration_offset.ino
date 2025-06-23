#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Create BNO055 object with default I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void displayCalStatus() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("Calibration Status - Sys: ");
  Serial.print(system);
  Serial.print(" Gyro: ");
  Serial.print(gyro);
  Serial.print(" Accel: ");
  Serial.print(accel);
  Serial.print(" Mag: ");
  Serial.println(mag);
}

void printOffsets() {
  adafruit_bno055_offsets_t offsets;
  if (bno.getSensorOffsets(offsets)) {
    Serial.println("=== Sensor Offsets ===");
    Serial.print("Accel Offset X: "); Serial.println(offsets.accel_offset_x);
    Serial.print("Accel Offset Y: "); Serial.println(offsets.accel_offset_y);
    Serial.print("Accel Offset Z: "); Serial.println(offsets.accel_offset_z);

    Serial.print("Gyro Offset X: "); Serial.println(offsets.gyro_offset_x);
    Serial.print("Gyro Offset Y: "); Serial.println(offsets.gyro_offset_y);
    Serial.print("Gyro Offset Z: "); Serial.println(offsets.gyro_offset_z);

    Serial.print("Mag Offset X: "); Serial.println(offsets.mag_offset_x);
    Serial.print("Mag Offset Y: "); Serial.println(offsets.mag_offset_y);
    Serial.print("Mag Offset Z: "); Serial.println(offsets.mag_offset_z);

    Serial.print("Accel Radius: "); Serial.println(offsets.accel_radius);
    Serial.print("Mag Radius: "); Serial.println(offsets.mag_radius);
  } else {
    Serial.println("Failed to read sensor offsets.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BNO055 Offset Reader");

  // Initialize I2C for ESP8266 (adjust pins if needed)
  Wire.begin(D2, D1);  // SDA = GPIO4 (D2), SCL = GPIO5 (D1)

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1) delay(10);
  }

  delay(1000);
  bno.setExtCrystalUse(true);  // Use external crystal for better accuracy
}

void loop() {
  displayCalStatus();

  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // When fully calibrated, print offsets once and stop
  if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
    Serial.println("Sensor fully calibrated!");
    printOffsets();
    while (1) delay(1000);  // Halt here after printing offsets
  }

  delay(1000);
}
