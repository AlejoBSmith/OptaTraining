/**
  Machine Learning for Motor Anomaly Detection with Nicla Sense ME & Opta
  Sends corrected acceleration, temperature, humidity, bVOC, and IAQ via BLE.
  Correction: Temperature +5°C, Humidity +20%
*/

#include <ArduinoBLE.h>
#include "Arduino_BHY2.h"
#include "sensors/SensorBSEC.h"

// BLE data: 3x acc + temp + hum + bVOC + IAQ (all int16_t) = 14 bytes
#define bufferSize 14

// BLE UUIDs (should match those in Opta code)
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// Sensor objects
SensorXYZ accel(SENSOR_ID_ACC);
SensorBSEC bsec(SENSOR_ID_BSEC);

// Correction offsets
const float temp_correction = -3.0;     // Sensor reads +5°C high, so subtract 5
const float humidity_correction = 20.0; // Sensor reads -20%, so add 20

// BLE Service and Characteristic
BLEService niclaService(deviceServiceUuid);
BLECharacteristic sensorCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLENotify, bufferSize);

// Transmission interval
const long interval = 100; // ms (10 Hz)

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Init sensors
  if (!BHY2.begin()) {
    Serial.println("Failed to initialize BHY2!");
    while (1);
  }
  if (!accel.begin()) {
    Serial.println("Failed to initialize accelerometer!");
    while (1);
  }
  if (!bsec.begin()) {
    Serial.println("Failed to initialize BSEC sensor!");
    while (1);
  }

  // Init BLE
  if (!BLE.begin()) {
    Serial.println("Failed to start Bluetooth!");
    while (1);
  }
  BLE.setDeviceName("NICLA");
  BLE.setAdvertisedService(niclaService);
  niclaService.addCharacteristic(sensorCharacteristic);
  BLE.addService(niclaService);
  BLE.advertise();

  Serial.println("Bluetooth initialized and waiting for a connection...");
}

void loop() {
  BLEDevice central = BLE.central();
  Serial.println("Searching for central device...");

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      static unsigned long lastUpdateTime = millis();
      BHY2.update();

      if (millis() - lastUpdateTime >= interval) {
        lastUpdateTime = millis();

        // --- Get sensor data ---
        int16_t accX = accel.x();
        int16_t accY = accel.y();
        int16_t accZ = accel.z();

        // Apply corrections here
        float temperature = bsec.comp_t() + temp_correction;       // Compensated, then corrected
        float humidity    = bsec.comp_h() + humidity_correction;   // Compensated, then corrected
        float bvoc        = bsec.b_voc_eq();  // ppm
        float iaq         = bsec.iaq();       // 0-500

        // Convert to int16 for BLE
        int16_t temp_int = (int16_t)(temperature * 100); // °C x100
        int16_t hum_int  = (int16_t)(humidity * 100);    // % x100
        int16_t bvoc_int = (int16_t)(bvoc * 100);        // ppm x100
        int16_t iaq_int  = (int16_t)(iaq * 10);          // index x10

        // Debug output (shows *corrected* values)
        Serial.print("Accel: ");
        Serial.print(accX); Serial.print(", ");
        Serial.print(accY); Serial.print(", ");
        Serial.print(accZ); Serial.print(" | Temp: ");
        Serial.print(temperature); Serial.print(" C | Hum: ");
        Serial.print(humidity); Serial.print(" % | bVOC: ");
        Serial.print(bvoc); Serial.print(" ppm | IAQ: ");
        Serial.println(iaq);

        // --- Pack data into buffer ---
        byte data[bufferSize];
        memcpy(data,      &accX,     sizeof(accX));
        memcpy(data + 2,  &accY,     sizeof(accY));
        memcpy(data + 4,  &accZ,     sizeof(accZ));
        memcpy(data + 6,  &temp_int, sizeof(temp_int));
        memcpy(data + 8,  &hum_int,  sizeof(hum_int));
        memcpy(data +10,  &bvoc_int, sizeof(bvoc_int));
        memcpy(data +12,  &iaq_int,  sizeof(iaq_int));

        // --- Send data via BLE ---
        sensorCharacteristic.writeValue(data, sizeof(data));
      }
    }
    Serial.println("Disconnected from central device!");
  }
  delay(100);
}
