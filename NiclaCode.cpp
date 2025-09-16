#include <Arduino.h>
#include <Arduino_BHY2.h>
#include "sensors/SensorBSEC.h"
#include <OptaMotorDCTraining_inferencing.h>
#include <ArduinoBLE.h>   // BLE

// Identificador
const char* DEVICE_ID = "Nicla_BH1";

// UUIDs (deben coincidir con el Opta)
const char* serviceUuid        = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// BLE Service & Characteristic
BLEService niclaService(serviceUuid);
BLECharacteristic sensorCharacteristic(characteristicUuid, BLERead | BLENotify, 32);

// Sensores
SensorXYZ accel(SENSOR_ID_ACC);
SensorBSEC bsec(SENSOR_ID_BSEC);

// Correcciones
const float temp_correction = 0;
const float humidity_correction = 0;

// Buffer para inferencia
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

// Control de tiempo
unsigned long lastSample = 0;
const unsigned long sampleInterval = 100; // 10 Hz
unsigned long sampleTime_ms = 0;

// --- Paquete compacto ---
struct Packet {
  uint32_t sampleTime; 
  int16_t accXRMS;
  int16_t accYRMS;
  int16_t accZRMS;
  int16_t temp;
  int16_t hum;
  int16_t bvoc;
  int16_t iaq;
  int32_t anomaly;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BHY2.begin(NICLA_STANDALONE)) {
    Serial.println("Failed to init BHY2!");
    while (1);
  }
  if (!accel.begin()) {
    Serial.println("Failed to init accelerometer!");
    while (1);
  }
  if (!bsec.begin()) {
    Serial.println("Failed to init BSEC!");
    while (1);
  }

  // --- Init BLE ---
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1);
  }
  BLE.setDeviceName("NICLA");
  BLE.setLocalName("NICLA");
  BLE.setAdvertisedService(niclaService);
  niclaService.addCharacteristic(sensorCharacteristic);
  BLE.addService(niclaService);
  BLE.advertise();

  Serial.println("BLE initialized, advertising started");
}

void loop() {
  // Esperar a que un central se conecte
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // Mientras el Opta siga conectado
    while (central.connected()) {
      int samples = 0;

      // --- Recolectar ventana de datos ---
      while (samples < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        BHY2.update();

        if (accel.dataAvailable()) {
          unsigned long now = millis();
          if (now - lastSample >= sampleInterval) {
            sampleTime_ms = now - lastSample;
            lastSample = now;

            buffer[samples + 0] = accel.x();
            buffer[samples + 1] = accel.y();
            buffer[samples + 2] = accel.z();

            samples += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
          }
        }
      }

      // --- Inferencia ---
      signal_t signal;
      if (numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal) != 0) continue;

      ei_impulse_result_t result = { 0 };
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
      if (res != EI_IMPULSE_OK) continue;

      float anomaly_f = result.anomaly;           
      int32_t anomaly_int = (int32_t)(anomaly_f * 1000.0f);

      // --- Calcular RMS por eje ---
      float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
      int numSamples = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
      for (int i = 0; i < numSamples; i++) {
        float x = buffer[i * 3 + 0];
        float y = buffer[i * 3 + 1];
        float z = buffer[i * 3 + 2];
        sumX += x * x;
        sumY += y * y;
        sumZ += z * z;
      }
      float rmsX = sqrt(sumX / numSamples);
      float rmsY = sqrt(sumY / numSamples);
      float rmsZ = sqrt(sumZ / numSamples);

      int16_t rmsX_int = (int16_t)rmsX;
      int16_t rmsY_int = (int16_t)rmsY;
      int16_t rmsZ_int = (int16_t)rmsZ;

      // --- Leer sensores ---
      float temperature = bsec.comp_t() + temp_correction;
      float humidity    = bsec.comp_h() + humidity_correction;
      float bvoc        = bsec.b_voc_eq();  
      float iaq         = bsec.iaq();       

      int16_t temp_int = (int16_t)(temperature * 100);
      int16_t hum_int  = (int16_t)(humidity * 100);
      int16_t bvoc_int = (int16_t)(bvoc * 100);
      int16_t iaq_int  = (int16_t)(iaq * 10);

      // --- Crear paquete ---
      Packet pkt;
      pkt.sampleTime = sampleTime_ms;
      pkt.accXRMS = rmsX_int;
      pkt.accYRMS = rmsY_int;
      pkt.accZRMS = rmsZ_int;
      pkt.temp = temp_int;
      pkt.hum = hum_int;
      pkt.bvoc = bvoc_int;
      pkt.iaq = iaq_int;
      pkt.anomaly = anomaly_int;

      // --- Debug Serial ---
      Serial.print(DEVICE_ID); Serial.print(", ");
      Serial.print(pkt.sampleTime); Serial.print(", ");
      Serial.print(pkt.accXRMS); Serial.print(", ");
      Serial.print(pkt.accYRMS); Serial.print(", ");
      Serial.print(pkt.accZRMS); Serial.print(", ");
      Serial.print(pkt.temp); Serial.print(", ");
      Serial.print(pkt.hum); Serial.print(", ");
      Serial.print(pkt.bvoc); Serial.print(", ");
      Serial.print(pkt.iaq); Serial.print(", ");
      Serial.println(pkt.anomaly);

      // --- Enviar por BLE ---
      sensorCharacteristic.writeValue((uint8_t*)&pkt, sizeof(pkt));
    }

    Serial.println("Central disconnected");
  }
}
