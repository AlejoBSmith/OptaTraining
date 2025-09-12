/**
  Machine Learning for Motor Anomaly Detection with Nicla Sense ME & Opta
  Modified to send results via UDP instead of Arduino Cloud.
*/

#include <ArduinoBLE.h>               // BLE functionality
#include <Ethernet.h>                 // Ethernet
#include <EthernetUdp.h>              // UDP
#include <OptaMotorDC_inferencing.h>  // Edge Impulse inference

// ---------- BLE ----------
static const uint8_t bufferSize = 14;  // 7x int16_t from Nicla
const char* serviceUuid        = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// ---------- Network ----------
byte mac[] = { 0x02, 0x12, 0x34, 0x56, 0x78, 0x9A }; // any locally unique MAC
IPAddress PC_IP(10, 100, 32, 166);                   // <-- set this to your PC/router IP
const uint16_t PC_PORT    = 5005;                    // destination UDP port
const uint16_t LOCAL_PORT = 5006;                    // Opta local port
EthernetUDP Udp;

const char* DEVICE_ID = "optaA";
static uint32_t seq = 0;

// ---------- Inference ----------
static bool debug_nn = false;  // set true for verbose EI debug
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

// threshold from original code
#define ANOMALY_TH 1500

// Local state
float anomaly_score = 0.0f;
bool fault = false;

// ---------- Functions ----------
void inference_run() {
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal (%d)\n", err);
    return;
  }

  ei_impulse_result_t result = { 0 };
  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: run_classifier (%d)\n", err);
    return;
  }

  anomaly_score = result.anomaly;

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("Anomaly score: %.3f\n", anomaly_score);
#endif

  // fault flag (inverted as in original)
  if (anomaly_score > ANOMALY_TH) {
    fault = false;
  } else {
    fault = true;
  }
}

// Handle connection to Nicla
void NiclaHandler(BLEDevice peripheral) {
  Serial.println("Connecting ...");
  if (!peripheral.connect()) {
    Serial.println("Failed to connect!");
    return;
  }
  Serial.println("Connected!");

  if (!peripheral.discoverAttributes()) {
    Serial.println("Failed to discover attributes!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Attributes discovered.");

  BLEService niclaService = peripheral.service(serviceUuid);
  BLECharacteristic sensorCharacteristic = niclaService.characteristic(characteristicUuid);

  if (!sensorCharacteristic || !sensorCharacteristic.canSubscribe() || !sensorCharacteristic.subscribe()) {
    Serial.println("Failed to subscribe characteristic!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to Nicla characteristic.");

  // buffers
  uint8_t data[bufferSize];
  int16_t accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10;

  while (peripheral.connected()) {
    BLE.poll();

    // Drain backlog
    bool got = false;
    while (sensorCharacteristic.valueUpdated()) {
      int bytes = sensorCharacteristic.readValue(data, sizeof(data));
      if (bytes == bufferSize) got = true;
      BLE.poll();
    }

    if (!got) continue;

    // Parse values
    memcpy(&accX,      data + 0, 2);
    memcpy(&accY,      data + 2, 2);
    memcpy(&accZ,      data + 4, 2);
    memcpy(&temp_x100, data + 6, 2);
    memcpy(&hum_x100,  data + 8, 2);
    memcpy(&bvoc_x100, data +10, 2);
    memcpy(&iaq_x10,   data +12, 2);

    // Fill buffer for EI (only accelerometer goes in)
    static int samples = 0;
    buffer[samples + 0] = accX;
    buffer[samples + 1] = accY;
    buffer[samples + 2] = accZ;
    samples += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

    if (samples >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      samples = 0;
      inference_run();

      // Build CSV payload
      unsigned long ts_ms = millis();
      char payload[200];
      int n = snprintf(payload, sizeof(payload),
                       "%s,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%.3f,%d\n",
                       DEVICE_ID, (unsigned long)seq++, ts_ms,
                       accX, accY, accZ,
                       temp_x100, hum_x100, bvoc_x100, iaq_x10,
                       anomaly_score, fault ? 1 : 0);

      if (n > 0) {
        Udp.beginPacket(PC_IP, PC_PORT);
        Udp.write((const uint8_t*)payload, (size_t)n);
        Udp.endPacket();
      }
    }
  }

  Serial.println("Peripheral disconnected.");
  peripheral.disconnect();
}

// ---------- Arduino setup/loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // Ethernet setup
  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    IPAddress ip(192,168,143,36), gw(192,168,143,33), mask(255,255,255,224), dns(192,168,18,2);
    Ethernet.begin(mac, ip, dns, gw, mask);
  }
  Serial.print("IP: ");      Serial.println(Ethernet.localIP());
  Serial.print("Gateway: "); Serial.println(Ethernet.gatewayIP());
  Udp.begin(LOCAL_PORT);

  // BLE setup
  if (!BLE.begin()) {
    Serial.println("Failed to init BLE!");
    while (1);
  }
  Serial.println("BLE initialized.");

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: Model expects 3 axes\n");
    while (1);
  }

  BLE.scanForUuid(serviceUuid);
  Serial.println("Scanning for BLE devices...");
}

void loop() {
  // toggle heartbeat LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  BLEDevice peripheral = BLE.available();
  if (peripheral && peripheral.localName() == "NICLA") {
    BLE.stopScan();
    NiclaHandler(peripheral);
    BLE.scanForUuid(serviceUuid);  // resume scanning
  }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
