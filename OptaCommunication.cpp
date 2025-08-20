/** 
  Opta firmware — BLE (Nicla Sense ME) -> UDP (LAN)
  Expects 14-byte notifications: int16_t[7] =
    [accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10]
  Sends CSV over UDP only after a valid notification is received.
*/

#include <ArduinoBLE.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ---------- BLE ----------
static const uint8_t bufferSize = 14;
const char* serviceUuid        = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// ---------- Network (edit PC_IP only) ----------
byte mac[] = { 0x02,0x12,0x34,0x56,0x78,0x9A }; // any locally-admin MAC
IPAddress PC_IP(1,1,1,1);                   // <-- set to your router's WAN IP (10.100.x.y)
const uint16_t PC_PORT    = 5005;               // destination UDP port on PC (forwarded)
const uint16_t LOCAL_PORT = 5006;               // Opta local UDP port
EthernetUDP Udp;

const char* DEVICE_ID = "optaA";
static uint32_t seq = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  if (Ethernet.localIP()==IPAddress(0,0,0,0)) {
    IPAddress ip(192,168,130,55), gw(192,168,130,1), mask(255,255,255,0), dns(192,168,130,1);
    Ethernet.begin(mac, ip, dns, gw, mask);
  }

  Serial.print("IP: ");      Serial.println(Ethernet.localIP());
  Serial.print("Gateway: "); Serial.println(Ethernet.gatewayIP());

  delay(200);
  Udp.begin(LOCAL_PORT);

  // BLE init
  if (!BLE.begin()) {
    Serial.println("Failed to initialize Bluetooth!");
    while (1) {;}
  }
}

void loop() {
  // 1) Scan until we actually see a peripheral advertising the target service
  Serial.println("Scanning for BLE devices...");
  BLE.scanForUuid(serviceUuid);

  BLEDevice peripheral;
  unsigned long scanStart = millis();
  while (millis() - scanStart < 5000) { // 5s scan window
    BLEDevice d = BLE.available();
    if (d) {
      peripheral = d;
      Serial.print("Device found: "); Serial.println(peripheral.address());
      Serial.print("Name: ");         Serial.println(peripheral.localName());
      break;
    }
    BLE.poll();
    delay(5);
  }
  BLE.stopScan();

  if (!peripheral) { delay(200); return; }

  // 2) Connect
  if (!peripheral.connect()) { Serial.println("Failed to connect"); delay(200); return; }
  Serial.println("Connected to peripheral device!");

  // 3) Discover attributes
  Serial.println("Discovering peripheral device attributes...");
  if (!peripheral.discoverAttributes()) {
    Serial.println("Failed to discover attributes");
    peripheral.disconnect();
    delay(200);
    return;
  }
  Serial.println("Peripheral device attributes discovered!");

  // 4) Locate service/characteristic and subscribe
  BLECharacteristic sensorCharacteristic = peripheral.characteristic(characteristicUuid);

  if (!sensorCharacteristic)            { Serial.println("Characteristic not found!"); peripheral.disconnect(); delay(200); return; }
  if (!sensorCharacteristic.canSubscribe()) { Serial.println("Characteristic not subscribable!"); peripheral.disconnect(); delay(200); return; }
  if (!sensorCharacteristic.subscribe())    { Serial.println("Subscription failed!"); peripheral.disconnect(); delay(200); return; }
  Serial.println("Subscribed to accelerometer/environment characteristic.");

  // 5) Receive notifications and forward via UDP
  //    Drain any backlog and send only the latest sample to avoid long lags.
  while (peripheral.connected()) {
    BLE.poll();  // keep BLE events flowing

    uint8_t data[bufferSize];
    bool got = false;

    // Drain queue: keep the most recent full payload
    while (sensorCharacteristic.valueUpdated()) {
      int bytes = sensorCharacteristic.readValue(data, sizeof(data));
      if (bytes == bufferSize) got = true;
      BLE.poll();
    }

    if (got) {
      // Parse little-endian int16_t values
      int16_t accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10;
      memcpy(&accX,      data + 0, 2);
      memcpy(&accY,      data + 2, 2);
      memcpy(&accZ,      data + 4, 2);
      memcpy(&temp_x100, data + 6, 2);
      memcpy(&hum_x100,  data + 8, 2);
      memcpy(&bvoc_x100, data +10, 2);
      memcpy(&iaq_x10,   data +12, 2);

      unsigned long ts_ms = millis();  // Opta timestamp

      // CSV: device,seq,ts_ms,accX,accY,accZ,temp_x100,hum_x100,bvoc_x100,iaq_x10
      char payload[140];
      int n = snprintf(payload, sizeof(payload),
                       "%s,%lu,%lu,%d,%d,%d,%d,%d,%d,%d\n",
                       DEVICE_ID, (unsigned long)seq++, ts_ms,
                       accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10);

      if (n > 0) {
        Udp.beginPacket(PC_IP, PC_PORT);
        Udp.write((const uint8_t*)payload, (size_t)n);
        Udp.endPacket();
      }
    }
    // no artificial delay; BLE.poll() already yields
  }

  Serial.println("Connection lost.");
  peripheral.disconnect();
  delay(200);
}
