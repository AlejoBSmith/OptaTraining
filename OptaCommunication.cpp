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

void setup() {
  // Keep serial for status messages (no data streaming here)
  Serial.begin(115200);
  while (!Serial) {;}

  // Ethernet: DHCP with static fallback
  if (Ethernet.begin(mac) == 0) {
    IPAddress ip(1,1,1,1), dns(1,1,1,1), gw(1,1,1,1), mask(255,255,255,0);
    Ethernet.begin(mac, ip, dns, gw, mask);
  }
  delay(500);
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
      // Optional: you could also filter by name "NICLA"
      peripheral = d;
      Serial.print("Device found: "); Serial.println(peripheral.address());
      Serial.print("Name: ");         Serial.println(peripheral.localName());
      break;
    }
    delay(10);
  }
  BLE.stopScan();

  if (!peripheral) {
    // Nothing found this pass; try again
    delay(300);
    return;
  }

  // 2) Connect
  if (!peripheral.connect()) {
    Serial.println("Failed to connect to peripheral device!");
    delay(300);
    return;
  }
  Serial.println("Connected to peripheral device!");

  // 3) Discover attributes
  Serial.println("Discovering peripheral device attributes...");
  if (!peripheral.discoverAttributes()) {
    Serial.println("Failed to discover peripheral attributes!");
    peripheral.disconnect();
    delay(300);
    return;
  }
  Serial.println("Peripheral device attributes discovered!");

  // 4) Locate service/characteristic and subscribe
  BLEService         niclaService        = peripheral.service(serviceUuid);
  BLECharacteristic  sensorCharacteristic = peripheral.characteristic(characteristicUuid);

  if (!sensorCharacteristic) {
    Serial.println("Characteristic not found!");
    peripheral.disconnect();
    delay(300);
    return;
  }
  if (!sensorCharacteristic.canSubscribe()) {
    Serial.println("Characteristic not subscribable!");
    peripheral.disconnect();
    delay(300);
    return;
  }
  if (!sensorCharacteristic.subscribe()) {
    Serial.println("Subscription failed!");
    peripheral.disconnect();
    delay(300);
    return;
  }
  Serial.println("Subscribed to accelerometer/environment characteristic.");

  // 5) Receive notifications and forward via UDP
  //    Only send once we get a valid 14-byte payload.
  while (peripheral.connected()) {
    if (sensorCharacteristic.valueUpdated()) {
      uint8_t data[bufferSize];
      int bytes = sensorCharacteristic.readValue(data, sizeof(data));
      if (bytes == bufferSize) {
        // Parse little-endian int16_t values
        int16_t accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10;
        memcpy(&accX,      data + 0, 2);
        memcpy(&accY,      data + 2, 2);
        memcpy(&accZ,      data + 4, 2);
        memcpy(&temp_x100, data + 6, 2);
        memcpy(&hum_x100,  data + 8, 2);
        memcpy(&bvoc_x100, data +10, 2);
        memcpy(&iaq_x10,   data +12, 2);

        // CSV payload: X,Y,Z,temp_x100,hum_x100,bvoc_x100,iaq_x10\n
        char payload[96];
        int n = snprintf(payload, sizeof(payload), "%d,%d,%d,%d,%d,%d,%d\n",
                         accX, accY, accZ, temp_x100, hum_x100, bvoc_x100, iaq_x10);

        if (n > 0) {
          Udp.beginPacket(PC_IP, PC_PORT);
          Udp.write((const uint8_t*)payload, (size_t)n);
          Udp.endPacket();
          Serial.println("pakete enviado");
        }
      }
      // If bytes != 14, ignore and wait for the next valid notification
    }
    delay(1);
  }

  Serial.println("Connection lost.");
  peripheral.disconnect();
  delay(300);
}
