/**
 * Opta firmware — BLE (Nicla Sense ME) -> UDP (LAN)
 * Expects 26-byte notifications (struct Packet) from Nicla.
 * Sends CSV over UDP only after a valid notification is received.
 */

#include <ArduinoBLE.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ---------- BLE ----------
// The buffer size must match the size of the 'Packet' struct from Nicla
static const uint8_t bufferSize = 26;
const char* serviceUuid        = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* characteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// Data structure to receive the packet from Nicla
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

// ---------- Network (edit PC_IP only) ----------
byte mac[] = { 0x02,0x12,0x34,0x56,0x78,0x9A }; // a locally-administered MAC
IPAddress PC_IP(10,100,32,166);                 // <-- set to your PC's IP or router's WAN IP
const uint16_t PC_PORT     = 5005;              // destination UDP port on PC
const uint16_t LOCAL_PORT  = 5006;              // Opta local UDP port
EthernetUDP Udp;

const char* DEVICE_ID = "optaA";
static uint32_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Ethernet initialization with DHCP for robustness
  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // If DHCP fails, try with a static IP as a fallback
    IPAddress ip(192,168,143,36);
    IPAddress gateway(192,168,143,33);
    IPAddress subnet(255,255,255,224);
    IPAddress dns(192,168,18,2);
    Ethernet.begin(mac, ip, dns, gateway, subnet);
  }


  Serial.print("IP: ");     Serial.println(Ethernet.localIP());
  Serial.print("Gateway: "); Serial.println(Ethernet.gatewayIP());

  delay(200);
  Udp.begin(LOCAL_PORT);

  // BLE initialization
  if (!BLE.begin()) {
    Serial.println("Failed to initialize Bluetooth!");
    while (1) {;}
  }
}

void loop() {
  BLEDevice peripheral;

  // 1) Search for the BLE peripheral
  Serial.println("Scanning for BLE devices...");
  BLE.scanForUuid(serviceUuid);

  unsigned long scanStart = millis();
  while (millis() - scanStart < 5000) { // 5-second scan window
    BLEDevice d = BLE.available();
    // Corrected line: converts String to const char* using .c_str()
    if (d && d.hasLocalName() && strcmp(d.localName().c_str(), "NICLA") == 0) {
      peripheral = d;
      Serial.print("Device found: "); Serial.println(peripheral.address());
      Serial.print("Name: ");         Serial.println(peripheral.localName());
      break;
    }
    BLE.poll();
    delay(5);
  }
  BLE.stopScan();

  if (!peripheral) { 
    delay(200); 
    return; // No peripheral found, restart loop
  }

  // 2) Connect to the peripheral
  if (!peripheral.connect()) { 
    Serial.println("Failed to connect"); 
    delay(200); 
    return;
  }
  Serial.println("Connected to peripheral device!");

  // 3) Discover attributes
  if (!peripheral.discoverAttributes()) {
    Serial.println("Failed to discover attributes");
    peripheral.disconnect();
    delay(200);
    return;
  }
  Serial.println("Peripheral device attributes discovered!");

  // 4) Locate and subscribe to the characteristic
  BLECharacteristic sensorCharacteristic = peripheral.characteristic(characteristicUuid);

  if (!sensorCharacteristic || !sensorCharacteristic.canSubscribe() || !sensorCharacteristic.subscribe()) {
    Serial.println("Characteristic not found, not subscribable, or subscription failed!");
    peripheral.disconnect();
    delay(200);
    return;
  }
  Serial.println("Subscribed to characteristic.");

  // 5) Process BLE notifications and forward via UDP
  while (peripheral.connected()) {
    BLE.poll();

    if (sensorCharacteristic.valueUpdated()) {
      Packet pkt;
      int bytes = sensorCharacteristic.readValue((uint8_t*)&pkt, sizeof(pkt));

      if (bytes == sizeof(pkt)) {
        // Format data for CSV and print to Serial
        char payload[140];
        int n = snprintf(payload, sizeof(payload),
                         "%s,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
                         DEVICE_ID, (unsigned long)seq++, (unsigned long)pkt.sampleTime,
                         pkt.accXRMS, pkt.accYRMS, pkt.accZRMS, pkt.temp, pkt.hum, pkt.bvoc, pkt.iaq, pkt.anomaly);
        
        Serial.print(payload); // Optional: print to Serial Monitor

        if (n > 0) {
          // Send UDP packet
          Udp.beginPacket(PC_IP, PC_PORT);
          Udp.write((const uint8_t*)payload, (size_t)n);
          Udp.endPacket();
        }
      }
    }
  }

  Serial.println("Connection lost.");
  peripheral.disconnect();
  delay(200);
}
