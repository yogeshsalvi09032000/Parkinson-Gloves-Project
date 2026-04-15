/*
  Parkinson's Glove — SLAVE Firmware (Right Hand)
  =================================================
  MAC:    00:4B:12:3A:A9:94
  Master: 00:4B:12:3B:73:3C

  CRITICAL INIT ORDER:
    1. WiFi.mode(WIFI_STA)
    2. esp_now_init()          <-- ESP-NOW BEFORE BLE
    3. BLEDevice::init()       <-- BLE AFTER ESP-NOW
  This order prevents the LoadProhibited crash.

  Features:
    - ESP-NOW: receives patterns from master
    - BLE: HTML app connects here to control both gloves
    - NVS: persists settings, master reads them each cycle
    - Solo fallback if master lost for >10 seconds
*/

#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
#define SDA_PIN     21
#define SCL_PIN     22
#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// ── ESP-NOW sync packet ───────────────────────────────────────────────────────
typedef struct {
  uint8_t  pattern[3];
  uint16_t bdVal;
  uint16_t pdVal;
  uint8_t  maVal;
} GloveSync;

volatile bool  newDataReceived = false;
GloveSync      receivedData;
GloveSync      currentData;
unsigned long  lastSyncTime = 0;
const unsigned long SYNC_TIMEOUT = 10000;

// ── DRV2605 ───────────────────────────────────────────────────────────────────
Adafruit_DRV2605 drv[4];
bool motorOK[4];

// ── Settings ──────────────────────────────────────────────────────────────────
uint16_t MAval = 100;
uint16_t PDval = 744;
uint16_t BDval = 100;

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR1_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // MAval
#define CHAR2_UUID   "e3223119-9445-4e96-a4a1-85358c4046a2"  // PDval
#define CHAR3_UUID   "e3223119-9445-4e96-a4a1-85358c4046a3"  // BDval

// ── Permutation table ─────────────────────────────────────────────────────────
uint8_t perms[24][4] = {
  {0,1,2,3},{1,0,2,3},{2,0,1,3},{0,2,1,3},
  {1,2,0,3},{2,1,0,3},{2,1,3,0},{1,2,3,0},
  {3,2,1,0},{2,3,1,0},{1,3,2,0},{3,1,2,0},
  {3,0,2,1},{0,3,2,1},{2,3,0,1},{3,2,0,1},
  {0,2,3,1},{2,0,3,1},{1,0,3,2},{0,1,3,2},
  {3,1,0,2},{1,3,0,2},{0,3,1,2},{3,0,1,2}
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
  delay(5);
}
void TCA9548A_disable() {
  Wire.beginTransmission(0x70);
  Wire.write(0x00);
  Wire.endTransmission();
}
uint8_t amplitudeToEffect(uint16_t ma) {
  return map(constrain(ma, 0, 100), 0, 100, 1, 14);
}
void vibrateMotor(int idx, uint8_t ma) {
  if (!motorOK[idx]) return;
  TCA9548A(idx);
  drv[idx].setWaveform(0, amplitudeToEffect(ma));
  drv[idx].setWaveform(1, 0);
  drv[idx].go();
}
void stopMotor(int idx) {
  if (!motorOK[idx]) return;
  TCA9548A(idx);
  drv[idx].stop();
}

// ─────────────────────────────────────────────────────────────────────────────
// NVS
// ─────────────────────────────────────────────────────────────────────────────
void saveSettings() {
  Preferences prefs;
  prefs.begin("glove", false);
  prefs.putUShort("MAval", MAval);
  prefs.putUShort("PDval", PDval);
  prefs.putUShort("BDval", BDval);
  prefs.end();
  Serial.printf("[NVS] Saved MA=%d PD=%d BD=%d\n", MAval, PDval, BDval);
}

void loadSettings() {
  Preferences prefs;
  prefs.begin("glove", true);
  MAval = prefs.getUShort("MAval", 100);
  PDval = prefs.getUShort("PDval", 744);
  BDval = prefs.getUShort("BDval", 100);
  prefs.end();
  Serial.printf("[NVS] Loaded MA=%d PD=%d BD=%d\n", MAval, PDval, BDval);
}

// ─────────────────────────────────────────────────────────────────────────────
// BLE Callbacks
// ─────────────────────────────────────────────────────────────────────────────
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* p) {
    Serial.println("[BLE] Phone connected");
  }
  void onDisconnect(BLEServer* p) {
    Serial.println("[BLE] Disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class CB_MA : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* p) override {
    MAval = constrain(p->getValue().toInt(), 0, 100);
    saveSettings();
    Serial.printf("[BLE] Amplitude -> %d%%\n", MAval);
  }
};
class CB_PD : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* p) override {
    PDval = constrain(p->getValue().toInt(), 200, 2000);
    saveSettings();
    Serial.printf("[BLE] Pattern Duration -> %dms\n", PDval);
  }
};
class CB_BD : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* p) override {
    BDval = constrain(p->getValue().toInt(), 20, 500);
    saveSettings();
    Serial.printf("[BLE] Burst Duration -> %dms\n", BDval);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW receive callback
// ─────────────────────────────────────────────────────────────────────────────
void onDataReceived(const esp_now_recv_info* info,
                    const uint8_t* data, int len) {
  if (len == sizeof(GloveSync)) {
    memcpy(&receivedData, data, sizeof(GloveSync));
    newDataReceived = true;
    lastSyncTime    = millis();
    Serial.println("[ESP-NOW] Pattern received from master");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Init functions
// ─────────────────────────────────────────────────────────────────────────────
void setupMotors() {
  for (int i = 0; i < 4; i++) {
    TCA9548A(i);
    if (!drv[i].begin()) {
      Serial.printf("[MOTOR] Ch%d: FAIL\n", i);
      motorOK[i] = false;
    } else {
      drv[i].useLRA();
      drv[i].setMode(DRV2605_MODE_INTTRIG);
      drv[i].writeRegister8(0x1D, 161);
      motorOK[i] = true;
      Serial.printf("[MOTOR] Ch%d: OK\n", i);
    }
  }
  TCA9548A_disable();
}

void setupESPNow() {
  // ⚠️ ESP-NOW MUST be initialised before BLEDevice::init()
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.print("[WIFI] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);
  Serial.println("[ESP-NOW] Init OK — listening for master");
}

void setupBLE() {
  // ⚠️ BLE MUST be initialised AFTER esp_now_init()
  BLEDevice::init("ESP32");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* svc = pServer->createService(SERVICE_UUID);

  BLECharacteristic* c1 = svc->createCharacteristic(CHAR1_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  BLECharacteristic* c2 = svc->createCharacteristic(CHAR2_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  BLECharacteristic* c3 = svc->createCharacteristic(CHAR3_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  c1->addDescriptor(new BLE2902());
  c2->addDescriptor(new BLE2902());
  c3->addDescriptor(new BLE2902());
  c1->setCallbacks(new CB_MA());
  c2->setCallbacks(new CB_PD());
  c3->setCallbacks(new CB_BD());

  // Publish current values so app reads them on connect
  c1->setValue(String(MAval).c_str());
  c2->setValue(String(PDval).c_str());
  c3->setValue(String(BDval).c_str());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(false);
  adv->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as 'ESP32'");
}

void runPattern(GloveSync& data) {
  for (int pat = 0; pat < 3; pat++) {
    uint8_t slavePattern = (data.pattern[pat] + 12) % 24;
    uint8_t* order = perms[slavePattern];
    Serial.printf("Period %d master=%d slave=%d: ",
                  pat, data.pattern[pat], slavePattern);
    for (int j = 0; j < 4; j++) {
      Serial.printf("f%d ", order[j]);
      vibrateMotor(order[j], data.maVal);
      delay(data.bdVal);
      stopMotor(order[j]);
      delay(data.pdVal / 4 - data.bdVal);
    }
    Serial.println();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP — order matters!
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SLAVE Glove (Right Hand) ===");
  Serial.println("MAC: 00:4B:12:3A:A9:94");

  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(35));
  Wire.begin(SDA_PIN, SCL_PIN);

  loadSettings();
  setupMotors();
  setupESPNow();   // 1st — ESP-NOW before BLE
  setupBLE();      // 2nd — BLE after ESP-NOW

  currentData.bdVal = BDval;
  currentData.pdVal = PDval;
  currentData.maVal = MAval;
  for (int i = 0; i < 3; i++) currentData.pattern[i] = random(24);

  Serial.println("[READY] Slave running");
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  bool masterAlive = lastSyncTime > 0 &&
                     (millis() - lastSyncTime) < SYNC_TIMEOUT;

  if (newDataReceived) {
    memcpy(&currentData, &receivedData, sizeof(GloveSync));
    newDataReceived = false;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  if (masterAlive) {
    Serial.println("[SYNC] Running master pattern");
    runPattern(currentData);
  } else {
    Serial.println("[SOLO] No master — running independently");
    GloveSync solo;
    solo.bdVal = BDval;
    solo.pdVal = PDval;
    solo.maVal = MAval;
    for (int i = 0; i < 3; i++) solo.pattern[i] = random(24);
    runPattern(solo);
  }

  digitalWrite(LED_BUILTIN, LOW);
  delay(2 * (masterAlive ? currentData.pdVal : PDval));
}
