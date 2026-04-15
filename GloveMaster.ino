/*
  Parkinson's Glove — MASTER Firmware (Left Hand)
  =================================================
  MAC:   00:4B:12:3B:73:3C
  Slave: 00:4B:12:3A:A9:94

  - Pure ESP-NOW (no BLE) — no radio conflicts
  - Generates VCR patterns, syncs to slave
  - Reads settings from NVS (slave writes them via BLE app)
*/

#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
#define SDA_PIN     21
#define SCL_PIN     22
#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif

// ── Slave MAC ─────────────────────────────────────────────────────────────────
uint8_t slaveMAC[] = {0x00, 0x4B, 0x12, 0x3A, 0xA9, 0x94};

// ── Sync packet ───────────────────────────────────────────────────────────────
typedef struct {
  uint8_t  pattern[3];
  uint16_t bdVal;
  uint16_t pdVal;
  uint8_t  maVal;
} GloveSync;

// ── DRV2605 ───────────────────────────────────────────────────────────────────
Adafruit_DRV2605 drv[4];
bool motorOK[4];

// ── Settings ──────────────────────────────────────────────────────────────────
uint16_t MAval = 100;
uint16_t PDval = 744;
uint16_t BDval = 100;

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

void vibrateMotor(int idx) {
  if (!motorOK[idx]) return;
  TCA9548A(idx);
  drv[idx].setWaveform(0, amplitudeToEffect(MAval));
  drv[idx].setWaveform(1, 0);
  drv[idx].go();
}
void stopMotor(int idx) {
  if (!motorOK[idx]) return;
  TCA9548A(idx);
  drv[idx].stop();
}

void loadSettings() {
  Preferences prefs;
  prefs.begin("glove", true);
  MAval = prefs.getUShort("MAval", 100);
  PDval = prefs.getUShort("PDval", 744);
  BDval = prefs.getUShort("BDval", 100);
  prefs.end();
  Serial.printf("[NVS] MA=%d PD=%d BD=%d\n", MAval, PDval, BDval);
}

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
  // ESP-NOW init — WiFi STA only, no BLE
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.print("[WIFI] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    return;
  }
  Serial.println("[ESP-NOW] Init OK");

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveMAC, 6);
  peer.channel = 0;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK)
    Serial.println("[ESP-NOW] Slave peer added");
  else
    Serial.println("[ESP-NOW] Peer add FAILED");
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== MASTER Glove (Left Hand) ===");

  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(35));
  Wire.begin(SDA_PIN, SCL_PIN);

  loadSettings();
  setupMotors();
  setupESPNow();   // pure WiFi — no BLE conflict

  Serial.println("[READY] Master running");
}

void loop() {
  loadSettings();  // pick up any BLE changes from slave

  // Build sync message
  GloveSync msg;
  msg.bdVal = BDval;
  msg.pdVal = PDval;
  msg.maVal = MAval;
  for (int p = 0; p < 3; p++) msg.pattern[p] = random(24);

  // Send to slave BEFORE firing
  esp_err_t res = esp_now_send(slaveMAC, (uint8_t*)&msg, sizeof(msg));
  Serial.printf("[ESP-NOW] Send: %s\n", res == ESP_OK ? "OK" : "FAIL");
  delay(20);

  // Fire VCR pattern
  digitalWrite(LED_BUILTIN, HIGH);
  for (int pat = 0; pat < 3; pat++) {
    uint8_t* order = perms[msg.pattern[pat]];
    Serial.printf("Period %d pattern %d: ", pat, msg.pattern[pat]);
    for (int j = 0; j < 4; j++) {
      Serial.printf("f%d ", order[j]);
      vibrateMotor(order[j]);
      delay(BDval);
      stopMotor(order[j]);
      delay(PDval / 4 - BDval);
    }
    Serial.println();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("[QUIET] %dms\n", 2 * PDval);
  delay(2 * PDval);
}
