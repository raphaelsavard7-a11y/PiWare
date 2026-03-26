// Test screen for Lilygo 7670e — MQTT over WiFi (WSS)
// Publishes button/pot states, subscribes to LED commands
//
// MQTT topics (prefix = MQTT_CLIENT_ID from auth.h):
//   Publish:   {id}/testscreen/status    → JSON state (on change + heartbeat)
//   Subscribe: {id}/testscreen/led/red   → "ON" / "OFF"
//              {id}/testscreen/led/green  → "ON" / "OFF"
//              {id}/testscreen/led/blue   → "ON" / "OFF"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <mbedtls/base64.h>
#include <Wire.h>
#include "auth.h"

#ifdef WIFI_SECURITY_WPA2_ENTERPRISE
#include "esp_eap_client.h"
#endif

// ====== WEBSOCKET CLIENT WRAPPER ======
class WebSocketClient : public Client {
private:
  WiFiClientSecure* _sslClient;
  bool _wsConnected;
  uint8_t _rxBuffer[512];
  size_t _rxBufferLen;
  size_t _rxBufferPos;

  String generateWebSocketKey() {
    uint8_t key[16];
    for(int i = 0; i < 16; i++) key[i] = random(0, 256);
    size_t olen;
    unsigned char output[64];
    mbedtls_base64_encode(output, sizeof(output), &olen, key, 16);
    return String((char*)output);
  }

  bool readWebSocketFrame() {
    if (!_sslClient->available()) return false;
    uint8_t byte1 = _sslClient->read();
    if (!_sslClient->available()) return false;
    uint8_t byte2 = _sslClient->read();
    uint8_t opcode = byte1 & 0x0F;
    bool masked = (byte2 & 0x80) != 0;
    size_t payloadLen = byte2 & 0x7F;
    if (payloadLen == 126) {
      if (_sslClient->available() < 2) return false;
      payloadLen = (_sslClient->read() << 8) | _sslClient->read();
    } else if (payloadLen == 127) {
      if (_sslClient->available() < 8) return false;
      payloadLen = 0;
      for(int i = 0; i < 8; i++) payloadLen = (payloadLen << 8) | _sslClient->read();
    }
    uint8_t mask[4] = {0};
    if (masked) {
      if (_sslClient->available() < 4) return false;
      for(int i = 0; i < 4; i++) mask[i] = _sslClient->read();
    }
    if (opcode == 0x01 || opcode == 0x02) {
      if (_sslClient->available() < payloadLen) return false;
      _rxBufferLen = payloadLen < sizeof(_rxBuffer) ? payloadLen : sizeof(_rxBuffer);
      for(size_t i = 0; i < _rxBufferLen; i++) {
        _rxBuffer[i] = _sslClient->read();
        if (masked) _rxBuffer[i] ^= mask[i % 4];
      }
      _rxBufferPos = 0;
      return true;
    } else if (opcode == 0x08) {
      _wsConnected = false;
      return false;
    } else if (opcode == 0x09) {
      uint8_t pong[2] = {0x8A, 0x00};
      _sslClient->write(pong, 2);
      return false;
    }
    return false;
  }

public:
  WebSocketClient(WiFiClientSecure* sslClient) {
    _sslClient = sslClient;
    _wsConnected = false;
    _rxBufferLen = 0;
    _rxBufferPos = 0;
  }

  int connect(IPAddress ip, uint16_t port) { return 0; }
  int connect(const char *host, uint16_t port) {
    Serial.println("[WSS] Connecting SSL...");
    if (!_sslClient->connect(host, port)) {
      Serial.println("[WSS] SSL connection failed");
      return 0;
    }
    Serial.println("[WSS] SSL connected, sending WebSocket handshake...");
    String wsKey = generateWebSocketKey();
    _sslClient->print("GET /mqtt HTTP/1.1\r\nHost: ");
    _sslClient->print(host);
    _sslClient->print("\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ");
    _sslClient->print(wsKey);
    _sslClient->print("\r\nSec-WebSocket-Protocol: mqtt\r\nSec-WebSocket-Version: 13\r\n\r\n");
    unsigned long timeout = millis();
    while (!_sslClient->available() && millis() - timeout < 5000) delay(10);
    if (!_sslClient->available()) {
      Serial.println("[WSS] Handshake timeout");
      return 0;
    }
    String response = "";
    while (_sslClient->available()) {
      char c = _sslClient->read();
      response += c;
      if (response.endsWith("\r\n\r\n")) break;
    }
    if (response.indexOf("101") > 0 && response.indexOf("Switching Protocols") > 0) {
      Serial.println("[WSS] WebSocket handshake OK!");
      _wsConnected = true;
      return 1;
    }
    Serial.println("[WSS] WebSocket handshake failed");
    return 0;
  }

  size_t write(uint8_t b) { return write(&b, 1); }
  size_t write(const uint8_t *buf, size_t size) {
    if (!_wsConnected) return 0;
    uint8_t header[14];
    int headerLen = 2;
    header[0] = 0x82;
    if (size < 126) {
      header[1] = 0x80 | size;
    } else if (size < 65536) {
      header[1] = 0x80 | 126;
      header[2] = (size >> 8) & 0xFF;
      header[3] = size & 0xFF;
      headerLen = 4;
    } else {
      header[1] = 0x80 | 127;
      for(int i = 0; i < 8; i++) header[2 + i] = 0;
      header[6] = (size >> 24) & 0xFF;
      header[7] = (size >> 16) & 0xFF;
      header[8] = (size >> 8) & 0xFF;
      header[9] = size & 0xFF;
      headerLen = 10;
    }
    uint8_t mask[4];
    for(int i = 0; i < 4; i++) { mask[i] = random(0, 256); header[headerLen + i] = mask[i]; }
    headerLen += 4;
    _sslClient->write(header, headerLen);
    for(size_t i = 0; i < size; i++) {
      uint8_t maskedByte = buf[i] ^ mask[i % 4];
      _sslClient->write(&maskedByte, 1);
    }
    return size;
  }

  int available() {
    if (_rxBufferPos < _rxBufferLen) return _rxBufferLen - _rxBufferPos;
    if (_sslClient->available()) {
      if (readWebSocketFrame()) return _rxBufferLen - _rxBufferPos;
    }
    return 0;
  }

  int read() {
    if (_rxBufferPos < _rxBufferLen) return _rxBuffer[_rxBufferPos++];
    if (_sslClient->available()) {
      if (readWebSocketFrame() && _rxBufferPos < _rxBufferLen) return _rxBuffer[_rxBufferPos++];
    }
    return -1;
  }

  int read(uint8_t *buf, size_t size) {
    size_t count = 0;
    while (count < size) { int c = read(); if (c < 0) break; buf[count++] = (uint8_t)c; }
    return count;
  }

  int peek() {
    if (_rxBufferPos < _rxBufferLen) return _rxBuffer[_rxBufferPos];
    return -1;
  }

  void flush() { _sslClient->flush(); }
  void stop() { _wsConnected = false; _sslClient->stop(); }
  uint8_t connected() { return _wsConnected && _sslClient->connected(); }
  operator bool() { return _wsConnected; }
};

// ====== COMPONENT PINS ======
#define LED_RED    21
#define LED_GREEN  22
#define LED_BLUE   23
#define POT1       39
#define POT2       36
#define BUTTON1    19
#define BUTTON2    18

#define MPU6050_ADDR 0x68
#define MPU6050_SDA  33
#define MPU6050_SCL  32

// ====== MQTT TOPICS ======
char STATUS_TOPIC[60];
char LED_R_TOPIC[60];
char LED_G_TOPIC[60];
char LED_B_TOPIC[60];

// ====== STATE ======
bool ledR = false, ledG = false, ledB = false;
unsigned long lastPoll = 0;
const unsigned long POLL_INTERVAL = 50;
const unsigned long HEARTBEAT_INTERVAL = 5000;
const int POT_DEADBAND = 10;
unsigned long lastGprsCheck = 0;

// Previous values for change detection
bool prevB1 = false, prevB2 = false;
int  prevP1 = -99, prevP2 = -99;
bool prevLedR = false, prevLedG = false, prevLedB = false;
unsigned long lastPublish = 0;
float acX = 0, acY = 0, acZ = 0;
float gyX = 0, gyY = 0, gyZ = 0;
float prevAcX = 0, prevAcY = 0, prevAcZ = 0;
float prevGyX = 0, prevGyY = 0, prevGyZ = 0;
const float ACCEL_DEADBAND = 0.05;
const float GYRO_DEADBAND = 1.0;

// ====== NETWORK ======
WiFiClientSecure wifiClient;
WebSocketClient wsClient(&wifiClient);
PubSubClient mqttClient(wsClient);

// ====== MPU-6050 ======
void initMPU6050() {
  Wire.begin(MPU6050_SDA, MPU6050_SCL);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  int16_t rawAcX = (Wire.read() << 8) | Wire.read();
  int16_t rawAcY = (Wire.read() << 8) | Wire.read();
  int16_t rawAcZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  int16_t rawGyX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyZ = (Wire.read() << 8) | Wire.read();
  acX = rawAcX / 16384.0; // ±2g range
  acY = rawAcY / 16384.0;
  acZ = rawAcZ / 16384.0;
  gyX = rawGyX / 131.0;   // ±250°/s range
  gyY = rawGyY / 131.0;
  gyZ = rawGyZ / 131.0;
}

// ====== MQTT CALLBACK ======
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (strcmp(topic, LED_R_TOPIC) == 0) {
    ledR = (msg == "ON"); digitalWrite(LED_RED, ledR);
  } else if (strcmp(topic, LED_G_TOPIC) == 0) {
    ledG = (msg == "ON"); digitalWrite(LED_GREEN, ledG);
  } else if (strcmp(topic, LED_B_TOPIC) == 0) {
    ledB = (msg == "ON"); digitalWrite(LED_BLUE, ledB);
  }
}

// ====== WIFI ======
bool connectWiFi() {
  Serial.print("[WIFI] Connecting to "); Serial.println(WIFI_SSID);

#ifdef WIFI_SECURITY_WPA2_ENTERPRISE
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_eap_client_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin(WIFI_SSID);
#else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] Connected, IP: "); Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("[WIFI] Connection failed");
  return false;
}

// ====== MQTT ======
bool connectMQTT() {
  Serial.println("[MQTT] Connecting...");
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    mqttClient.subscribe(LED_R_TOPIC);
    mqttClient.subscribe(LED_G_TOPIC);
    mqttClient.subscribe(LED_B_TOPIC);
    Serial.println("[MQTT] Connected + subscribed");
    return true;
  }
  Serial.print("[MQTT] Failed, rc="); Serial.println(mqttClient.state());
  return false;
}

// ====== SETUP ======
void setup() {
  // Enable DC boost early to prevent brownout on battery
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Testscreen MQTT WiFi ===");

  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   LOW);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, LOW);
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  LOW);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  initMPU6050();

  snprintf(STATUS_TOPIC, sizeof(STATUS_TOPIC), "%s/testscreen/status", MQTT_CLIENT_ID);
  snprintf(LED_R_TOPIC,  sizeof(LED_R_TOPIC),  "%s/testscreen/led/red", MQTT_CLIENT_ID);
  snprintf(LED_G_TOPIC,  sizeof(LED_G_TOPIC),  "%s/testscreen/led/green", MQTT_CLIENT_ID);
  snprintf(LED_B_TOPIC,  sizeof(LED_B_TOPIC),  "%s/testscreen/led/blue", MQTT_CLIENT_ID);

  // Retry WiFi up to 3 times
  bool wifiOk = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("[INIT] WiFi attempt "); Serial.print(attempt); Serial.println("/3");
    if (connectWiFi()) { wifiOk = true; break; }
    delay(5000);
  }
  if (!wifiOk) {
    Serial.println("[FATAL] WiFi failed after 3 attempts");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(300); }
  }

  wifiClient.setInsecure();
  mqttClient.setServer(MQTT_BROKER, 443);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);

  // Connect WebSocket then MQTT
  Serial.println("[WSS] Connecting to broker...");
  if (!wsClient.connect(MQTT_BROKER, 443)) {
    Serial.println("[FATAL] WebSocket connection failed");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(500); }
  }

  // Retry MQTT up to 3 times
  bool mqttOk = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("[MQTT] Attempt "); Serial.print(attempt); Serial.println("/3");
    if (connectMQTT()) { mqttOk = true; break; }
    delay(5000);
  }
  if (!mqttOk) {
    Serial.println("[FATAL] MQTT failed after 3 attempts");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(500); }
  }

  Serial.println("=== Ready ===");
}

// ====== LOOP ======
void loop() {
  unsigned long now = millis();

  // Reconnect WiFi if dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected, reconnecting...");
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      wsClient.connect(MQTT_BROKER, 443);
    }
  }

  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // Poll inputs and publish only on change (or heartbeat)
  if (now - lastPoll >= POLL_INTERVAL) {
    lastPoll = now;
    bool b1 = (digitalRead(BUTTON1) == LOW);
    bool b2 = (digitalRead(BUTTON2) == LOW);
    int  p1 = analogRead(POT1);
    int  p2 = analogRead(POT2);
    readMPU6050();

    bool changed = (b1 != prevB1) || (b2 != prevB2)
                || (abs(p1 - prevP1) > POT_DEADBAND)
                || (abs(p2 - prevP2) > POT_DEADBAND)
                || (ledR != prevLedR) || (ledG != prevLedG) || (ledB != prevLedB)
                || (fabs(acX - prevAcX) > ACCEL_DEADBAND)
                || (fabs(acY - prevAcY) > ACCEL_DEADBAND)
                || (fabs(acZ - prevAcZ) > ACCEL_DEADBAND)
                || (fabs(gyX - prevGyX) > GYRO_DEADBAND)
                || (fabs(gyY - prevGyY) > GYRO_DEADBAND)
                || (fabs(gyZ - prevGyZ) > GYRO_DEADBAND);
    bool heartbeat = (now - lastPublish >= HEARTBEAT_INTERVAL);

    if (changed || heartbeat) {
      prevB1 = b1; prevB2 = b2;
      prevP1 = p1; prevP2 = p2;
      prevLedR = ledR; prevLedG = ledG; prevLedB = ledB;
      prevAcX = acX; prevAcY = acY; prevAcZ = acZ;
      prevGyX = gyX; prevGyY = gyY; prevGyZ = gyZ;
      lastPublish = now;

      char buf[256];
      snprintf(buf, sizeof(buf),
        "{\"btn1\":%d,\"btn2\":%d,\"pot1\":%d,\"pot2\":%d,"
        "\"ledR\":%d,\"ledG\":%d,\"ledB\":%d,"
        "\"acX\":%.2f,\"acY\":%.2f,\"acZ\":%.2f,"
        "\"gyX\":%.1f,\"gyY\":%.1f,\"gyZ\":%.1f}",
        b1, b2, p1, p2, ledR, ledG, ledB,
        acX, acY, acZ, gyX, gyY, gyZ);
      mqttClient.publish(STATUS_TOPIC, buf);
    }
  }

  delay(10);
}
