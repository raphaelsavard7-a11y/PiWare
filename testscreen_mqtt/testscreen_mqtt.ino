// Test screen for Lilygo 7670e — MQTT version (via LTE)
// Publishes button/pot states, subscribes to LED commands
//
// MQTT topics (prefix = MQTT_CLIENT_ID from auth.h):
//   Publish:   {id}/testscreen/status    → JSON state every 200ms
//   Subscribe: {id}/testscreen/led/red   → "ON" / "OFF"
//              {id}/testscreen/led/green  → "ON" / "OFF"
//              {id}/testscreen/led/blue   → "ON" / "OFF"

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define ENABLE_DEBUG
#define ENABLE_ERROR_STRING
#define DEBUG_PORT Serial
#define SSLCLIENT_INSECURE_ONLY

#include <ESP_SSLClient.h>
#include <mbedtls/base64.h>
#include <Wire.h>

#include "auth.h"

// ====== MODEM PINS (Lilygo 7670) ======
#define MODEM_TX     26
#define MODEM_RX     27
#define MODEM_PWRKEY 4
#define MODEM_DTR    12  // Also BOARD_POWERON_PIN (DC boost enable)
#define MODEM_RI     13
#define MODEM_FLIGHT 25
#define MODEM_STATUS 0

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

// ====== MQTT/WSS ======
const char* MQTT_HOST = MQTT_BROKER;
const int   MQTT_WSS_PORT = 443;
const char* MQTT_PATH = "/";

char STATUS_TOPIC[60];
char LED_R_TOPIC[60];
char LED_G_TOPIC[60];
char LED_B_TOPIC[60];

// ====== STATE ======
bool ledR = false, ledG = false, ledB = false;
unsigned long lastSend = 0;
const unsigned long POLL_INTERVAL = 50;       // check inputs every 50ms
const unsigned long HEARTBEAT_INTERVAL = 5000; // force-send every 5s
const int POT_DEADBAND = 10;
unsigned long lastGprsCheck = 0;
const unsigned long GPRS_CHECK_INTERVAL = 30000;

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

// ====== NETWORK STACK ======
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem, 0);
ESP_SSLClient sslClient;

// ====== WEBSOCKET CLIENT ======
class WebSocketClient : public Client {
private:
  ESP_SSLClient* _ssl;
  bool _wsOk;
  uint8_t _rxBuf[512];
  size_t _rxLen, _rxPos;

  String genKey() {
    uint8_t k[16]; for(int i=0;i<16;i++) k[i]=random(0,256);
    size_t ol; unsigned char o[64];
    mbedtls_base64_encode(o,sizeof(o),&ol,k,16);
    return String((char*)o);
  }

  bool readFrame() {
    if(!_ssl->available()) return false;
    uint8_t b1=_ssl->read(); if(!_ssl->available()) return false;
    uint8_t b2=_ssl->read();
    uint8_t op=b1&0x0F; bool masked=(b2&0x80)!=0;
    size_t pLen=b2&0x7F;
    if(pLen==126){if(_ssl->available()<2)return false; pLen=(_ssl->read()<<8)|_ssl->read();}
    else if(pLen==127){if(_ssl->available()<8)return false; pLen=0; for(int i=0;i<8;i++) pLen=(pLen<<8)|_ssl->read();}
    uint8_t mask[4]={0};
    if(masked){if(_ssl->available()<4)return false; for(int i=0;i<4;i++) mask[i]=_ssl->read();}
    if(op==0x01||op==0x02){
      if((size_t)_ssl->available()<pLen) return false;
      _rxLen=pLen<sizeof(_rxBuf)?pLen:sizeof(_rxBuf);
      for(size_t i=0;i<_rxLen;i++){_rxBuf[i]=_ssl->read(); if(masked) _rxBuf[i]^=mask[i%4];}
      _rxPos=0; return true;
    } else if(op==0x08){_wsOk=false; return false;}
    else if(op==0x09){uint8_t p[2]={0x8A,0x00}; _ssl->write(p,2); return false;}
    return false;
  }
public:
  WebSocketClient(ESP_SSLClient* s):_ssl(s),_wsOk(false),_rxLen(0),_rxPos(0){}
  int connect(IPAddress ip,uint16_t port){return 0;}
  int connect(const char*host,uint16_t port){
    if(!_ssl->connect(host,port)) return 0;
    String k=genKey();
    _ssl->print("GET "); _ssl->print(MQTT_PATH); _ssl->print(" HTTP/1.1\r\nHost: ");
    _ssl->print(host); _ssl->print("\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ");
    _ssl->print(k); _ssl->print("\r\nSec-WebSocket-Protocol: mqtt\r\nSec-WebSocket-Version: 13\r\n\r\n");
    unsigned long t=millis(); while(!_ssl->available()&&millis()-t<5000) delay(10);
    if(!_ssl->available()) return 0;
    String r=""; while(_ssl->available()){char c=_ssl->read(); r+=c; if(r.endsWith("\r\n\r\n")) break;}
    if(r.indexOf("101")>0&&r.indexOf("Switching")>0){_wsOk=true; return 1;}
    return 0;
  }
  size_t write(uint8_t b){return write(&b,1);}
  size_t write(const uint8_t*buf,size_t sz){
    if(!_wsOk) return 0;
    uint8_t hdr[14]; int hl=2; hdr[0]=0x82;
    if(sz<126){hdr[1]=0x80|sz;} else if(sz<65536){hdr[1]=0x80|126; hdr[2]=(sz>>8)&0xFF; hdr[3]=sz&0xFF; hl=4;}
    else{hdr[1]=0x80|127; for(int i=0;i<8;i++) hdr[2+i]=0; hdr[6]=(sz>>24)&0xFF; hdr[7]=(sz>>16)&0xFF; hdr[8]=(sz>>8)&0xFF; hdr[9]=sz&0xFF; hl=10;}
    uint8_t m[4]; for(int i=0;i<4;i++){m[i]=random(0,256); hdr[hl+i]=m[i];} hl+=4;
    _ssl->write(hdr,hl);
    for(size_t i=0;i<sz;i++){uint8_t mb=buf[i]^m[i%4]; _ssl->write(&mb,1);}
    return sz;
  }
  int available(){
    if(_rxPos<_rxLen) return _rxLen-_rxPos;
    if(_ssl->available()&&readFrame()) return _rxLen-_rxPos;
    return 0;
  }
  int read(){
    if(_rxPos<_rxLen) return _rxBuf[_rxPos++];
    if(_ssl->available()&&readFrame()&&_rxPos<_rxLen) return _rxBuf[_rxPos++];
    return -1;
  }
  int read(uint8_t*buf,size_t sz){size_t c=0; while(c<sz){int v=read(); if(v<0) break; buf[c++]=(uint8_t)v;} return c;}
  int peek(){return(_rxPos<_rxLen)?_rxBuf[_rxPos]:-1;}
  void flush(){_ssl->flush();}
  void stop(){_wsOk=false; _ssl->stop();}
  uint8_t connected(){return _wsOk&&_ssl->connected();}
  operator bool(){return _wsOk;}
};

WebSocketClient wsClient(&sslClient);
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
  Wire.read(); Wire.read();
  int16_t rawGyX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyZ = (Wire.read() << 8) | Wire.read();
  acX = rawAcX / 16384.0;
  acY = rawAcY / 16384.0;
  acZ = rawAcZ / 16384.0;
  gyX = rawGyX / 131.0;
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

// ====== MODEM FUNCTIONS ======
void modemPowerOn() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH); delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH); delay(3000);
}

bool initModem() {
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  if (!modem.restart()) { Serial.println("[MODEM] Restart failed"); return false; }
  Serial.print("[MODEM] "); Serial.println(modem.getModemInfo());

  snprintf(STATUS_TOPIC, sizeof(STATUS_TOPIC), "%s/testscreen/status", MQTT_CLIENT_ID);
  snprintf(LED_R_TOPIC,  sizeof(LED_R_TOPIC),  "%s/testscreen/led/red", MQTT_CLIENT_ID);
  snprintf(LED_G_TOPIC,  sizeof(LED_G_TOPIC),  "%s/testscreen/led/green", MQTT_CLIENT_ID);
  snprintf(LED_B_TOPIC,  sizeof(LED_B_TOPIC),  "%s/testscreen/led/blue", MQTT_CLIENT_ID);
  return true;
}

bool connectNetwork() {
  modem.sendAT("+CGDCONT=1,\"IP\",\"", APN, "\""); modem.waitResponse();
  Serial.println("[NET] Waiting for network registration (up to 120s)...");
  Serial.print("[NET] Signal quality: "); Serial.println(modem.getSignalQuality());
  if (!modem.waitForNetwork(120000L)) {
    Serial.println("[NET] No network");
    Serial.print("[NET] Signal quality: "); Serial.println(modem.getSignalQuality());
    return false;
  }
  Serial.println("[NET] Registered. Connecting GPRS...");
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { Serial.println("[GPRS] Failed"); return false; }
  Serial.print("[GPRS] IP: "); Serial.println(modem.localIP());
  return true;
}

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
  // Enable DC boost ASAP to prevent brownout on battery power
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Testscreen MQTT ===");

  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   LOW);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, LOW);
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  LOW);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  initMPU6050();

  modemPowerOn();
  Serial.println("[INIT] Waiting for modem to stabilize...");
  delay(5000);  // Extra warm-up time (helps on battery power)

  // Retry modem + network init up to 3 times
  bool networkOk = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("[INIT] Attempt "); Serial.print(attempt); Serial.println("/3");
    if (initModem() && connectNetwork()) { networkOk = true; break; }
    Serial.println("[INIT] Power cycling modem and retrying in 10s...");
    modemPowerOn();
    delay(10000);
  }
  if (!networkOk) {
    Serial.println("[FATAL] Network init failed after 3 attempts");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(300); }
  }

  sslClient.setClient(&gsmClient);
  sslClient.setInsecure();
  sslClient.setBufferSizes(2048, 1024);
  sslClient.setDebugLevel(1);

  mqttClient.setServer(MQTT_HOST, MQTT_WSS_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);

  // Retry WSS + MQTT connection up to 3 times
  bool mqttOk = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("[MQTT] Connection attempt "); Serial.print(attempt); Serial.println("/3");
    if (wsClient.connect(MQTT_HOST, MQTT_WSS_PORT) && connectMQTT()) { mqttOk = true; break; }
    Serial.println("[MQTT] Retrying in 5s...");
    delay(5000);
  }
  if (!mqttOk) {
    Serial.println("[FATAL] MQTT init failed after 3 attempts");
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(500); }
  }

  Serial.println("=== Ready ===");
}

// ====== LOOP ======
void loop() {
  unsigned long now = millis();

  if (now - lastGprsCheck > GPRS_CHECK_INTERVAL) {
    lastGprsCheck = now;
    if (!modem.isGprsConnected()) {
      if (connectNetwork() && wsClient.connect(MQTT_HOST, MQTT_WSS_PORT))
        connectMQTT();
    }
  }

  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // Poll inputs and publish only on change (or heartbeat)
  if (now - lastSend >= POLL_INTERVAL) {
    lastSend = now;
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
