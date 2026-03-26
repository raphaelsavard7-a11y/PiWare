// Test screen for Lilygo 7670e (ESP32)
// Sends component states via Serial, receives LED commands from Pi

// --- Pin definitions ---
#define LED_RED    21
#define LED_GREEN  22
#define LED_BLUE   23
#define POT1       39
#define POT2       36
#define BUTTON1    19
#define BUTTON2    18

#include <Wire.h>
#define MPU6050_ADDR 0x68
#define MPU6050_SDA  33
#define MPU6050_SCL  32

// --- State ---
#define SEND_INTERVAL 20   // ms between status updates
unsigned long lastSend = 0;
bool ledR = false, ledG = false, ledB = false;
float acX = 0, acY = 0, acZ = 0;
float gyX = 0, gyY = 0, gyZ = 0;

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

void setup() {
  Serial.begin(115200);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);

  digitalWrite(LED_RED,   LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  initMPU6050();
}

void loop() {
  // --- Receive LED toggle commands from Pi ---
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'R': ledR = !ledR; digitalWrite(LED_RED,   ledR); break;
      case 'G': ledG = !ledG; digitalWrite(LED_GREEN, ledG); break;
      case 'B': ledB = !ledB; digitalWrite(LED_BLUE,  ledB); break;
    }
  }

  // --- Send status line: S,btn1,btn2,pot1,pot2,ledR,ledG,ledB ---
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    bool b1 = (digitalRead(BUTTON1) == LOW);
    bool b2 = (digitalRead(BUTTON2) == LOW);
    int  p1 = analogRead(POT1);
    int  p2 = analogRead(POT2);
    readMPU6050();

    // CSV: S,btn1,btn2,pot1,pot2,ledR,ledG,ledB,acX,acY,acZ,gyX,gyY,gyZ
    Serial.print("S,");
    Serial.print(b1); Serial.print(',');
    Serial.print(b2); Serial.print(',');
    Serial.print(p1); Serial.print(',');
    Serial.print(p2); Serial.print(',');
    Serial.print(ledR); Serial.print(',');
    Serial.print(ledG); Serial.print(',');
    Serial.print(ledB); Serial.print(',');
    Serial.print(acX, 2); Serial.print(',');
    Serial.print(acY, 2); Serial.print(',');
    Serial.print(acZ, 2); Serial.print(',');
    Serial.print(gyX, 1); Serial.print(',');
    Serial.print(gyY, 1); Serial.print(',');
    Serial.println(gyZ, 1);
  }

  delay(2);
}
