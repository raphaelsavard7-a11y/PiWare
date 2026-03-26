// auth.h - Configuration WiFi et MQTT
#ifndef AUTH_H
#define AUTH_H

// ============================================================================
// CONFIGURATION WiFi - CHOISIR UN SEUL TYPE DE SÉCURITÉ
// ============================================================================

// --- Option 1: WPA2-Personal (réseau domestique) ---
// #define WIFI_SECURITY_WPA2_PERSONAL

// --- Option 2: WPA2-Enterprise (réseau du Cégep) ---
#define WIFI_SECURITY_WPA2_ENTERPRISE

// ============================================================================
// WPA2-Personal
// ============================================================================
const char* WIFI_SSID     = "climoilou";
const char* WIFI_PASSWORD = "CHANGE_ME";

// ============================================================================
// WPA2-Enterprise (EAP-PEAP MSCHAPv2)
// ============================================================================
const char* EAP_IDENTITY = "CHANGE_ME";
const char* EAP_USERNAME = "CHANGE_ME";
const char* EAP_PASSWORD = "CHANGE_ME";

// ============================================================================
// CONFIGURATION MQTT
// ============================================================================
const char* MQTT_BROKER    = "mqtt.wafie.net";
const int   MQTT_PORT      = 443;
const char* MQTT_USER      = "esp_user";
const char* MQTT_PASS      = "CHANGE_ME";
const char* MQTT_CLIENT_ID = "esp32-raphael-savard";

#endif // AUTH_H
