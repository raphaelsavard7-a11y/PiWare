// auth.h - Configuration LTE/Cellulaire
#ifndef AUTH_H
#define AUTH_H

// ============================================================================
// CONFIGURATION APN (Access Point Name) - CELLULAIRE
// ============================================================================

const char APN[] = "internet.keepgo.com";
const char APN_USER[] = "";             // Généralement vide au Canada
const char APN_PASS[] = "";             // Généralement vide au Canada

// ============================================================================
// CONFIGURATION MQTT
// ============================================================================

const char MQTT_BROKER[] = "mqtt.wafie.net";
const int  MQTT_PORT = 1883;  // Port standard MQTT (non-SSL via LTE)
const char MQTT_USER[] = "esp_user";
const char MQTT_PASS[] = "CHANGE_ME";

// Device ID - Identifiant unique de l'appareil
// Format suggéré: "esp32-XXXXXX" (6 derniers caractères de l'IMEI ou personnalisé)
// Cet identifiant sera utilisé comme préfixe pour tous les topics MQTT
const char MQTT_CLIENT_ID[] = "esp32-raphael-savard";

#endif // AUTH_H
