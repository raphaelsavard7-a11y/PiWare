# Configuration MQTT pour le contrôle des LEDs du LilyGo
#
# ⚠️ IMPORTANT:
# 1. Copiez ce fichier vers mqtt_config.py
# 2. Adaptez les valeurs selon votre configuration
# 3. Ne commitez JAMAIS mqtt_config.py (il est dans .gitignore)
#
# Commande pour créer votre fichier de config:
# cp mqtt_config.py.example mqtt_config.py
#
# Pour trouver le device_id de votre ESP32:
# 1. Connectez l'ESP32 via USB
# 2. Ouvrez le moniteur série (115200 baud)
# 3. Après la connexion WiFi, cherchez la ligne "Device ID: esp32-XXXXXX"
# 4. Copiez cette valeur complète (ex: "esp32-123456")

MQTT_CONFIG = {
    # Broker MQTT (via Cloudflare Tunnel)
    "broker": "mqtt.wafie.net",

    # Port WSS (WebSocket Secure) - 443 pour passer par Cloudflare
    "port": 443,

    # Identifiants Mosquitto
    "username": "esp_user",
    "password": "CHANGE_ME",  # ⚠️ Remplacez par votre mot de passe réel

    # Device ID de votre ESP32
    # Format: esp32-XXXXXX (les 6 derniers caractères de l'adresse MAC)
    "device_id": "esp32-raphael-savard",  # ⚠️ OBLIGATOIRE: Remplacez par l'ID réel affiché par l'ESP32
}

# Les topics MQTT seront automatiquement générés:
# - LED 1: {device_id}/led/1/set
# - LED 2: {device_id}/led/2/set
# - Bouton 1: {device_id}/button/1/state (lecture seule)
# - Bouton 2: {device_id}/button/2/state (lecture seule)
