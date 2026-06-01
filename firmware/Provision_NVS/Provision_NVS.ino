/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         TRU-TRACK — Provision_NVS                           ║
 * ║   Flash this ONCE per device to store WiFi + MQTT           ║
 * ║   credentials securely in ESP32 NVS flash partition.        ║
 * ║   After flashing, upload TruTrack_v21.ino as main sketch.   ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * INSTRUCTIONS:
 *   1. Fill in your credentials in the CONFIG section below
 *   2. Flash to ESP32
 *   3. Open Serial Monitor (115200 baud) — verify "NVS written OK"
 *   4. Flash TruTrack_v21.ino (credentials persist in NVS)
 *
 * MQTT credentials:
 *   User:     esp32_device
 *   Password: TruTrackDevice2026
 *   Host:     14.139.121.53
 *   Port:     1883
 */

#include <Preferences.h>

// ════════════════════════════════════════════════════════════════
//  CONFIG — fill in before flashing
// ════════════════════════════════════════════════════════════════
const char* WIFI_SSID   = "YOUR_WIFI_SSID";
const char* WIFI_PASS   = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST   = "14.139.121.53";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER   = "esp32_device";
const char* MQTT_PASS   = "TruTrackDevice2026";
// ════════════════════════════════════════════════════════════════

Preferences prefs;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("  TRU-TRACK — NVS Provisioning");
  Serial.println("========================================");

  prefs.begin("creds", false);
  prefs.putString("wifi_ssid", WIFI_SSID);
  prefs.putString("wifi_pass", WIFI_PASS);
  prefs.putString("mqtt_host", MQTT_HOST);
  prefs.putUShort("mqtt_port", MQTT_PORT);
  prefs.putString("mqtt_user", MQTT_USER);
  prefs.putString("mqtt_pass", MQTT_PASS);
  prefs.end();

  // Verify by reading back
  prefs.begin("creds", true);
  char ssid[64], host[64], user[32];
  prefs.getString("wifi_ssid", ssid, sizeof(ssid));
  prefs.getString("mqtt_host", host, sizeof(host));
  prefs.getString("mqtt_user", user, sizeof(user));
  uint16_t port = prefs.getUShort("mqtt_port", 0);
  prefs.end();

  Serial.println("\nNVS written OK. Verification:");
  Serial.printf("  WiFi SSID : %s\n", ssid);
  Serial.printf("  MQTT Host : %s:%d\n", host, port);
  Serial.printf("  MQTT User : %s\n", user);
  Serial.println("\nNow flash TruTrack_v21.ino as the main sketch.");
  Serial.println("Credentials will persist in NVS across flashes.");
}

void loop() {
  // Nothing — provisioning is one-time
}
