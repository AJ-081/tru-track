# TRU-TRACK Firmware

ESP32 firmware for real-time GNSS + IMU + ESKF vehicle tracking.

---

## Directory Structure

```
firmware/
├── TruTrack_v21/           ← Main sketch (flash this)
│   ├── TruTrack_v21.ino
│   └── EskfConfig.h        ← ESKF noise tuning parameters
├── Provision_NVS/          ← Flash ONCE per device before main sketch
│   └── Provision_NVS.ino
└── libraries/
    └── Eskf3D/             ← Custom 15-state ESKF library
        ├── Eskf3D.h / .cpp
        ├── EskfMath.h / .cpp
        └── EskfConfig.h
```

---

## First-time Setup (per device)

### Step 1 — Install Arduino Libraries

**Via Arduino Library Manager** (Tools → Manage Libraries):

| Library | Author | Version tested |
|---------|--------|----------------|
| TinyGPSPlus | Mikal Hart | 1.0.3 |
| PubSubClient | Nick O'Leary | 2.8 |
| ArduinoJson | Benoit Blanchon | 6.x |
| MPU6050 | Electronic Cats | 1.3.0 |

**Manual install — Eskf3D (custom library):**
1. Copy the `firmware/libraries/Eskf3D/` folder into your Arduino libraries directory:
   - Windows: `C:\Users\<name>\Documents\Arduino\libraries\`
   - Linux/Mac: `~/Arduino/libraries/`
2. Restart Arduino IDE
3. Verify: Sketch → Include Library → you should see "Eskf3D"

**Board support — ESP32:**
1. File → Preferences → Additional Boards Manager URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. Tools → Board → Boards Manager → search "esp32" → install **esp32 by Espressif**
3. Select: Tools → Board → ESP32 Arduino → **DOIT ESP32 DEVKIT V1**

---

### Step 2 — Provision NVS (once per device)

1. Open `Provision_NVS/Provision_NVS.ino`
2. Fill in WiFi SSID, password, and MQTT details in the CONFIG section
3. Flash to device
4. Open Serial Monitor (115200 baud) — verify **"NVS written OK"**

---

### Step 3 — Flash Main Firmware

1. Open `TruTrack_v21/TruTrack_v21.ino`
2. Check the **DEVICE CONFIGURATION** section at the top:
   ```cpp
   static const int GNSS_RX_PIN = 16;  // NEO-6M TX → GPIO16
   static const int GNSS_TX_PIN = 17;  // NEO-6M RX → GPIO17
   ```
   If GNSS shows NO_FIX despite outdoor conditions, swap pins to 17/16.
3. Flash to device
4. Open Serial Monitor — expected boot sequence:
   ```
   TRU-TRACK tt-v2.1
   NVS OK — SSID: ...
   WiFi connecting... OK
   NTP sync (IST UTC+5:30)... OK — IST: 2026-xx-xx xx:xx:xx
   MQTT connected.
   GNSS UART2 started (RX=GPIO16 TX=GPIO17) @ 9600 baud
   MPU6050 OK.
   IMU calibrating — keep device still for ~2s...
   ```

---

## Wiring

```
ESP32 DevKit V1
│
├── GPIO 16 (UART2 RX) ←── NEO-6M TX
├── GPIO 17 (UART2 TX) ──► NEO-6M RX
├── GPIO 21 (I2C SDA)  ←─► MPU6050 SDA
├── GPIO 22 (I2C SCL)  ←─► MPU6050 SCL
├── GPIO 2  (LED)           Built-in — no wiring needed
├── 3.3V ──────────────────► NEO-6M VCC + MPU6050 VCC
└── GND  ──────────────────► NEO-6M GND + MPU6050 GND
                              MPU6050 AD0 → GND (I2C addr 0x68)
```

---

## LED Status Guide

| Pattern | Meaning |
|---------|---------|
| Fast blink (100ms) | Boot / WiFi connecting / IMU calibrating |
| Medium blink (500ms) | WiFi OK — waiting for GNSS fix |
| Double flash (ON-OFF-ON-pause) | GNSS fix — ESKF aligning (2s) |
| **Solid ON** | **Fully ready — tracking** |
| Rapid blink (50ms) | WiFi or MQTT lost |

---

## MQTT Payload

Topic: `nav/{MAC_ADDRESS}/eskf`  
Rate: 5 Hz (every 200ms)

```json
{
  "device_id": "00:70:07:83:BA:FC",
  "session_id": "00:70:07:83:BA:FC:BOOTID",
  "fw_version": "tt-v2.1",
  "t_ms": 12345,
  "t_epoch_ms": 1748000000000,
  "gnss": { "fix_valid": true, "lat": 21.16, "lon": 72.78, "sats": 8 },
  "eskf": { "init_valid": true, "alignment_valid": true, "lat": 21.16, "lon": 72.78 },
  "imu":  { "calibrated": true, "accel_mps2": [...], "gyro_radps": [...] }
}
```

---

## ESKF Tuning

Edit `EskfConfig.h` to adjust noise parameters.  
Current baseline (`tt-eskf-v1.0`):

| Parameter | Value | Notes |
|-----------|-------|-------|
| SIGMA_ACC | 0.15 m/s²/√Hz | MPU6050 datasheet ref |
| SIGMA_GYRO | 0.005 rad/s/√Hz | Reduced — trust gyro more |
| SIGMA_GNSS_POS | 3.0 m | NEO-6M typical CEP |
| GATING_THRESH | 10.0 (χ²) | Mahalanobis outlier rejection |

Systematic tuning via MATLAB NIS analysis is planned (see project roadmap).
