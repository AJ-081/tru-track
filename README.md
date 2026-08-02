# TRU-TRACK

Real-time vehicle tracking and navigation system with GNSS/IMU sensor fusion.
Developed at SVNIT Surat by the team.

**Current mainline: `tt-v3.0.0`** — everything after v3.0.0 is experimental and archived (see [Archived branches](#archived--experimental)).

---

## System Overview

TRU-TRACK fuses a multi-constellation GNSS receiver (Quectel L89HA, NavIC / dual-band L1+L5) with a 6-axis IMU (MPU6050) using a 15-state Error-State Kalman Filter running on an ESP32. Telemetry is published at 5 Hz over 4G LTE (SIMCom A7672S) via MQTT to a cloud backend, and visualised on a real-time web dashboard.

```
ESP32 (dual-core FreeRTOS)
  Core 0 — modemTask : LTE attach, MQTT publish, NTP, outbox drain
  Core 1 — loop()    : IMU @100 Hz, ESKF predict/update, GNSS NMEA feed
        │
        ├── L89HA GNSS   (UART1 @9600)  GPS + GLONASS + BDS + Galileo + NavIC
        ├── MPU6050 IMU  (I2C 0x68)     accel + gyro @100 Hz
        ├── MAX17048     (I2C 0x36)     battery voltage / SoC / charge rate
        └── A7672S 4G    (UART2 @921600) MQTT transport
                 │
                 └── MQTT → ingest → MongoDB → Flask API → dashboard
```

---

## Hardware

| Component | Role | Interface | Pins |
|---|---|---|---|
| ESP32 (custom PCB) | Main controller | — | dual-core 240 MHz |
| SIMCom A7672S | 4G LTE modem + MQTT | UART2 | RX 17 / TX 16 |
| Quectel L89HA | Dual-band NavIC GNSS | UART1 | RX 26 / TX 27 |
| MPU6050 | 6-axis IMU | I2C 0x68 | SDA 21 / SCL 22 |
| MAX17048 | Battery fuel gauge | I2C 0x36 | SDA 21 / SCL 22, ALRT → 4 |
| LM2596S | 2S LiPo (8.4 V) → 5 V | — | — |
| Status LED (onboard) | Blink-pattern state | GPIO | 2 |
| LED RED | Error / not ready | GPIO | 14 |
| LED GREEN | Driving / ESKF active | GPIO | 12 |
| LED YELLOW | Waiting / warning | GPIO | 13 |

> **Note (GPIO12):** GPIO12 is an ESP32 strapping pin. If held HIGH at power-on it selects 1.8 V flash mode and the chip may fail to boot. Known risk on this PCB revision — consider moving GREEN to GPIO32/33 on the next board spin.

### PCB LED states

| RED | GREEN | YELLOW | Meaning |
|---|---|---|---|
| ● | | | IMU not calibrated, or no GNSS fix |
| ● | | ● | Network / MQTT down |
| | | ● | GNSS fix acquired, waiting for motion |
| | ● | | **Driving** — ESKF active, speed > 1 m/s |
| | ● | ● | Ready but stationary |
| ● | ● | | Driving + battery critical |

### GPIO2 blink patterns

| Pattern | Meaning |
|---|---|
| Fast (100 ms) | Booting / IMU calibrating |
| Medium (500 ms) | Waiting for GNSS fix |
| Triple | Fix acquired, waiting for motion to init ESKF |
| Double | ESKF aligning (2 s) |
| Solid | Fully ready and tracking |
| Rapid (50 ms) | LTE or MQTT link down |

---

## Carrier compatibility

APN is auto-detected from the IMSI prefix on every attach attempt.

| Carrier | Status | APN |
|---|---|---|
| Airtel | ✅ Working | `airtelgprs.com` |
| BSNL | ✅ Working | `bsnlnet` |
| Vi | ✅ Expected working | `portalnmms` |
| Jio | ❌ **Not usable** | `jionet` |

**Jio:** registration returns `LTE:DENIED / CS:DENIED / PS:DENIED` with full signal (CSQ 27–31) on multiple known-good SIMs. Jio runs a VoLTE-only network and enforces an IMEI whitelist of certified consumer devices; M2M modules such as the A7672S are not on it. This is a carrier-side policy and **cannot be fixed in firmware**. Use Airtel/BSNL/Vi.

---

## Repository structure

```
firmware/
  4g/TruTrack_v300/       ← CURRENT MAINLINE (tt-v3.0.0)
  4g/TruTrack_v276/       ← v2.7.6 reference
  TruTrack_v21/           ← historical v2.1
  libraries/Eskf3D/       ← Eskf3D, EskfMath, GeoUtils, ImuConvert, EskfConfig
  Provision_NVS/          ← NVS provisioning sketch (MQTT host, APN)
  archive/                ← experimental builds after v3.0.0 (see below)

backend/                  ← Flask API + WebSocket (port 9000, gunicorn)
ingest/                   ← MQTT async ingest service → MongoDB
tuning/                   ← ESKF offline replay + tuning UI (port 9001)
dashboard/                ← web dashboard (single-device + fleet)
dashboard_backup_*/       ← pre-rebuild snapshots

deployment/
  nginx/                  ← site config (HTTPS, socket.io, /tuning auth)
  systemd/                ← service units + drop-in overrides
  scripts/                ← backup + disk-check cron
  mongodb/                ← index definitions

secrets.env               ← server credentials
DEPLOYMENT.md             ← full deployment guide
```

---

## Firmware version history

### v2.x — schema and fusion correctness

| Version | Changes |
|---|---|
| **v2.1** | Legacy WiFi + MQTT. Single-constellation GPS only. |
| **v2.3** | 4G transport added (A7672S). Boot-looped on network loss. |
| **v2.5** | Dual-core architecture. GNSS moved from A7672S AT-polling → dedicated L89HA on UART1. WiFi removed entirely. UART raised to 921600. No `ESP.restart()` on network loss. |
| **v2.6** | GNSS dedup (each fix consumed once). CSQ poll 10 s → 30 s. Link-RESTORED logging. Optional LittleFS offline buffer (default OFF). |
| **v2.7** | **Payload schema v2.7.** `gnss.speed` canonical alias, live-computed `gnss_age_ms`, NavIC/Galileo SV counts, `constellations_used[]`, `yaw_init_source/speed/course`, `gnss_pos_applied/gated`, `nhc_applied`, `zupt_applied`, `accel_bias_mps2`, raw innovation components. `imu_hf` now carries **raw int16 + corrected floats + fs/gs** (required for Allan variance and replay — corrected-only double-corrects bias). Eskf3D library: `updateGnssLLA()` returns `EskfGnssStatus`, NHC/ZUPT return `bool`, added `getAccelBias()`. |
| **v2.7.3** | **Yaw-init wait-for-motion.** ESKF no longer initialises while stationary — waits for >1.0 m/s so yaw always comes from real GNSS course. Eliminated `default_north` heading contamination (mean NIS 8180 → 952, ~8.6×). New LED triple-blink state. SIM-ready polling loop. Modem-stuck watchdog (`ESP.restart()` after 120 s). |
| **v2.7.4** | **MAX17048 fuel gauge live** — register-level driver, no external library. `battery_v` / `battery_pct` / `battery_low` / `battery_critical` populated (placeholders since v2.6). New field `battery_crate_pct_hr`. IMSI-based APN auto-detect. Removed `$PSTMSAVEPAR` from boot sequence (was corrupting L89HA saved config and silencing NMEA → 0 sats). Added `$PSTMNMEACONFIG` recovery. **Fixed GPS UART pins: RX 26 / TX 27** (were swapped). |
| **v2.7.5** | Airtel IMSI prefix list expanded (40498 and others were not matching). |
| **v2.7.6** | **Gravity-derived roll/pitch init.** ESKF previously always initialised roll=pitch=0 (assumed flat); now derives true attitude from the gravity vector captured during calibration via new `Eskf3D::setInitialAttitude(roll,pitch,yaw)`. Fixes garbage output when the device is mounted at any angle. **Motion-robust IMU calibration** — retries until accel stddev < 0.08 m/s² so motion during startup cannot contaminate bias. New telemetry: `yaw_init_roll_rad`, `yaw_init_pitch_rad`. |
| **v2.7.7** | **Hot-swap support.** IMU: I2C probe each cycle, reinit + recalibrate on reconnect. MAX17048: probe every 5 s when absent, reinit on reconnect. GNSS: 30 s no-data timeout triggers `gnssInit()`. Modem already covered by existing reconnect watchdog. |
| **v2.8.1** | PCB tri-colour status LEDs (RED/GREEN/YELLOW on GPIO 14/12/13) with `syncStatusLeds()` and boot self-test. |
| **v2.8.2** | Added `AT+CNBP` LTE band unlock (later found unnecessary — see v2.9.5). |
| **v2.9.x** | **Stationary drift fix.** `bias_ax/ay/az` forced to 0 at init — gravity was being subtracted twice (once in `readIMU()`, once inside the ESKF). **ZUPT false-firing fix** — now requires GNSS speed < 0.35 m/s **and** ESKF state speed < 0.60 m/s before the IMU static test runs, preventing velocity clamping at constant highway speed. Tightened `EskfConfig`: σ_acc 0.8 → 0.35, σ_gnss_pos 5.0 → 2.0, σ_gnss_vel 1.0 → 0.5. New `ImuConvert` library module. |
| **v2.9.5** | Reverted the v2.8.2 band-lock (it was not the Jio cause) and reset band config to auto. Added explicit serial note documenting the Jio IMEI block. |

### v3.0.0 — consolidated mainline ✅

Current production build. Header fully rewritten with accurate wiring, all LED states, carrier compatibility, hot-swap behaviour, ESKF summary and version history; ~150 lines of stale v2.5–v2.7 changelog removed (now in git history).

Consolidates everything from v2.7.x through v2.9.x: MAX17048 fuel gauge, PCB status LEDs, hot-swap for IMU/GNSS/fuel gauge, gravity-derived attitude init, motion-robust calibration, stationary drift fix, ZUPT double-gate, tightened EskfConfig, carrier auto-detect, ImuConvert.

**All hardware verified end-to-end on this build.**

### Archived / experimental

> Work after v3.0.0 is **archived and not part of the mainline**. Development continues from v3.0.0.

| Version | Status | Notes |
|---|---|---|
| **v3.1.0** | 🧪 Archived | microSD storage. Two-file ring buffer replacing LittleFS for outage recovery (`OFFLINE_BUFFER_MODE=2`, main + HF), plus always-on black-box session logging to `/sessions/<id>.jsonl` from first sample of boot, independent of link state. Wiring: VSPI CS 5 / SCK 18 / MISO 19 / MOSI 23, **3.3 V only**. |
| **v3.1.1** | 🧪 Archived | Sanitised `session_id` for FAT32 filenames (colons are illegal → silent `SD.open()` failure left `/sessions` empty). |
| **v3.1.2** | 🧪 Archived | SD mount retry ×3; 8.3 short-filename fallback; yields in drain loop. |

**Known unresolved issues in the v3.1.x line:**
- microSD mount fails intermittently on cold boot (succeeds on retry-boot).
- Black-box session file never opens — both long and 8.3 names fail while the volume reports mounted. Suspected root cause: **card formatted with a non-power-of-two 3 KB allocation unit**, which is not valid FAT32 geometry. Reformat with the official SD Association *SD Card Formatter* before further debugging.
- Task watchdog abort in `modemTask` when draining a large buffer after reconnect — partially mitigated with yields, not fully resolved.

---

## Server

- **URL:** `https://tru-track.bittest.in`
- **SSH:** `crl@14.139.121.53`
- **OS:** Ubuntu 24

| Service | Port | Description |
|---|---|---|
| `tru-track-backend` | 9000 | Flask API + WebSocket (gunicorn) |
| `tru-track-ingest` | — | MQTT → MongoDB |
| `tru-track-tuning` | 9001 | ESKF replay / tuning (basic auth) |
| `mongod` | 27017 | MongoDB (`nav` database) |
| `mosquitto` | 1883 | MQTT broker |
| `nginx` | 80/443 | Reverse proxy + Let's Encrypt SSL |

### Health check

```bash
sudo systemctl status tru-track-backend tru-track-ingest tru-track-tuning mongod mosquitto --no-pager | grep Active
curl -s http://127.0.0.1:9000/api/server/health | python3 -m json.tool
```

---

## MQTT topics

| Topic | Rate | Contents |
|---|---|---|
| `nav/{device_id}/eskf` | 5 Hz | Full telemetry — identity, status, GNSS, ESKF, one IMU sample |
| `nav/{device_id}/imu_hf` | 1 Hz | Batch of 100 raw + corrected IMU samples (100 Hz stream) |

---

## MongoDB collections

| Collection | Contents | Retention |
|---|---|---|
| `gnss_raw` | Raw GNSS fixes (5 Hz) | 90 days |
| `eskf_state` | Fused ESKF output + internals (5 Hz) | 90 days |
| `imu_raw` | IMU snapshots (5 Hz) | 90 days |
| `imu_hf` | Unpacked 100 Hz IMU samples | 7 days (TTL) |
| `device_latest` | Latest full payload per device | permanent |
| `sessions` | Per-boot session metadata | permanent |

---

## Sensor fusion

15-state Error-State Kalman Filter: position (3), velocity (3), attitude (3), accel bias (3), gyro bias (3).

- **Predict** at 100 Hz from IMU; **update** from GNSS position (chi² gated, threshold 8.9443 = √80), GNSS velocity, and ZUPT.
- **NHC disabled** (`USE_NHC=0`) — deliberate, for vehicle-agnostic deployment (2-wheelers through multi-axle).
- **Yaw init** waits for motion > 1 m/s, then uses GNSS course. `yaw_init_source` can therefore only ever be `"course"` or `"none"`.
- **Roll/pitch init** derived from the gravity vector captured during startup calibration.
- **Accel bias forced to 0 at init** — the ESKF handles gravity internally; subtracting it in firmware as well caused stationary drift.

### Tuning status

Current `EskfConfig` was tuned in Octave on 100 Hz `imu_hf` data. Mean NIS on a clean post-fix session is ~950 against a target of ~3, verdict `too_tight` — the σ values likely need a re-tune on a multi-drive clean dataset. The 8.6× improvement from 8180 → 952 came from the yaw-init fix, not from parameter changes.

Offline replay **must** use `imu_hf` (100 Hz raw), never `imu_raw` (5 Hz) — 5 Hz replay diverges catastrophically because `dt` sits exactly at the `ESKF_MAX_DT` clamp.

---

## Setup for a new developer

1. Clone the repo.
2. Copy `secrets.env` → `/etc/tru-track/secrets.env`.
3. `sudo cp deployment/systemd/*.service /etc/systemd/system/`
4. `sudo cp -r deployment/systemd/*.service.d /etc/systemd/system/`
5. `sudo systemctl daemon-reload`
6. `sudo cp deployment/nginx/tru-track /etc/nginx/sites-enabled/`
7. `pip install -r requirements.txt` in each service folder.
8. `sudo systemctl enable --now tru-track-backend tru-track-ingest tru-track-tuning`

See `DEPLOYMENT.md` for the full guide.

## Firmware flash

1. Open `firmware/4g/TruTrack_v300/TruTrack_v300.ino` in Arduino IDE (folder name must match the `.ino` name).
2. Copy `firmware/libraries/Eskf3D/` into your Arduino `libraries/` folder — all files including `ImuConvert.h/.cpp`.
3. Flash `firmware/Provision_NVS/Provision_NVS.ino` once to write MQTT host / APN into NVS.
4. Flash `TruTrack_v300.ino`.

**Required libraries:** ArduinoJson v6, MPU6050 (Electronic Cats), TinyGPSPlus, Eskf3D (local).

---

## Key lessons learned

- **Never initialise yaw while stationary.** A default-North guess rotates every dead-reckoned position and produces a huge NIS spike that decays but never recovers.
- **`imu_hf` must carry raw int16**, not bias-corrected floats — otherwise offline replay double-corrects bias and Allan variance is impossible.
- **Shared logic must live in one function.** The `wifi_rssi_dbm` → `lte_rssi_dbm` rename was fixed in the dashboard but silently missed in the alerts and fleet endpoints; alert computation is now shared via `_compute_alerts()`.
- **Self-host all front-end libraries.** CDN-loaded socket.io/Leaflet/Chart.js time out on the institute network and freeze the dashboard even though the server itself can reach the CDN.
- **Enforce log rotation and TTLs from day one** — uncapped logging previously filled the disk.
- **Don't send `$PSTMSAVEPAR` on every boot** — repeated flash writes to the L89HA corrupted its saved config and silenced NMEA output entirely.

---

## Team

Devaam Dalal · Abha Jadav · Vrushti Javeri · Dhruv Shah
Faculty advisor: Dr. Shweta Shah, Electronics Engineering, SVNIT Surat

## Contributions

- ESKF and Website by Abha Jadav
- ESP code and Backend by Devaam Dalal
- PCB by Dhruv Shah
- Ai by Vrusthi

##Publication

**ION GNSS+ 2026** — September 14–18, Orlando, FL
