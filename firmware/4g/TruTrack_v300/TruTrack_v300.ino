/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║              TRU-TRACK  v3.0.0  —  FULL HARDWARE REV             ║
 * ║   ESP32 + A7672S (4G) + L89HA (NavIC GNSS) + MPU6050 (IMU)       ║
 * ║   + MAX17048 (fuel gauge) — Custom PCB, SVNIT Navigation         ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * ARCHITECTURE
 *   Dual-core FreeRTOS:
 *   Core 0 — modemTask: LTE attach, MQTT, NTP, outbox drain
 *   Core 1 — loop():    IMU @ 100 Hz, ESKF predict/update, GNSS feed
 *
 *   Outboxes (teleOut / hfOut) are latest-wins ring buffers protected
 *   by mutexes. If Core 0 is busy, stale frames are overwritten —
 *   live tracking always shows the most recent position.
 *
 *   UART2 @ 921600 baud: A7672S (4G modem + MQTT via AT commands)
 *   UART1 @ 9600 baud:   L89HA  (NavIC GNSS, NMEA streaming)
 *   I2C (SDA21/SCL22):   MPU6050 (0x68) + MAX17048 (0x36)
 *
 * WIRING
 *   A7672S TX  → ESP32 GPIO17  (UART2 RX)
 *   A7672S RX  → ESP32 GPIO16  (UART2 TX)
 *   L89HA  TX  → ESP32 GPIO26  (UART1 RX)   ← NMEA stream in
 *   L89HA  RX  → ESP32 GPIO27  (UART1 TX)   ← config out
 *   L89HA  ANT → dual-band L1+L5 ACTIVE antenna (required for NavIC)
 *   MPU6050 SDA → GPIO21   SCL → GPIO22   (I2C 0x68)
 *   MAX17048 SDA → GPIO21  SCL → GPIO22   (I2C 0x36)
 *   MAX17048 ALRT → GPIO4  (alert interrupt, INPUT_PULLUP)
 *   MAX17048 CELL+ → 2S battery Cell-1 mid-tap
 *   LED (status, onboard) → GPIO2
 *   LED_RED    (D14) → GPIO14  error / not ready
 *   LED_GREEN  (D12) → GPIO12  driving / ESKF active + moving
 *   LED_YELLOW (D13) → GPIO13  waiting / warning state
 *   LM2596S: 2S LiPo (8.4V) → 5V for system
 *
 * PCB LED STATES
 *   RED only             → IMU not calibrated or no GNSS fix
 *   RED + YELLOW         → Network/MQTT down (attention needed)
 *   YELLOW only          → GNSS fix acquired, waiting for motion
 *   GREEN only           → Driving — ESKF active, speed > 1 m/s
 *   GREEN + YELLOW       → Ready but stationary
 *   RED + GREEN          → Driving + battery critical
 *
 * GPIO2 LED STATES (onboard blink patterns)
 *   Fast blink   100ms  → Booting / IMU calibrating
 *   Medium blink 500ms  → Waiting for GNSS fix
 *   Triple blink        → Fix acquired, waiting for motion to init ESKF
 *   Double blink        → ESKF initializing (2s alignment)
 *   Solid ON            → Fully ready and tracking
 *   Rapid blink   50ms  → LTE or MQTT link down
 *
 * CARRIER COMPATIBILITY
 *   Airtel ✅  BSNL ✅  Vi ✅  Jio ✗ (IMEI not on Jio VoLTE whitelist)
 *   APN is auto-detected from IMSI prefix at every attach attempt.
 *
 * NVS PROVISIONING (namespace "creds", use Provision_NVS sketch)
 *   mqtt_host  (required)
 *   mqtt_port  (default 1883)
 *   mqtt_user / mqtt_pass
 *   apn        (optional — auto-detected if unset)
 *
 * MQTT TOPICS
 *   nav/{device_id}/eskf    5 Hz   full telemetry (schema v2.7)
 *   nav/{device_id}/imu_hf  1 Hz   100 raw+corrected IMU samples
 *
 * SENSOR FUSION
 *   15-state Error-State Kalman Filter (ESKF)
 *   States: position(3) + velocity(3) + attitude(3) + accel_bias(3) + gyro_bias(3)
 *   Updates: GNSS position (gated, chi²=80), GNSS velocity, ZUPT
 *   NHC disabled (USE_NHC=0) — vehicle-agnostic deployment
 *   Yaw init: waits for motion >1 m/s, then uses GNSS course
 *   Roll/pitch init: derived from gravity vector at calibration time
 *   Bias: accel bias forced to 0 at init (ESKF handles gravity internally)
 *
 * HOT-SWAP SUPPORT
 *   L89HA  — reinit if no NMEA for >30s (same as GNSS-deny recovery)
 *   MPU6050 — I2C probe every cycle, reinit + recalibrate on reconnect
 *   MAX17048 — probe every 5s when absent, reinit on reconnect
 *   A7672S  — existing reconnect watchdog + 120s ESP.restart() fallback
 *
 * REQUIRED LIBRARIES
 *   ArduinoJson v6
 *   MPU6050 (Electronic Cats)
 *   TinyGPSPlus
 *   Eskf3D (local — Eskf3D.h/cpp, EskfMath, GeoUtils, ImuConvert)
 *   EskfConfig.h (tuned parameters)
 *
 * VERSION HISTORY
 *   v2.5   L89HA GNSS (moved from A7672S AT-polling), dual-core arch
 *   v2.6   GNSS dedup, offline buffer (OFF by default), schema fields
 *   v2.7   Full schema v2.7: NavIC, gate/NHC/ZUPT flags, bias, innovations
 *   v2.7.3 Yaw-init wait-for-motion, LED triple-blink, modem watchdog
 *   v2.7.4 MAX17048 live, Jio APN auto-detect, GPS pin fix, PSTMSAVEPAR removed
 *   v2.7.6 Gravity-derived roll/pitch init, motion-robust IMU calibration
 *   v2.7.7 Hot-swap: IMU/MAX17048/GNSS
 *   v2.8.1 PCB status LEDs (RED/GREEN/YELLOW on GPIO14/12/13)
 *   v2.9.x Stationary drift fix (bias=0, ZUPT double-gate), tighter sigma
 *   v3.0.0 Full hardware verified, consolidated mainline release
 */

// ════════════════════════════════════════════════════════════════
//  OFFLINE BUFFER  (optional — default OFF, zero behavior change)
// ════════════════════════════════════════════════════════════════
//  When enabled, telemetry frames that fail to publish (link down) are
//  queued to flash (LittleFS) instead of being dropped, then replayed
//  once the link returns. Three modes:
//
//    0 = OFF (default)        — v2.5 behavior: drop frames while down.
//    1 = MAIN TOPIC ONLY       — buffer nav/.../eskf only. Recommended:
//                                 small (~1.4KB/pkt), position track is
//                                 the data that matters most to recover.
//    2 = MAIN + HF             — also buffer nav/.../imu_hf. Much more
//                                 flash traffic (100Hz source data) —
//                                 only use this if HF tuning data must
//                                 survive outages too.
//
//  Ring buffer caps how many frames are queued; oldest is dropped first
//  once full, so flash usage is bounded regardless of outage length.
#define OFFLINE_BUFFER_MODE        0          // 0/1/2 — see above
#define OFFLINE_BUFFER_MAX_MAIN    600         // ~2 min @ 5Hz
#define OFFLINE_BUFFER_MAX_HF      60          // ~1 min @ 1Hz batches
#define OFFLINE_BUFFER_DRAIN_PER_TICK 3        // frames replayed per modemTask pass
#if OFFLINE_BUFFER_MODE > 0
  #include <LittleFS.h>
#endif

// ════════════════════════════════════════════════════════════════
//  MODEM + L89 GNSS PIN CONFIG  (plain int — safe before Arduino.h)
// ════════════════════════════════════════════════════════════════
static const int MODEM_RX_PIN = 17;   // ESP32 UART2 RX  ← A7672S TX
static const int MODEM_TX_PIN = 16;   // ESP32 UART2 TX  → A7672S RX
static const int GPS_RX_PIN   = 26;   // ESP32 UART1 RX  ← L89HA TX
static const int GPS_TX_PIN   = 27;   // ESP32 UART1 TX  → L89HA RX
static const long GPS_BAUD    = 9600; // L89HA default NMEA baud

// ════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <sys/time.h>
#include <time.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include "Eskf3D.h"
#include "EskfConfig.h"

// ════════════════════════════════════════════════════════════════
//  CONSTANTS
// ════════════════════════════════════════════════════════════════
static const uint32_t MODEM_BAUD_BOOT = 115200;   // factory default
static const uint32_t MODEM_BAUD_FAST = 921600;   // runtime target
static const uint32_t CSQ_POLL_MS     = 30000;    // signal quality — diagnostic only, kept slow (briefly blocks outbox drain)

static const char* FW_VERSION   = "tt-v3.0.0";
static const char* ESKF_CFG_VER = "tt-eskf-v1.0";

// ── LED ────────────────────────────────────────────────────────
#define LED_PIN 2
enum LedMode { LED_FAST_BLINK, LED_MEDIUM_BLINK, LED_TRIPLE_BLINK,
               LED_DOUBLE_BLINK, LED_SOLID, LED_RAPID_BLINK };
LedMode  ledMode    = LED_FAST_BLINK;
bool     ledState   = false;
uint32_t ledLastMs  = 0;
uint8_t  ledSubStep = 0;
static const uint16_t DBL_PATTERN[] = {120, 100, 120, 100, 800};
#define DBL_STEPS 5
// Triple-blink: "have GPS fix, waiting for motion to set heading" —
// distinct from medium-blink (no fix at all yet) and double-blink
// (fix + moving, currently in the 2s alignment window). Without this,
// standing next to a stationary vehicle gave no visual difference
// between "GPS still searching" and "GPS locked, just need to drive."
static const uint16_t TRIPLE_PATTERN[] = {120,100,120,100,120,100,700};
#define TRIPLE_STEPS 7

// ── Timing ─────────────────────────────────────────────────────
static const uint32_t PUB_PERIOD_MS  = 200;    // 5 Hz telemetry build
static const uint32_t IMU_PERIOD_MS  = 10;     // 100 Hz IMU
static const uint32_t IMU_HF_PUB_MS  = 1000;   // 1 Hz HF batch
static const uint32_t ALIGN_MS       = 2000;
static const long     GMT_OFFSET_SEC = 19800;  // IST = UTC+5:30

// ── GNSS gating ────────────────────────────────────────────────
static const float GNSS_VEL_GATE_MPS = 0.8f;
// ZUPT must be speed-gated. IMU-only static detection can falsely fire
// during smooth driving and pull the ESKF away from the road.
static const float ZUPT_GNSS_SPEED_GATE_MPS  = 0.35f;
static const float ZUPT_STATE_SPEED_GATE_MPS = 0.60f;
static const float NHC_STATE_SPEED_MIN_MPS   = 1.00f;

// ── GNSS-outage DR governor ──────────────────────────────────────────────────
// Prevents INS velocity from exploding straight during GNSS-denied turns.
// Integrates gyro-z to rotate last-known heading, decays speed slowly,
// then blends that target into the ESKF velocity each IMU tick.
static const bool     DR_OUTAGE_ENABLE       = true;
static const float    DR_VEL_BLEND_GAIN      = 0.05f; // per 100Hz tick (~5%/tick)
static const float    DR_MAX_SPEED_MPS       = 14.0f; // hard cap during outage
static const float    DR_MIN_SPEED_MPS       = 1.00f; // don't blend if nearly stopped
static const float    DR_SPEED_DECAY_PER_SEC = 0.015f;// slow speed reduction
static const float    DR_DR_SIGMA_MPS        = 0.5f;  // P_vv inflation per tick
static const uint32_t DR_GNSS_STALE_MS       = 1500;  // start DR after GNSS stale this long
// ─────────────────────────────────────────────────────────────────────────────


// ── Auto yaw-gain calibration (v30) ──────────────────────────────────────────
// Learns dr_yaw_gain from GNSS course rate vs gz_rps during healthy GNSS,
// so the DR governor self-corrects for MPU6050 mounting sign/scale.
static const bool     DR_AUTO_YAW_GAIN_ENABLE     = true;
static const float    DR_YAW_GAIN_DEFAULT         = 1.0f;
static const float    DR_YAW_GAIN_ALPHA           = 0.04f;
static const float    DR_YAW_GAIN_MIN_ABS         = 0.25f;
static const float    DR_YAW_GAIN_MAX_ABS         = 3.00f;
static const float    DR_CAL_SPEED_MIN_MPS        = 2.0f;
static const float    DR_CAL_GYRO_MIN_RADPS       = 0.010f;
static const float    DR_CAL_COURSE_RATE_MIN      = 0.010f;
static const uint32_t DR_CAL_MIN_DT_MS            = 500;
static const uint32_t DR_CAL_MAX_DT_MS            = 4000;
// ─────────────────────────────────────────────────────────────────────────────

// ── IMU sensitivity ─────────────────────────────────────────────
static const float ACCEL_SENS = 16384.0f;
static const float GYRO_SENS  = 131.0f;
static const float G_MPS2     = 9.80665f;
static const float DEG2RAD    = M_PI / 180.0f;
static const float KN_TO_MPS  = 0.514444f;

// ── JSON ────────────────────────────────────────────────────────
// Widened 3072 -> 4096: v2.6.1 schema adds ~34 more JSON keys/array
// elements (yaw_init provenance, update-status flags, accel_bias,
// constellations_used[], extended innov components). This sizes the
// ArduinoJson StaticJsonDocument node pool, separate from teleOut.buf
// (the serialized string buffer).
static const size_t JSON_BUF = 4096;

// ── IMU HF buffer ───────────────────────────────────────────────
// Carries BOTH raw int16 counts AND bias-corrected floats. Raw is the
// schema-correct field for offline replay/Allan-variance (firmware
// bias correction must not be baked into the signal used for that),
// corrected floats are kept too for any consumer that wants them
// directly without doing the raw->physical conversion itself.
#define IMU_HF_BUF_SIZE 100
struct ImuHfSample {
    uint32_t t_ms;
    int16_t  ax_raw, ay_raw, az_raw;
    int16_t  gx_raw, gy_raw, gz_raw;
    float    ax, ay, az;     // bias-corrected, m/s^2
    float    gx, gy, gz;     // bias-corrected, rad/s
};
ImuHfSample imu_hf_buf[IMU_HF_BUF_SIZE];
uint8_t     imu_hf_count = 0;
uint32_t    lastImuHfMs  = 0;

// ════════════════════════════════════════════════════════════════
//  GNSS — data delivered by L89HA via TinyGPSPlus (NMEA, UART1)
// ════════════════════════════════════════════════════════════════
struct GnssFix {
    bool   valid      = false;
    int    mode       = 0;
    int    gpsSv      = 0;
    int    gloSv      = 0;
    int    bdsSv      = 0;
    int    galSv      = 0;
    int    irnssSv    = 0;   // NavIC — was folded into totalSv only, now exposed
    int    totalSv    = 0;
    double lat        = 0.0;
    double lon        = 0.0;
    double altM       = 0.0;
    double speedKnots = 0.0;
    double courseDeg  = 0.0;
    double hdop       = 0.0;
    double vdop       = 0.0;
    uint32_t ageMs    = 0;
};

// Live GNSS state — written by Core 0 modemTask, read by Core 1 loop.
GnssFix           g_fix;
uint32_t          lastGnssFixMs  = 0;
SemaphoreHandle_t gnssMutex      = nullptr;

// ── Outboxes: Core 1 writes, Core 0 (modemTask) publishes ──────
//    Latest-wins. Core 1 uses zero-timeout take → never blocks.
struct Outbox {
    char              buf[12288];  // widened from 7680 — HF rows now carry
                                    // both raw int16 and corrected floats
    size_t            len     = 0;
    volatile bool     pending = false;
    SemaphoreHandle_t mtx     = nullptr;
};
Outbox teleOut;   // 5 Hz main telemetry
Outbox hfOut;     // 1 Hz HF IMU batch

// ════════════════════════════════════════════════════════════════
//  OFFLINE BUFFER IMPLEMENTATION — only compiled when MODE > 0.
//  Queues to LittleFS (JSON Lines: one packet per line) and drains
//  a few lines per modemTask pass once mqtt_up is true again.
//  Called ONLY from modemTask (Core 0) — no cross-core locking needed
//  beyond the existing outbox mutexes already held by the caller.
// ════════════════════════════════════════════════════════════════
#if OFFLINE_BUFFER_MODE > 0
bool fsReady=false;

bool offlineBufInit(){
    fsReady = LittleFS.begin(true);   // true = format if mount fails
    if(!fsReady){
        Serial.println("[buffer] LittleFS mount FAILED — offline buffering disabled.");
        return false;
    }
    Serial.println("[buffer] LittleFS ready.");
    return true;
}

// Append one frame as a JSON Line; enforce ring cap by trimming the
// oldest lines once the file exceeds maxLines. Trimming rewrites the
// file (acceptable: only happens while offline, at low Hz).
void offlineBufPush(const char* path, const char* payload, size_t len, int maxLines){
    if(!fsReady) return;
    File f = LittleFS.open(path, "a");
    if(!f) return;
    f.write((const uint8_t*)payload, len);
    f.write((const uint8_t*)"\n", 1);
    f.close();

    // Cheap line-count check: only do the expensive trim occasionally.
    static uint32_t lastTrimMs=0;
    if(millis()-lastTrimMs < 5000) return;
    lastTrimMs=millis();

    File rf = LittleFS.open(path, "r");
    if(!rf) return;
    int total=0;
    while(rf.available()){ if(rf.read()=='\n') total++; }
    rf.close();
    if(total<=maxLines) return;

    // Rewrite keeping only the last maxLines lines.
    File src = LittleFS.open(path, "r");
    if(!src) return;
    String all; all.reserve(8192);
    while(src.available()) all+=(char)src.read();
    src.close();
    int drop = total-maxLines;
    int idx=0;
    while(drop>0 && idx<(int)all.length()){
        if(all[idx]=='\n') drop--;
        idx++;
    }
    File dst = LittleFS.open(path, "w");
    if(dst){ dst.print(all.substring(idx)); dst.close(); }
}

// Drain up to `maxLines` queued frames to `topic`. Stops early if a
// publish fails (link is flaky again) so remaining lines stay queued.
void offlineBufDrain(const char* path, const char* topic, int maxLines){
    if(!fsReady || !mqtt_up) return;
    File f = LittleFS.open(path, "r");
    if(!f || f.size()==0) return;

    String remaining; remaining.reserve(8192);
    int sent=0;
    bool stop=false;
    while(f.available()){
        String line = f.readStringUntil('\n');
        if(line.length()==0) continue;
        if(stop || sent>=maxLines){
            remaining += line; remaining += "\n";
            continue;
        }
        if(mqttPubTracked(topic, line.c_str(), line.length())){
            sent++;
        } else {
            stop=true;             // link dropped mid-drain — keep the rest queued
            remaining += line; remaining += "\n";
        }
    }
    f.close();
    File w = LittleFS.open(path, "w");
    if(w){ w.print(remaining); w.close(); }
    if(sent>0) Serial.printf("[buffer] drained %d queued frame(s) from %s\n",sent,path);
}
#endif // OFFLINE_BUFFER_MODE

// ── Link state (written by modemTask, read by Core 1 for LED) ──
volatile bool net_up      = false;   // LTE registered + attached
volatile bool mqtt_up     = false;   // CMQTT session connected
volatile int  lte_rssi    = -127;    // dBm from AT+CSQ
volatile bool hf_enabled  = true;    // false if stuck at 115200 baud

// ── Hardware ────────────────────────────────────────────────────
HardwareSerial Modem(2);
HardwareSerial GpsSerial(1);
TinyGPSPlus    gps;
MPU6050        imu_mpu;

// ════════════════════════════════════════════════════════════════
//  MAX17048 FUEL GAUGE — register-level driver (no external lib)
//  Transplanted from FuelGauge_Standalone.ino, adapted for v2.7.
//  Hardware: Cell 1 mid-tap → IC +  |  3.3V → VCC  |  ALRT → GPIO4
// ════════════════════════════════════════════════════════════════
// Pin + threshold constants (from TruTrackHardware.h)
#define FG_I2C_ADDR    0x36
#define FG_REG_VCELL   0x02
#define FG_REG_SOC     0x04
#define FG_REG_CONFIG  0x0C
#define FG_REG_VALRT   0x14
#define FG_REG_CRATE   0x16
#define FG_REG_VRESET  0x18
#define FG_REG_STATUS  0x1A
#define FG_REG_VERSION 0x08
#define FG_RCOMP0      0x97
#define FG_TEMPCO_UP  -0.5f
#define FG_TEMPCO_DN  -5.0f
#define FG_SOC_ALERT    10
#define FG_VMIN_MV    3000
#define FG_VMAX_MV    4250
static const int      MAX17048_ALRT_PIN    = 4;
static const float    BATTERY_LOW_PCT      = 15.0f;
static const float    BATTERY_CRITICAL_PCT =  5.0f;
static const uint32_t BATTERY_POLL_MS      = 5000;

struct BatteryState {
    float    voltage_V    = 0.0f;
    float    soc_pct      = -1.0f;
    float    crate_pct_hr = 0.0f;
    bool     low          = false;
    bool     critical     = false;
    bool     valid        = false;
};
BatteryState bat;
bool         fg_present  = false;   // set true if IC found at boot

bool fg_read(uint8_t reg, uint16_t &out){
    Wire.beginTransmission(FG_I2C_ADDR);
    Wire.write(reg);
    if(Wire.endTransmission(false)!=0) return false;
    if(Wire.requestFrom((uint8_t)FG_I2C_ADDR,(uint8_t)2)!=2) return false;
    out=((uint16_t)Wire.read()<<8)|Wire.read();
    return true;
}
bool fg_write(uint8_t reg, uint16_t val){
    Wire.beginTransmission(FG_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val>>8));
    Wire.write((uint8_t)(val&0xFF));
    return(Wire.endTransmission()==0);
}

bool fg_begin(){
    uint16_t ver=0;
    if(!fg_read(FG_REG_VERSION,ver)){
        Serial.println("[FUEL] MAX17048 not found — battery fields will stay placeholder.");
        return false;
    }
    Serial.printf("[FUEL] MAX17048 found. IC version: 0x%04X\n",ver);
    // Clear reset indicator
    uint16_t st=0; fg_read(FG_REG_STATUS,st);
    if(st&0x0100) fg_write(FG_REG_STATUS,st&~0x0100);
    // CONFIG: RCOMP=0x97, SOC alert at FG_SOC_ALERT %
    uint16_t cfg=0; fg_read(FG_REG_CONFIG,cfg);
    uint8_t athd=(uint8_t)(32-FG_SOC_ALERT)&0x1F;
    fg_write(FG_REG_CONFIG,((uint16_t)FG_RCOMP0<<8)|athd);
    // Voltage alert thresholds
    fg_write(FG_REG_VALRT,((uint16_t)(FG_VMIN_MV/20)<<8)|(FG_VMAX_MV/20));
    // VRESET: ~2.5V (captive 2S pack, cell never swapped)
    uint16_t vr=0; fg_read(FG_REG_VRESET,vr);
    fg_write(FG_REG_VRESET,((uint16_t)(63<<1)<<8)|(vr&0x00FF));
    Serial.printf("[FUEL] Init OK. SOC alert @%d%%, VALRT %d-%dmV\n",
                  FG_SOC_ALERT,FG_VMIN_MV,FG_VMAX_MV);
    return true;
}

void fg_update(){
    uint16_t rv=0,rs=0,rc=0;
    if(!fg_read(FG_REG_VCELL,rv)||!fg_read(FG_REG_SOC,rs)||!fg_read(FG_REG_CRATE,rc)){
        bat.valid=false; return;
    }
    bat.voltage_V   =(float)rv*78.125e-6f;
    bat.soc_pct     =constrain((float)(rs>>8)+(float)(rs&0xFF)/256.0f,0.0f,100.0f);
    bat.crate_pct_hr=(float)(int16_t)rc*0.208f;
    bat.low         =(bat.soc_pct<BATTERY_LOW_PCT);
    bat.critical    =(bat.soc_pct<BATTERY_CRITICAL_PCT);
    bat.valid       =true;
}

// Same as fg_update() but returns false on I2C failure — used by hot-swap logic.
bool fg_update_safe(){
    uint16_t rv=0,rs=0,rc=0;
    if(!fg_read(FG_REG_VCELL,rv)||!fg_read(FG_REG_SOC,rs)||!fg_read(FG_REG_CRATE,rc)){
        bat.valid=false; return false;
    }
    bat.voltage_V   =(float)rv*78.125e-6f;
    bat.soc_pct     =constrain((float)(rs>>8)+(float)(rs&0xFF)/256.0f,0.0f,100.0f);
    bat.crate_pct_hr=(float)(int16_t)rc*0.208f;
    bat.low         =(bat.soc_pct<BATTERY_LOW_PCT);
    bat.critical    =(bat.soc_pct<BATTERY_CRITICAL_PCT);
    bat.valid       =true;
    return true;
}

void fg_handleAlert(){
    if(digitalRead(MAX17048_ALRT_PIN)==HIGH) return;
    uint16_t st=0;
    if(!fg_read(FG_REG_STATUS,st)) return;
    Serial.printf("[FUEL] ALERT: status=0x%04X (VL=%d VH=%d HD=%d)\n",
                  st,!!(st&0x1000),!!(st&0x2000),!!(st&0x0400));
    uint16_t cfg=0;
    if(fg_read(FG_REG_CONFIG,cfg)) fg_write(FG_REG_CONFIG,cfg&~(1<<5));
}

void fg_updateTempComp(float temp_C){
    if(!bat.valid) return;
    float rf=(float)FG_RCOMP0+(temp_C>20.0f?(temp_C-20.0f)*FG_TEMPCO_UP
                                            :(temp_C-20.0f)*FG_TEMPCO_DN);
    uint8_t rcomp=(uint8_t)constrain(rf,0.0f,255.0f);
    uint16_t cfg=0;
    if(fg_read(FG_REG_CONFIG,cfg))
        fg_write(FG_REG_CONFIG,((uint16_t)rcomp<<8)|(cfg&0x00FF));
}
Eskf3D         eskf;
Preferences    prefs;

// ── Credentials ─────────────────────────────────────────────────
char     mqtt_host[64]={0};
uint16_t mqtt_port=1883;
char     mqtt_user[32]={0}, mqtt_pass_[64]={0};
char     apn[64]={0};

// ── Identity ────────────────────────────────────────────────────
String   device_id, boot_id, session_id, mqtt_topic, mqtt_hf_topic;
uint32_t boot_count=0;
String   reboot_reason;
String   carrier_name="";          // populated at netAttach from IMSI/COPS

// ── ESKF state ──────────────────────────────────────────────────
bool eskf_initialized=false, alignment_done=false, gnss_vel_used=false;
// DR governor state
float    dr_heading_rad    = 0.0f;
float    dr_speed_mps      = 0.0f;
uint32_t dr_last_update_ms = 0;
bool     dr_valid          = false;

// Auto yaw-gain calibration state (v30)
float    dr_yaw_gain           = 1.0f;  // learned gz_rps → yaw_rate scale+sign
uint32_t dr_yaw_gain_updates   = 0;
float    dr_last_gnss_heading_rad = 0.0f;
uint32_t dr_last_gnss_heading_ms  = 0;
bool     dr_course_cal_valid   = false;

float bias_ax=0,bias_ay=0,bias_az=0,bias_gx=0,bias_gy=0,bias_gz=0;
float calib_grav_x=0,calib_grav_y=0,calib_grav_z=0; // gravity vector from calibration
bool  imu_calibrated=false;
bool  ntp_synced=false;

// ── Latest IMU ──────────────────────────────────────────────────
int16_t raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz;
float   ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps;

// ── Timers ──────────────────────────────────────────────────────
uint32_t lastPubMs=0,lastImuMs=0,alignStartMs=0;


// ════════════════════════════════════════════════════════════════
//  LED
// ════════════════════════════════════════════════════════════════
void setLed(LedMode mode){
    if(ledMode==mode)return;
    ledMode=mode; ledSubStep=0; ledLastMs=millis();
}
void ledUpdate(){
    if(ledMode==LED_SOLID){digitalWrite(LED_PIN,HIGH);return;}
    if(ledMode==LED_DOUBLE_BLINK){
        uint32_t now=millis();
        if(now-ledLastMs>=DBL_PATTERN[ledSubStep]){
            ledLastMs=now;
            ledSubStep=(ledSubStep+1)%DBL_STEPS;
            ledState=(ledSubStep==0||ledSubStep==2);
            digitalWrite(LED_PIN,ledState?HIGH:LOW);
        }
        return;
    }
    if(ledMode==LED_TRIPLE_BLINK){
        uint32_t now=millis();
        if(now-ledLastMs>=TRIPLE_PATTERN[ledSubStep]){
            ledLastMs=now;
            ledSubStep=(ledSubStep+1)%TRIPLE_STEPS;
            ledState=(ledSubStep==0||ledSubStep==2||ledSubStep==4);
            digitalWrite(LED_PIN,ledState?HIGH:LOW);
        }
        return;
    }
    uint16_t period;
    switch(ledMode){
        case LED_FAST_BLINK:   period=100; break;
        case LED_MEDIUM_BLINK: period=500; break;
        case LED_RAPID_BLINK:  period=50;  break;
        default:               period=200; break;
    }
    uint32_t now=millis();
    if(now-ledLastMs>=(uint32_t)period){
        ledLastMs=now; ledState=!ledState;
        digitalWrite(LED_PIN,ledState?HIGH:LOW);
    }
}
// ── PCB Status LEDs ────────────────────────────────────────────
// Three dedicated status LEDs on the custom PCB, driven independently
// from the single onboard ESP32 GPIO2 blink LED.
//   RED   GPIO14 (D14) — error / not ready
//   GREEN GPIO12 (D12) — driving / ESKF fully active
//   YELLOW GPIO13 (D13) — intermediate / warning state
#define LED_RED_PIN    14
#define LED_GREEN_PIN  12
#define LED_YELLOW_PIN 13

void syncStatusLeds(){
    bool red=false, green=false, yellow=false;

    if(!imu_calibrated){
        // IMU not ready — red only
        red=true;
    } else if(!net_up || !mqtt_up){
        // Network/MQTT down — red + yellow (attention needed)
        red=true; yellow=true;
    } else if(!eskf_initialized){
        bool haveFix = g_fix.valid && g_fix.totalSv>=5
                       && g_fix.hdop<=3.0 && g_fix.ageMs<=1500;
        if(!haveFix){
            // No GNSS fix yet — red
            red=true;
        } else {
            // Fix acquired, waiting for motion — yellow
            yellow=true;
        }
    } else if(!alignment_done){
        // ESKF initializing (2s alignment) — yellow
        yellow=true;
    } else {
        // Fully ready — check drive state
        float spd=sqrtf(g_fix.speedKnots>0?
                        (float)(g_fix.speedKnots*KN_TO_MPS*g_fix.speedKnots*KN_TO_MPS):0.0f);
        bool driving=(spd>1.0f);   // moving above 1 m/s = actively driving
        if(driving){
            green=true;            // GREEN: driving with full ESKF
        } else {
            green=true; yellow=true;  // GREEN+YELLOW: ready but stationary
        }
        // Battery critical — add red regardless of drive state
        if(bat.valid && bat.critical) red=true;
    }

    digitalWrite(LED_RED_PIN,    red    ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN,  green  ? HIGH : LOW);
    digitalWrite(LED_YELLOW_PIN, yellow ? HIGH : LOW);
}

void syncLed(){
    if(!net_up || !mqtt_up)  {setLed(LED_RAPID_BLINK); return;}
    if(!imu_calibrated)      {setLed(LED_FAST_BLINK);  return;}
    if(!eskf_initialized){
        bool haveFix = g_fix.valid && g_fix.totalSv>=5 && g_fix.hdop<=3.0
                       && g_fix.ageMs<=1500;
        setLed(haveFix ? LED_TRIPLE_BLINK : LED_MEDIUM_BLINK);
        return;
    }
    if(!alignment_done)      {setLed(LED_DOUBLE_BLINK);return;}
    setLed(LED_SOLID);
}


// ════════════════════════════════════════════════════════════════
//  AT COMMAND HELPERS  — called ONLY from modemTask (Core 0)
//  after setup() hands over the UART.
// ════════════════════════════════════════════════════════════════

void modemFlush(){
    while(Modem.available()) Modem.read();
}

#define MAX_MODEM_RESP 512

bool modemWaitAny(const char*a,const char*b,const char*c,const char*d,
                  uint32_t tms, String*out=nullptr){
    static char buf[MAX_MODEM_RESP];
    int n=0;
    memset(buf,0,sizeof(buf));
    uint32_t t0=millis();
    while(millis()-t0<tms){
        while(Modem.available()&&n<(int)(MAX_MODEM_RESP-1)){
            buf[n++]=(char)Modem.read();
            buf[n]='\0';
        }
        if(a&&strstr(buf,a)){if(out)*out=String(buf);return true;}
        if(b&&strstr(buf,b)){if(out)*out=String(buf);return true;}
        if(c&&strstr(buf,c)){if(out)*out=String(buf);return true;}
        if(d&&strstr(buf,d)){if(out)*out=String(buf);return true;}
        delay(2);
    }
    if(out)*out=String(buf);
    return false;
}

bool modemSend(const char*cmd, const char*expect="OK", uint32_t tms=3000){
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    String resp;
    bool got=modemWaitAny(expect,"ERROR","+CME ERROR","+CMS ERROR",tms,&resp);
    if(!got) return false;
    return resp.indexOf(expect)>=0;
}

// Reads the response for `cmd`, returning EARLY as soon as a terminal
// token (OK / ERROR / +CME ERROR) arrives. v2.4.1 blocked the full
// timeout on every query, capping the whole modem task at ~0.3 Hz.
String modemQuery(const char*cmd, uint32_t tms=3000){
    static char buf[MAX_MODEM_RESP];
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    int n=0; buf[0]='\0';
    uint32_t t0=millis();
    while(millis()-t0<tms){
        while(Modem.available()&&n<(int)(MAX_MODEM_RESP-1)){
            buf[n++]=(char)Modem.read();
            buf[n]='\0';
        }
        if(strstr(buf,"\r\nOK\r\n")||strstr(buf,"ERROR")) break;
        delay(2);
    }
    return String(buf);
}

bool modemAlive(){
    for(int i=0;i<8;i++){
        if(modemSend("AT","OK",1500)) return true;
        delay(1000);
    }
    return false;
}

// Detect which baud the modem is currently using, then push it to
// MODEM_BAUD_FAST. If the modem was NOT power-cycled since the last
// run it may already be at 921600 while the ESP rebooted — handle both.
bool modemBaudSetup(){
    const uint32_t tries[2]={MODEM_BAUD_BOOT,MODEM_BAUD_FAST};
    bool alive=false;
    for(int i=0;i<2&&!alive;i++){
        Modem.updateBaudRate(tries[i]);
        delay(50); modemFlush();
        for(int k=0;k<3;k++){
            if(modemSend("AT","OK",1000)){alive=true;break;}
        }
        if(alive) Serial.printf("[modem] Responding @ %lu baud\n",
                                (unsigned long)tries[i]);
    }
    if(!alive) return false;

    // Already fast? done.
    String ipr=modemQuery("AT+IPR?",2000);
    if(ipr.indexOf("921600")>=0){hf_enabled=true;return true;}

    // Switch to fast baud.
    char cmd[24];
    snprintf(cmd,sizeof(cmd),"AT+IPR=%lu",(unsigned long)MODEM_BAUD_FAST);
    if(modemSend(cmd,"OK",3000)){
        delay(100);
        Modem.updateBaudRate(MODEM_BAUD_FAST);
        delay(100); modemFlush();
        if(modemSend("AT","OK",2000)){
            Serial.printf("[modem] UART switched to %lu baud\n",
                          (unsigned long)MODEM_BAUD_FAST);
            hf_enabled=true;
            return true;
        }
        // switch failed — fall back
        Modem.updateBaudRate(MODEM_BAUD_BOOT);
        delay(100); modemFlush();
        modemSend("AT","OK",2000);
    }
    Serial.println("[modem] WARNING: stuck at 115200 baud — "
                   "HF IMU publishing DISABLED (link too slow).");
    hf_enabled=false;
    return true;
}


// ════════════════════════════════════════════════════════════════
//  MODEM HELPERS (shared) — extractAfterPrefix used by CFUN/PDP/CCLK
// ════════════════════════════════════════════════════════════════
static String extractAfterPrefix(const String&resp, const String&prefix){
    int p=resp.indexOf(prefix);
    if(p<0) return "";
    int s=p+prefix.length();
    int er=resp.indexOf('\r',s);
    int en=resp.indexOf('\n',s);
    int end=-1;
    if(er>=0&&en>=0) end=min(er,en);
    else if(er>=0)   end=er;
    else if(en>=0)   end=en;
    else             end=resp.length();
    String line=resp.substring(s,end); line.trim();
    return line;
}

// ════════════════════════════════════════════════════════════════
//  GNSS — Quectel L89HA, NMEA streaming via TinyGPSPlus (UART1)
// ════════════════════════════════════════════════════════════════
//
//  TinyGPSPlus.satellites.value() reports only the LAST GSV's count,
//  which undercounts on multi-constellation receivers (each system
//  emits its own GSV with its own talker ID). To recover the true
//  total — including NavIC/IRNSS — we tap the raw NMEA stream and
//  read the "satellites in view" field of each GSV sentence, keyed
//  by talker ID, refreshing once per GSV cycle.
//
//  GSV talker IDs:  GP=GPS  GL=GLONASS  GB/BD=BeiDou  GA=Galileo
//                   GI/IR=NavIC(IRNSS)  GQ=QZSS  GN=combined
//
//  We also feed every byte to TinyGPSPlus for lat/lon/alt/speed/
//  course/hdop, so both paths run off the same stream.

// Per-constellation "in view" counts, updated as GSV sentences arrive.
struct ConstSV { int gps=0, glo=0, bds=0, gal=0, irnss=0, qzss=0; };
ConstSV g_csv_acc;     // accumulating during current cycle
ConstSV g_csv_pub;     // last completed snapshot

// Minimal GSV sniffer fed one NMEA char at a time, in parallel with
// TinyGPSPlus. Captures field 3 (total sats in view) per talker.
static char   nmeaLine[120];
static uint8_t nmeaPos=0;

void gnssSniffGSV(const char* line){
    // line looks like: $GPGSV,3,1,11,...  (talker = chars 1-2)
    if(line[0] != '$') return;
    char sys[3]={line[1],line[2],0};
    // a GSV?  chars 3-5 == "GSV"
    if(!(line[3]=='G'&&line[4]=='S'&&line[5]=='V')) return;
    // field 3 = total in view: find 3rd comma
    int commas=0; const char* p=line;
    while(*p){ if(*p==','){ if(++commas==3){ p++; break; } } p++; }
    if(commas<3) return;
    int sv=atoi(p);
    if(sv<0||sv>40) return;
    if      (!strcmp(sys,"GP")) g_csv_acc.gps  =sv;
    else if (!strcmp(sys,"GL")) g_csv_acc.glo  =sv;
    else if (!strcmp(sys,"GB")||!strcmp(sys,"BD")) g_csv_acc.bds=sv;
    else if (!strcmp(sys,"GA")) g_csv_acc.gal  =sv;
    else if (!strcmp(sys,"GI")||!strcmp(sys,"IR")) g_csv_acc.irnss=sv;
    else if (!strcmp(sys,"GQ")) g_csv_acc.qzss =sv;
}

bool gnssInit(){
    Serial.println("[GNSS] L89HA init on UART1...");
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    GpsSerial.setRxBufferSize(2048);
    delay(500);   // let L89HA finish its own startup before we send anything

    // Ensure NMEA output is enabled on this port — guards against a
    // previously-saved bad config having disabled sentences.
    // $PSTMNMEACONFIG: port0, rate=1Hz, GGA+RMC+GSV+GSA enabled
    GpsSerial.println("$PSTMNMEACONFIG,0,1,1,1,1,1,0,0,0,0");
    delay(200);

    // Enable full constellation incl. NavIC/IRNSS.
    // $PSTMCFGCONST,<GPS>,<GLO>,<GAL>,<BDS>,<QZSS>,<IRNSS> (2=on)
    // NOTE: $PSTMSAVEPAR deliberately removed — sending it every boot
    // writes flash unnecessarily and can corrupt saved config mid-init.
    // The constellation command takes effect immediately without saving.
    GpsSerial.println("$PSTMCFGCONST,2,2,2,2,0,2");
    delay(200);

    Serial.println("[GNSS] NavIC/IRNSS enable sent. Streaming NMEA — waiting for first fix.");
    return true;
}

// Drain available NMEA bytes, feed TinyGPSPlus + GSV sniffer, and
// refresh g_fix when a new position fix completes. Runs on Core 1
// (loop) — non-blocking, no AT round-trips.
void gnssFeed(){
    bool newFix=false;
    while(GpsSerial.available()){
        char c=(char)GpsSerial.read();
        // raw line buffer for the GSV sniffer
        if(c=='\n'||c=='\r'){
            if(nmeaPos>0){ nmeaLine[nmeaPos]='\0'; gnssSniffGSV(nmeaLine); nmeaPos=0; }
        } else if(nmeaPos<sizeof(nmeaLine)-1){
            nmeaLine[nmeaPos++]=c;
        }
        // TinyGPSPlus parse
        if(gps.encode(c) && gps.location.isUpdated()) newFix=true;
    }

    // On a completed RMC/GGA cycle, GSV "in view" counts are fresh:
    // snapshot them and recompute total.
    if(newFix){
        g_csv_pub=g_csv_acc;
        int total = g_csv_pub.gps+g_csv_pub.glo+g_csv_pub.bds+
                    g_csv_pub.gal+g_csv_pub.irnss+g_csv_pub.qzss;

        if(xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(20)) == pdTRUE){
            g_fix.gpsSv  =g_csv_pub.gps;
            g_fix.gloSv  =g_csv_pub.glo;
            g_fix.bdsSv  =g_csv_pub.bds;
            g_fix.galSv  =g_csv_pub.gal;
            g_fix.irnssSv=g_csv_pub.irnss;   // NavIC — now exposed in telemetry
            // QZSS folded into total; struct has no dedicated field.
            g_fix.totalSv= total>0 ? total
                          : (int)gps.satellites.value();  // fallback
            // mode: 2D/3D from TinyGPS altitude validity
            g_fix.mode   = gps.altitude.isValid() ? 3 : 2;

            if(gps.location.isValid()){
                g_fix.lat        =gps.location.lat();
                g_fix.lon        =gps.location.lng();
                g_fix.altM       =gps.altitude.isValid()?gps.altitude.meters():0.0;
                g_fix.speedKnots =gps.speed.isValid()?gps.speed.knots():0.0;
                g_fix.courseDeg  =gps.course.isValid()?gps.course.deg():0.0;
                g_fix.hdop       =gps.hdop.isValid()?gps.hdop.hdop():99.0;
                g_fix.vdop       =0.0;   // not exposed by TinyGPSPlus
                g_fix.valid      =true;
                lastGnssFixMs    =millis();
            }
            g_fix.ageMs = g_fix.valid ? (millis()-lastGnssFixMs) : 999999;
            xSemaphoreGive(gnssMutex);
        }
    }
}


// ════════════════════════════════════════════════════════════════
//  4G NETWORK  (modemTask only)
// ════════════════════════════════════════════════════════════════

// Extract <stat> from a "+XXREG: <n>,<stat>" response. -1 if absent.
static int regStat(const String&r,const char*prefix){
    int p=r.indexOf(prefix);
    if(p<0) return -1;
    int c=r.indexOf(',',p);
    if(c<0) return -1;
    return r.substring(c+1,c+2).toInt();
}
static const char* regStatStr(int s){
    switch(s){
        case 0: return "not searching";
        case 1: return "registered(home)";
        case 2: return "searching";
        case 3: return "DENIED";
        case 4: return "unknown";
        case 5: return "registered(roam)";
        default:return "?";
    }
}

// Attach to LTE. Returns true when registered + PS attached.
// Bounded: gives up after ~60 s so GNSS polling resumes; will be
// retried on the next modemTask pass.
bool netAttach(){
    Serial.println("[4G] Attaching...");

    // Full functionality? CFUN=0/4 (minimum/airplane) blocks RF entirely.
    String cf=modemQuery("AT+CFUN?",3000);
    if(cf.indexOf("+CFUN: 1")<0){
        Serial.printf("[4G] CFUN not 1 (%s) — enabling full RF.\n",
                      extractAfterPrefix(cf,"+CFUN:").c_str());
        modemSend("AT+CFUN=1","OK",10000);
    }

    // SIM readiness after CFUN=1 is NOT instant — on cold boot or after
    // certain resets, CPIN? can legitimately report "not ready" for
    // anywhere from 2 to 15+ seconds while the SIM finishes init. The
    // previous code checked once after a fixed 2s delay and gave up,
    // which is why this sometimes got stuck retrying the whole CFUN
    // dance forever instead of just waiting a bit longer. Poll instead.
    bool simReady=false;
    uint32_t simT0=millis();
    while(millis()-simT0<15000){
        String pin=modemQuery("AT+CPIN?",3000);
        if(pin.indexOf("READY")>=0){ simReady=true; break; }
        delay(1000);
    }
    if(!simReady){
        Serial.println("[4G] SIM not ready after 15s poll (no SIM / PIN locked / hardware fault?).");
        return false;
    }

    // One-time SIM/operator info for diagnosis
    String imsi=modemQuery("AT+CIMI",3000);
    imsi.replace("OK",""); imsi.replace("\r",""); imsi.replace("\n","");
    imsi.trim();
    Serial.printf("[4G] IMSI:%s\n",imsi.c_str());

    modemSend("AT+COPS=0","OK",10000);   // force automatic operator selection

    // Reset band configuration to automatic — undoes any previously
    // saved CNBP bitmask that may have locked out Jio's bands.
    // AT+CNBP with all-zero mask = let modem decide automatically.
    modemSend("AT+CNBP=0x0000000000080000,0x0000000000000000","OK",3000);
    delay(300);

    // APN auto-detection by IMSI prefix (Indian carriers).
    // User-set APN in NVS always wins; auto-detect is the fallback.
    // Jio MUST have APN set explicitly — it rejects registration without it.
    // IMSI prefixes: 405857/405858/405859/405861-865 = Jio
    //                404010/404003/405010-012 = Airtel
    //                404005/405005/405030 = Vi/Vodafone
    //                404009/404060 = BSNL
    String detectedApn="";
    if(imsi.startsWith("40585")||imsi.startsWith("40586")||
       imsi.startsWith("40857")||imsi.startsWith("40858")){
        detectedApn="jionet"; carrier_name="Jio";
    } else if(imsi.startsWith("40401")||imsi.startsWith("40402")||
              imsi.startsWith("40410")||imsi.startsWith("40416")||
              imsi.startsWith("40420")||imsi.startsWith("40445")||
              imsi.startsWith("40449")||imsi.startsWith("40470")||
              imsi.startsWith("40492")||imsi.startsWith("40493")||
              imsi.startsWith("40494")||imsi.startsWith("40495")||
              imsi.startsWith("40496")||imsi.startsWith("40497")||
              imsi.startsWith("40498")||imsi.startsWith("40499")||
              imsi.startsWith("40550")||imsi.startsWith("40556")){
        detectedApn="airtelgprs.com"; carrier_name="Airtel";
    } else if(imsi.startsWith("40400")||imsi.startsWith("40404")||
              imsi.startsWith("40420")||imsi.startsWith("40450")||
              imsi.startsWith("40451")||imsi.startsWith("40452")||
              imsi.startsWith("40502")||imsi.startsWith("40520")){
        detectedApn="portalnmms"; carrier_name="Vi";
    } else if(imsi.startsWith("40406")||imsi.startsWith("40409")||
              imsi.startsWith("40420")){
        detectedApn="bsnlnet"; carrier_name="BSNL";
    }

    // NVS APN overrides auto-detect; if neither, use modem default.
    const char* activeApn=(strlen(apn)>0)?apn:
                          (detectedApn.length()>0?detectedApn.c_str():"");
    if(strlen(activeApn)>0){
        char cmd[96];
        snprintf(cmd,sizeof(cmd),"AT+CGDCONT=1,\"IP\",\"%s\"",activeApn);
        modemSend(cmd,"OK",3000);
        Serial.printf("[4G] APN set: %s (carrier:%s)\n",
                      activeApn,carrier_name.length()?carrier_name.c_str():"?");
    }

    // Wait for registration: EPS (LTE) preferred, accept CS/PS (2G/3G fallback).
    // Print live status every ~6 s so the failure mode is visible.
    bool reg=false;
    uint32_t t0=millis(), lastDiag=0;
    while(millis()-t0<60000){
        int e =regStat(modemQuery("AT+CEREG?",2000),"+CEREG:");
        int c =regStat(modemQuery("AT+CREG?", 2000),"+CREG:");
        int g =regStat(modemQuery("AT+CGREG?",2000),"+CGREG:");
        if(e==1||e==5||g==1||g==5){reg=true;break;}
        if(millis()-lastDiag>=6000){
            lastDiag=millis();
            String q=modemQuery("AT+CSQ",2000);
            int p=q.indexOf("+CSQ:");
            int csq=(p>=0)?q.substring(p+5).toInt():99;
            Serial.printf("[4G] waiting… CSQ:%d%s  LTE:%s  CS:%s  PS:%s\n",
                csq,(csq==99)?" (NO SIGNAL — check MAIN/LTE antenna!)":"",
                regStatStr(e),regStatStr(c),regStatStr(g));
            if(e==3||c==3||g==3){
                Serial.println("[4G] Registration DENIED — SIM has no service/"
                               "plan on this network, or IMEI blocked.");
                Serial.println("[4G] NOTE: A7672S IMEI is blocked by Jio. Use Airtel/Vi/BSNL.");
                break;
            }
        }
        delay(1500);
    }
    if(!reg){
        Serial.println("[4G] Network registration timeout.");
        return false;
    }

    if(!modemSend("AT+CGATT=1","OK",10000)){
        Serial.println("[4G] PS attach failed.");
        return false;
    }

    // Make sure PDP context 1 is active and report the IP — CMQTT needs
    // an active data bearer, and this surfaces APN problems immediately.
    modemSend("AT+CGACT=1,1","OK",15000);
    String ip=modemQuery("AT+CGPADDR=1",3000);
    Serial.printf("[4G] PDP addr: %s\n",
                  extractAfterPrefix(ip,"+CGPADDR:").c_str());

    String op=modemQuery("AT+COPS?",3000);
    int q1=op.indexOf('"'), q2=op.indexOf('"',q1+1);
    String opName=(q1>=0&&q2>q1)?op.substring(q1+1,q2):"?";
    if(carrier_name.length()==0) carrier_name=opName;  // IMSI detect wins if set
    Serial.printf("[4G] Registered on \"%s\".\n",opName.c_str());
    return true;
}

void csqUpdate(){
    String r=modemQuery("AT+CSQ",2000);
    int p=r.indexOf("+CSQ:");
    if(p<0) return;
    int v=r.substring(p+5).toInt();
    lte_rssi=(v>=0&&v<=31)?(-113+2*v):-127;
}

// NTP over LTE. Sets ESP RTC (UTC) via settimeofday.
// Rotates servers across attempts — carriers sometimes block pool.ntp.org.
bool cntpSync(){
    static const char* NTP_SERVERS[]={
        "time.google.com","in.pool.ntp.org","pool.ntp.org"};
    static uint8_t srvIdx=0;
    const char* srv=NTP_SERVERS[srvIdx];
    srvIdx=(srvIdx+1)%3;

    Serial.printf("[4G] NTP sync (%s)...",srv);
    char cmd[64];
    // tz arg is in quarter-hours: IST = +5:30 = 22
    snprintf(cmd,sizeof(cmd),"AT+CNTP=\"%s\",22",srv);
    modemSend(cmd,"OK",3000);
    modemFlush();
    Modem.print("AT+CNTP\r");
    String resp;
    if(!modemWaitAny("+CNTP: 0","+CNTP:","ERROR",nullptr,15000,&resp)
       || resp.indexOf("+CNTP: 0")<0){
        Serial.printf(" FAILED (%s)\n",resp.c_str());
        return false;
    }
    // Read back local clock: +CCLK: "yy/MM/dd,hh:mm:ss+22"
    String c=modemQuery("AT+CCLK?",3000);
    int p=c.indexOf('"');
    if(p<0){Serial.println(" FAILED (CCLK)");return false;}
    String s=c.substring(p+1);
    int yy=s.substring(0,2).toInt(), MM=s.substring(3,5).toInt(),
        dd=s.substring(6,8).toInt(), hh=s.substring(9,11).toInt(),
        mi=s.substring(12,14).toInt(), ss=s.substring(15,17).toInt();
    struct tm ti={0};
    ti.tm_year=2000+yy-1900; ti.tm_mon=MM-1; ti.tm_mday=dd;
    ti.tm_hour=hh; ti.tm_min=mi; ti.tm_sec=ss;
    // NOTE: mktime() interprets `ti` in the C library's local timezone.
    // No TZ is ever set on this device (setenv("TZ",...) is never called),
    // so libc's local time == UTC, and mktime() returns the IST wall-clock
    // value reinterpreted as a UTC timestamp. Subtracting GMT_OFFSET_SEC
    // below corrects that back to true UTC. This only works because TZ
    // stays unset — do not add a setenv("TZ",...) call anywhere without
    // re-deriving this math, or the clock will be wrong by 2x the offset.
    time_t local=mktime(&ti);
    if(local<1000000000){Serial.println(" FAILED (parse)");return false;}
    time_t utc=local-GMT_OFFSET_SEC;   // CCLK returned IST local time
    struct timeval tv={.tv_sec=utc,.tv_usec=0};
    settimeofday(&tv,nullptr);
    ntp_synced=true;
    Serial.printf(" OK — IST: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                  yy,MM,dd,hh,mi,ss);
    return true;
}


// ════════════════════════════════════════════════════════════════
//  MQTT over 4G  — A7672S built-in stack (CMQTT*)
// ════════════════════════════════════════════════════════════════
uint8_t mqttFailCount=0;

void mqttTeardownAT(){
    modemSend("AT+CMQTTDISC=0,60","OK",8000);
    modemSend("AT+CMQTTREL=0","OK",3000);
    modemSend("AT+CMQTTSTOP","OK",5000);
    mqtt_up=false;
}

bool mqttConnectAT(){
    Serial.println("[MQTT] Connecting (4G)...");

    // The modem may hold a stale CMQTT session from a previous ESP boot
    // (it isn't power-cycled when the ESP resets). Tear down
    // unconditionally so connect is idempotent — errors here are normal.
    modemSend("AT+CMQTTDISC=0,60","OK",8000);
    modemSend("AT+CMQTTREL=0","OK",3000);
    modemSend("AT+CMQTTSTOP","OK",5000);

    String resp;
    modemFlush();
    Modem.print("AT+CMQTTSTART\r");
    modemWaitAny("+CMQTTSTART: 0","ERROR","+CMQTTSTART:",nullptr,10000,&resp);
    if(resp.indexOf("+CMQTTSTART: 0")<0){
        Serial.printf("[MQTT] CMQTTSTART failed: %s\n",resp.c_str());
        return false;
    }

    String cid="tt-"+device_id; cid.replace(":","");
    char cmd[160];
    snprintf(cmd,sizeof(cmd),"AT+CMQTTACCQ=0,\"%s\",0",cid.c_str());
    if(!modemSend(cmd,"OK",5000)){
        Serial.println("[MQTT] CMQTTACCQ failed.");
        mqttTeardownAT();
        return false;
    }

    if(strlen(mqtt_user)>0)
        snprintf(cmd,sizeof(cmd),
            "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1,\"%s\",\"%s\"",
            mqtt_host,mqtt_port,mqtt_user,mqtt_pass_);
    else
        snprintf(cmd,sizeof(cmd),
            "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1",
            mqtt_host,mqtt_port);

    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    if(!modemWaitAny("+CMQTTCONNECT: 0,0","ERROR","+CMQTTCONNECT:",nullptr,
                     20000,&resp)
       || resp.indexOf("+CMQTTCONNECT: 0,0")<0){
        Serial.printf("[MQTT] connect failed: %s\n",resp.c_str());
        mqttTeardownAT();
        return false;
    }

    mqttFailCount=0;
    mqtt_up=true;
    Serial.printf("MQTT connected (4G). topic:%s  hf:%s\n",
                  mqtt_topic.c_str(),mqtt_hf_topic.c_str());
    return true;
}

// Publish one message, QoS 0. Binary-safe payload write.
bool mqttPublishAT(const char*topic,const char*payload,size_t len){
    char cmd[96];  // widened from 48 — headroom for longer future topic names
    String resp;

    snprintf(cmd,sizeof(cmd),"AT+CMQTTTOPIC=0,%u",(unsigned)strlen(topic));
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    if(!modemWaitAny(">","ERROR",nullptr,nullptr,3000,&resp)
       || resp.indexOf('>')<0) return false;
    Modem.write((const uint8_t*)topic,strlen(topic));
    if(!modemWaitAny("OK","ERROR",nullptr,nullptr,3000)) return false;

    snprintf(cmd,sizeof(cmd),"AT+CMQTTPAYLOAD=0,%u",(unsigned)len);
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    if(!modemWaitAny(">","ERROR",nullptr,nullptr,3000,&resp)
       || resp.indexOf('>')<0) return false;
    Modem.write((const uint8_t*)payload,len);
    if(!modemWaitAny("OK","ERROR",nullptr,nullptr,5000)) return false;

    modemFlush();
    Modem.print("AT+CMQTTPUB=0,0,60\r");
    if(!modemWaitAny("+CMQTTPUB: 0,0","ERROR","+CMQTTPUB:",nullptr,15000,&resp)
       || resp.indexOf("+CMQTTPUB: 0,0")<0) return false;
    return true;
}

// Wrapper with failure tracking → reconnect after 3 consecutive fails.
bool mqttPubTracked(const char*topic,const char*payload,size_t len){
    if(!mqtt_up) return false;
    if(mqttPublishAT(topic,payload,len)){
        mqttFailCount=0;
        return true;
    }
    if(++mqttFailCount>=3){
        Serial.println("[MQTT] 3 consecutive publish failures — reconnecting.");
        mqttTeardownAT();
        net_up=false;   // force re-check of LTE attach too
    }
    return false;
}


// ── Per-cycle update-status flags, exposed in telemetry ───────────
// Set by processGNSSUpdate()/runESKF() each cycle, read by
// publishTelemetry(). Lets a stored session be audited for gating-
// rejection cascades or wrong NHC/ZUPT behavior without re-reading code.
bool gnss_pos_applied = false;
bool gnss_pos_gated   = false;
bool nhc_applied       = false;
bool zupt_applied      = false;

// ── Yaw-init provenance — recorded once per boot at ESKF init ──────
String yaw_init_source     = "none";   // "course" | "none" — default_north
                                        // retired: init now waits for motion,
                                        // so yaw is always course-derived once set
float  yaw_init_speed_mps  = 0.0f;
float  yaw_init_course_deg = 0.0f;
float  yaw_init_roll_rad   = 0.0f;   // gravity-derived roll at ESKF init
float  yaw_init_pitch_rad  = 0.0f;   // gravity-derived pitch at ESKF init

// ════════════════════════════════════════════════════════════════
//  ESKF — feeds from GnssFix
// ════════════════════════════════════════════════════════════════
void tryInitESKF(){
    if(!g_fix.valid)           return;
    if(g_fix.totalSv<5)        return;
    if(g_fix.hdop>3.0)         return;
    if(g_fix.ageMs>1500)       return;

    // Wait for genuine motion before initializing. Initializing while
    // stationary forces yaw to a default-North guess that is usually
    // WRONG (the vehicle is rarely actually pointed North), and that
    // heading error then contaminates every IMU-integrated position
    // until enough GNSS corrections slowly drag it out — producing
    // exactly the "huge initial NIS that decays but never recovers"
    // signature seen in early drive-test replays. Waiting here means
    // the filter only ever initializes with a trustworthy course-
    // derived heading, at the cost of a short delay if the vehicle is
    // stationary when GPS first locks (this is intentional — see the
    // discussion that led to this fix).
    float spd_mps=(float)(g_fix.speedKnots*KN_TO_MPS);
    if(spd_mps<=1.0f){
        return;   // not moving yet — keep waiting, do NOT initialize
    }

    float lat=(float)g_fix.lat;
    float lon=(float)g_fix.lon;
    float alt=(float)g_fix.altM;
    eskf.initLLA(lat,lon,alt);
    yaw_init_speed_mps  = spd_mps;
    yaw_init_course_deg = (float)g_fix.courseDeg;
    float yaw=(float)(M_PI/2.0)-(float)(g_fix.courseDeg*DEG2RAD);

    // Derive roll and pitch from the gravity vector measured during IMU
    // calibration. This makes ESKF init correct for any device orientation —
    // flat on dashboard, mounted on windshield, clipped to vent, etc.
    // calib_grav_x/y/z are the raw mean accel readings before bias subtraction,
    // which equals gravity in body frame (since device was stationary).
    float gx=calib_grav_x, gy=calib_grav_y, gz=calib_grav_z;
    float gnorm=sqrtf(gx*gx+gy*gy+gz*gz);
    float roll_init=0.0f, pitch_init=0.0f;
    if(gnorm>0.5f){   // sanity check — if we have a valid gravity vector
        // Standard gravity-from-accelerometer formulas (NED-compatible, then to ENU)
        pitch_init = asinf(-gx/gnorm);          // pitch from X-axis gravity
        roll_init  = atan2f(gy, gz);             // roll from Y/Z gravity
    }
    eskf.setInitialAttitude(roll_init, pitch_init, yaw);
    yaw_init_source    = "course";
    yaw_init_roll_rad  = roll_init;
    yaw_init_pitch_rad = pitch_init;

    Serial.printf("ESKF init lat=%.6f lon=%.6f alt=%.1f yaw_src=%s spd=%.2f\n",
        lat,lon,alt,yaw_init_source.c_str(),spd_mps);
    alignStartMs=millis(); alignment_done=false; eskf_initialized=true;
}

void processGNSSUpdate(){
    if(!g_fix.valid)    return;
    if(g_fix.ageMs>1500)return;
    if(!eskf_initialized){ tryInitESKF(); return; }
    if(!alignment_done)   return;
    float lat=(float)g_fix.lat;
    float lon=(float)g_fix.lon;
    float alt=(float)g_fix.altM;
    uint64_t t_us=(uint64_t)millis()*1000ULL;

    EskfGnssStatus st = eskf.updateGnssLLA(t_us,lat,lon,alt);
    gnss_pos_applied = (st==EskfGnssStatus::APPLIED);
    gnss_pos_gated   = (st==EskfGnssStatus::GATED);

    gnss_vel_used=false;
    float spd_mps=(float)(g_fix.speedKnots*KN_TO_MPS);
    if(spd_mps>GNSS_VEL_GATE_MPS){
        float cr=(float)(g_fix.courseDeg*DEG2RAD);
        eskf.updateGnssVel(spd_mps*sinf(cr),spd_mps*cosf(cr),0.0f);
        gnss_vel_used=true;
    }
    // Save heading+speed for DR governor: convert GNSS course to ENU yaw
    // GNSS course: degrees clockwise from North → ENU yaw = pi/2 - course_rad
    if(spd_mps >= DR_MIN_SPEED_MPS){
        float cr = (float)(g_fix.courseDeg * DEG2RAD);
        dr_heading_rad    = (float)(M_PI/2.0f) - cr;
        dr_speed_mps      = spd_mps;
        if(dr_speed_mps > DR_MAX_SPEED_MPS) dr_speed_mps = DR_MAX_SPEED_MPS;
        dr_last_update_ms = millis();
        dr_valid          = true;
        // v30: update auto yaw-gain from this GNSS fix
        float gnss_heading_rad = (float)(M_PI/2.0f) - cr;
        updateDrYawGainFromGnss(gnss_heading_rad, spd_mps, millis());
    }
}


// ════════════════════════════════════════════════════════════════
//  MODEM TASK — Core 0. Sole owner of the A7672S UART after setup.
//  Responsibilities: LTE attach, NTP, MQTT session, GNSS @1 Hz,
//  outbox draining, signal-quality polling.
// ════════════════════════════════════════════════════════════════
void modemTask(void*){
    Serial.println("[modemTask] Started on Core 0.");
    uint32_t lastCsqMs=0, lastNetRetryMs=0, lastBeatMs=0;

    static bool wasDown=false;   // tracks link state across loop iterations

    for(;;){
        uint32_t now=millis();

        bool linkDown = (!net_up||!mqtt_up);

        // ── Heartbeat: never let the monitor go silent while down ─
        if(linkDown && now-lastBeatMs>=15000){
            lastBeatMs=now;
            Serial.printf("[link] net:%s mqtt:%s rssi:%d sats:%d — retrying…\n",
                net_up?"UP":"down",mqtt_up?"UP":"down",lte_rssi,g_fix.totalSv);
        }
        // ── Recovery: clear, explicit log the moment both come back up ──
        if(wasDown && !linkDown){
            Serial.printf("[link] RESTORED — net:UP mqtt:UP rssi:%d sats:%d\n",
                lte_rssi,g_fix.totalSv);
        }
        wasDown = linkDown;

        // ── Bring up LTE (retry every 5 s while down) ───────────
        if(!net_up && now-lastNetRetryMs>=5000){
            lastNetRetryMs=now;
            net_up=netAttach();
        }

        // ── Modem-stuck watchdog ─────────────────────────────────
        // No PWRKEY/reset pin is wired to the A7672S, so a modem that
        // hangs mid-session (CFUN reverts, SIM stops responding, UART
        // wedges) has no hardware recovery path. The only thing that
        // sometimes fixes this in practice is a full power cycle —
        // this reproduces that in software by rebooting the ESP32
        // (which also cold-resets the modem's power rail sequencing
        // on most carrier boards) if the link has been down for too
        // long without recovering on its own.
        static uint32_t netDownSinceMs=0;
        if(!net_up){
            if(netDownSinceMs==0) netDownSinceMs=now;
            if(now-netDownSinceMs>=120000){
                Serial.println("[watchdog] Modem stuck >120s with no recovery — rebooting.");
                Serial.flush();
                delay(200);
                ESP.restart();
            }
        } else {
            netDownSinceMs=0;   // link recovered — reset the stuck-timer
        }

        // ── NTP: retry every 60 s until synced (max 10 tries) ──
        static uint8_t  ntpTries=0;
        static uint32_t lastNtpMs=0;
        if(net_up && !ntp_synced && ntpTries<10 &&
           (ntpTries==0 || now-lastNtpMs>=60000)){
            lastNtpMs=now;
            ntpTries++;
            if(cntpSync()) ntpTries=10;
        }

        // ── MQTT session ────────────────────────────────────────
        if(net_up && !mqtt_up){
            if(!mqttConnectAT()){
                // verify LTE is still alive; if not, re-attach next pass
                String r=modemQuery("AT+CEREG?",2000);
                if(r.indexOf(",1")<0 && r.indexOf(",5")<0) net_up=false;
                // A7672S err=3 (sock connect fail) needs extra settle time
                // for the modem TCP stack to fully reset before retrying.
                // 5000ms instead of 3000ms eliminates the double-fail on
                // first connect after PDP attach (cold boot / reconnect).
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }

        // GNSS now runs on Core 1 (L89 NMEA stream) — modemTask is
        // pure 4G/MQTT. ESKF GNSS correction also moved to loop().

        // ── Signal quality @ 0.1 Hz ─────────────────────────────
        if(now-lastCsqMs>=CSQ_POLL_MS){
            lastCsqMs=now;
            csqUpdate();
        }

        // ── Drain outboxes (HF first — it's 1 Hz and big) ───────
        // While mqtt is down, push pending frames to the offline buffer
        // here instead of letting next cycle's latest-wins write
        // silently discard them. While mqtt is up, behave as before:
        // attempt publish, and only fall back to the buffer if the
        // publish attempt itself fails.
        if(hfOut.pending && hf_enabled){
            if(xSemaphoreTake(hfOut.mtx,0)==pdTRUE){
                if(mqtt_up){
                    bool ok=mqttPubTracked(mqtt_hf_topic.c_str(),
                                           hfOut.buf,hfOut.len);
#if OFFLINE_BUFFER_MODE >= 2
                    if(!ok) offlineBufPush("/hf.jsonl",hfOut.buf,hfOut.len,
                                           OFFLINE_BUFFER_MAX_HF);
#endif
                    if(!ok) Serial.println("HF publish FAILED.");
                }
#if OFFLINE_BUFFER_MODE >= 2
                else {
                    // Link down — queue instead of letting this frame
                    // be silently overwritten by the next cycle.
                    offlineBufPush("/hf.jsonl",hfOut.buf,hfOut.len,
                                   OFFLINE_BUFFER_MAX_HF);
                }
#endif
                hfOut.pending=false;
                xSemaphoreGive(hfOut.mtx);
            }
        }
        if(teleOut.pending){
            if(xSemaphoreTake(teleOut.mtx,0)==pdTRUE){
                size_t plen=teleOut.len;
                bool ok=false;
                if(mqtt_up){
                    ok=mqttPubTracked(mqtt_topic.c_str(),teleOut.buf,plen);
#if OFFLINE_BUFFER_MODE >= 1
                    if(!ok) offlineBufPush("/main.jsonl",teleOut.buf,plen,
                                           OFFLINE_BUFFER_MAX_MAIN);
#endif
                }
#if OFFLINE_BUFFER_MODE >= 1
                else {
                    // Link down — queue instead of letting this frame
                    // be silently overwritten by the next cycle.
                    offlineBufPush("/main.jsonl",teleOut.buf,plen,
                                   OFFLINE_BUFFER_MAX_MAIN);
                }
#endif
                teleOut.pending=false;
                xSemaphoreGive(teleOut.mtx);
                if(ok){
                    // keep the familiar log line
                    const char* ls=ledMode==LED_SOLID?"SOLID":
                        ledMode==LED_DOUBLE_BLINK?"DOUBLE":
                        ledMode==LED_TRIPLE_BLINK?"TRIPLE":
                        ledMode==LED_MEDIUM_BLINK?"MED":
                        ledMode==LED_RAPID_BLINK?"RAPID":"FAST";
                    Serial.printf("[%7lu] %4dB | sats:%d hdop:%.1f | %s | %s | led:%s | rssi:%d\n",
                        millis(),(int)plen,g_fix.totalSv,(float)g_fix.hdop,
                        (g_fix.valid&&g_fix.ageMs<2000)?"FIX":"NO_FIX",
                        eskf_initialized?(alignment_done?"READY":"ALIGN"):"WAIT",
                        ls,lte_rssi);
                } else if(mqtt_up) {
                    Serial.println("PUBLISH FAILED.");
                }
            }
        }

#if OFFLINE_BUFFER_MODE >= 1
        // Drain queued frames gradually once the link is back, so a long
        // outage doesn't burst-flood the broker the instant we reconnect.
        if(mqtt_up){
            offlineBufDrain("/main.jsonl", mqtt_topic.c_str(),
                            OFFLINE_BUFFER_DRAIN_PER_TICK);
#if OFFLINE_BUFFER_MODE >= 2
            offlineBufDrain("/hf.jsonl", mqtt_hf_topic.c_str(),
                            OFFLINE_BUFFER_DRAIN_PER_TICK);
#endif
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ════════════════════════════════════════════════════════════════
//  SYSTEM
// ════════════════════════════════════════════════════════════════
String resetReasonStr(){
    switch(esp_reset_reason()){
        case ESP_RST_POWERON:  return "POWERON_RESET";
        case ESP_RST_SW:       return "SW_RESET";
        case ESP_RST_PANIC:    return "PANIC_RESET";
        case ESP_RST_INT_WDT:  return "INT_WDT_RESET";
        case ESP_RST_TASK_WDT: return "TASK_WDT_RESET";
        case ESP_RST_BROWNOUT: return "BROWNOUT_RESET";
        default:               return "OTHER_RESET";
    }
}
uint64_t getEpochMs_IST(){
    if(!ntp_synced) return 0;
    struct timeval tv; gettimeofday(&tv,NULL);
    return (uint64_t)tv.tv_sec*1000ULL+tv.tv_usec/1000;
}
void loadCredentials(){
    prefs.begin("creds",true);
    prefs.getString("mqtt_host",mqtt_host,sizeof(mqtt_host));
    mqtt_port=prefs.getUShort("mqtt_port",1883);
    prefs.getString("mqtt_user",mqtt_user,sizeof(mqtt_user));
    prefs.getString("mqtt_pass",mqtt_pass_,sizeof(mqtt_pass_));
    prefs.getString("apn",apn,sizeof(apn));        // optional, new in v2.4
    prefs.end();
    if(strlen(mqtt_host)==0){
        Serial.println("FATAL: No NVS mqtt_host. Flash Provision_NVS first.");
        while(true) delay(1000);
    }
    Serial.printf("NVS OK — MQTT:%s:%d  APN:%s\n",
                  mqtt_host,mqtt_port,strlen(apn)?apn:"(auto)");
}
void initBootMeta(){
    prefs.begin("navmeta",false);
    boot_count=prefs.getUInt("boot_count",0)+1;
    prefs.putUInt("boot_count",boot_count);
    prefs.end();
    reboot_reason=resetReasonStr();
    uint64_t chip=ESP.getEfuseMac();
    uint32_t rnd=esp_random();
    char buf[40];
    snprintf(buf,sizeof(buf),"%08X-%08X-%08X",(uint32_t)(chip&0xFFFFFFFF),boot_count,rnd);
    boot_id=String(buf);
    Serial.printf("Boot #%d  reason:%s  id:%s\n",
        boot_count,reboot_reason.c_str(),boot_id.c_str());
}
// Same MAC string WiFi.macAddress() produced — read from efuse so the
// MQTT topics (nav/<mac>/...) are byte-identical to v2.1/v2.3 and no
// server-side change is needed. Works with the WiFi stack never started.
void initIdentity(){
    uint8_t mac[6];
    esp_read_mac(mac,ESP_MAC_WIFI_STA);
    char buf[18];
    snprintf(buf,sizeof(buf),"%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    device_id    =String(buf);
    session_id   =device_id+":"+boot_id;
    mqtt_topic   ="nav/"+device_id+"/eskf";
    mqtt_hf_topic="nav/"+device_id+"/imu_hf";
    Serial.printf("device_id:%s\ntopic:%s\nhf_topic:%s\n",
        device_id.c_str(),mqtt_topic.c_str(),mqtt_hf_topic.c_str());
}


// ════════════════════════════════════════════════════════════════
//  IMU
// ════════════════════════════════════════════════════════════════
bool imu_present=false;
uint32_t lastImuRetryMs=0;

// Returns true when the MPU answered and calibration completed.
// Non-fatal: loop() retries every 5 s until it succeeds.
// Keeps sampling until we see a clean still window (stddev < threshold)
// so motion during calibration doesn't corrupt the bias estimate.
bool initIMU(){
    Wire.beginTransmission(0x68);          // quick bus probe first
    if(Wire.endTransmission()!=0){
        Serial.println("MPU6050 not on I2C bus (0x68) — will retry.");
        return false;
    }
    imu_mpu.initialize();
    if(!imu_mpu.testConnection()){
        Serial.println("MPU6050 WHO_AM_I failed — will retry.");
        return false;
    }
    Serial.println("MPU6050 OK.");
    imu_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    imu_mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    // Motion-robust calibration: collect windows of N samples.
    // If the accel magnitude stddev exceeds threshold (device is moving),
    // discard and retry — keeps trying until a clean still window is found.
    const int N=200;
    const float STDDEV_THRESH=0.08f;   // m/s² — anything above = motion detected
    int attempts=0;
    while(true){
        attempts++;
        double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
        double sax2=0,say2=0,saz2=0;
        for(int i=0;i<N;i++){
            int16_t ax,ay,az,gx,gy,gz;
            imu_mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
            float fax=(float)ax/ACCEL_SENS*G_MPS2;
            float fay=(float)ay/ACCEL_SENS*G_MPS2;
            float faz=(float)az/ACCEL_SENS*G_MPS2;
            sax+=fax; say+=fay; saz+=faz;
            sax2+=fax*fax; say2+=fay*fay; saz2+=faz*faz;
            sgx+=gx; sgy+=gy; sgz+=gz;
            ledUpdate(); delay(10);
        }
        double mx=sax/N, my=say/N, mz=saz/N;
        double vx=sax2/N-mx*mx, vy=say2/N-my*my, vz=saz2/N-mz*mz;
        float stddev=sqrtf((float)(vx+vy+vz));
        if(stddev<STDDEV_THRESH || attempts>=10){
            if(attempts>1)
                Serial.printf("IMU calib: %d attempts, stddev=%.4f %s\n",
                    attempts, stddev, stddev>=STDDEV_THRESH?"(gave up, using best)":"(ok)");
            // Store raw gravity vector (before bias subtraction) for roll/pitch init
            calib_grav_x=(float)mx;
            calib_grav_y=(float)my;
            calib_grav_z=(float)mz;
            // IMPORTANT FIX:
            // Do NOT subtract the gravity vector from accel before feeding ESKF.
            // predict() rotates body accel to ENU and then subtracts gravity itself.
            // Removing gravity here AND using roll/pitch from calib_grav creates
            // artificial horizontal acceleration when the unit is tilted, which
            // appears as the cyan/green drift loop on the map.
            bias_ax=0.0f;
            bias_ay=0.0f;
            bias_az=0.0f;
            bias_gx=(float)(sgx/N)/GYRO_SENS*DEG2RAD;
            bias_gy=(float)(sgy/N)/GYRO_SENS*DEG2RAD;
            bias_gz=(float)(sgz/N)/GYRO_SENS*DEG2RAD;
            break;
        }
        Serial.printf("IMU calib: motion detected (stddev=%.4f), retrying...\n",stddev);
    }
    imu_calibrated=true;
    imu_present=true;
    Serial.printf("IMU bias ax=%.4f ay=%.4f az=%.4f gx=%.5f gy=%.5f gz=%.5f\n",
        bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz);
    Serial.printf("IMU gravity vector: [%.3f, %.3f, %.3f] m/s²\n",
        calib_grav_x, calib_grav_y, calib_grav_z);
    return true;
}
void readIMU(){
    imu_mpu.getMotion6(&raw_ax,&raw_ay,&raw_az,&raw_gx,&raw_gy,&raw_gz);
    ax_mps2=(raw_ax/ACCEL_SENS)*G_MPS2-bias_ax;
    ay_mps2=(raw_ay/ACCEL_SENS)*G_MPS2-bias_ay;
    az_mps2=(raw_az/ACCEL_SENS)*G_MPS2-bias_az;
    gx_rps=(raw_gx/GYRO_SENS)*DEG2RAD-bias_gx;
    gy_rps=(raw_gy/GYRO_SENS)*DEG2RAD-bias_gy;
    gz_rps=(raw_gz/GYRO_SENS)*DEG2RAD-bias_gz;
}

// ── Auto yaw-gain calibration ─────────────────────────────────────────────────
// During healthy GNSS, learns the ratio of GNSS course-rate to gz_rps so the
// DR governor self-corrects for MPU6050 mounting sign and scale errors.
void updateDrYawGainFromGnss(float heading_rad, float speed_mps, uint32_t nowMs){
    if(!DR_AUTO_YAW_GAIN_ENABLE) return;
    if(speed_mps < DR_CAL_SPEED_MIN_MPS){
        dr_last_gnss_heading_rad = heading_rad;
        dr_last_gnss_heading_ms  = nowMs;
        dr_course_cal_valid      = true;
        return;
    }
    if(!dr_course_cal_valid){
        dr_last_gnss_heading_rad = heading_rad;
        dr_last_gnss_heading_ms  = nowMs;
        dr_course_cal_valid      = true;
        return;
    }
    uint32_t dtMs = nowMs - dr_last_gnss_heading_ms;
    if(dtMs < DR_CAL_MIN_DT_MS || dtMs > DR_CAL_MAX_DT_MS){
        dr_last_gnss_heading_rad = heading_rad;
        dr_last_gnss_heading_ms  = nowMs;
        return;
    }
    float dt = (float)dtMs * 0.001f;
    float dHeading = wrapPiLocal(heading_rad - dr_last_gnss_heading_rad);
    float headingRate = dHeading / dt;

    if(fabsf(headingRate) > DR_CAL_COURSE_RATE_MIN && fabsf(gz_rps) > DR_CAL_GYRO_MIN_RADPS){
        float measuredGain = headingRate / gz_rps;
        measuredGain = clampAbsLocal(measuredGain, DR_YAW_GAIN_MIN_ABS, DR_YAW_GAIN_MAX_ABS);
        dr_yaw_gain  = (1.0f - DR_YAW_GAIN_ALPHA) * dr_yaw_gain + DR_YAW_GAIN_ALPHA * measuredGain;
        dr_yaw_gain  = clampAbsLocal(dr_yaw_gain, DR_YAW_GAIN_MIN_ABS, DR_YAW_GAIN_MAX_ABS);
        dr_yaw_gain_updates++;
    }
    dr_last_gnss_heading_rad = heading_rad;
    dr_last_gnss_heading_ms  = nowMs;
}
// ─────────────────────────────────────────────────────────────────────────────

static inline float clampAbsLocal(float x, float minAbs, float maxAbs){
    float ax = fabsf(x);
    if(ax < minAbs) ax = minAbs;
    if(ax > maxAbs) ax = maxAbs;
    return (x >= 0.0f) ? ax : -ax;
}

static inline float wrapPiLocal(float a){
    while(a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while(a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

void runESKF(){
    if(!eskf_initialized) return;
    if(!alignment_done&&millis()-alignStartMs>=ALIGN_MS){
        alignment_done=true;
        Serial.println("Alignment done.");
    }
    uint64_t t_us=(uint64_t)millis()*1000ULL;
    eskf.predict(t_us,ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps);

    // ── GNSS-outage DR governor ───────────────────────────────────────────────
    // When GNSS is stale, integrate gyro-z to rotate last GNSS heading and
    // blend that course/speed into the ESKF velocity to follow curves.
    // SIGN CHECK: gz_rps > 0 must correspond to left turn (ENU yaw increasing).
    // If the track curves the wrong way during outage, negate gz_rps below.
    if(DR_OUTAGE_ENABLE && alignment_done && dr_valid){
        uint32_t nowMs = millis();
        bool gnssFresh = (g_fix.valid && lastGnssFixMs!=0 &&
                          (nowMs - lastGnssFixMs) < DR_GNSS_STALE_MS);
        if(!gnssFresh){
            float dt_dr = (dr_last_update_ms == 0) ? 0.0f
                        : (float)(nowMs - dr_last_update_ms) * 0.001f;
            if(dt_dr > 0.0f && dt_dr < 0.50f){
                // v30: dr_yaw_gain = learned sign+scale from GNSS course-rate vs gz_rps
            dr_heading_rad  = wrapPiLocal(dr_heading_rad + dr_yaw_gain * gz_rps * dt_dr);
                dr_speed_mps   *= (1.0f - DR_SPEED_DECAY_PER_SEC * dt_dr);
                if(dr_speed_mps < 0.0f) dr_speed_mps = 0.0f;
                if(dr_speed_mps > DR_MAX_SPEED_MPS) dr_speed_mps = DR_MAX_SPEED_MPS;
                if(dr_speed_mps > DR_MIN_SPEED_MPS){
                    float tgt_vE = dr_speed_mps * cosf(dr_heading_rad);
                    float tgt_vN = dr_speed_mps * sinf(dr_heading_rad);
                    eskf.blendHorizontalVelocity(tgt_vE, tgt_vN,
                                                 DR_VEL_BLEND_GAIN,
                                                 DR_DR_SIGMA_MPS);
                }
            }
            dr_last_update_ms = nowMs;
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

    float cur_vE=0.0f, cur_vN=0.0f, cur_vU=0.0f;
    eskf.getVelocity(cur_vE,cur_vN,cur_vU);
    float state_speed=sqrtf(cur_vE*cur_vE+cur_vN*cur_vN);

    // NHC should not be applied at zero/low speed. It is also compiled OFF in
    // the current tuned config, but this guard makes it safe if enabled later.
    bool allowNHC = alignment_done && state_speed>NHC_STATE_SPEED_MIN_MPS;
    nhc_applied  = allowNHC ? eskf.updateNonHolonomic() : false;

    // ZUPT must be both IMU-static and speed-static. IMU-only static can be true
    // during smooth moving sections because gyro is low and accel norm is near 1g.
    float gnss_speed_mps = 999.0f;
    if(g_fix.valid && g_fix.ageMs<1500){
        gnss_speed_mps=(float)(g_fix.speedKnots*KN_TO_MPS);
    }
    bool allowZUPT = alignment_done &&
                     gnss_speed_mps < ZUPT_GNSS_SPEED_GATE_MPS &&
                     state_speed    < ZUPT_STATE_SPEED_GATE_MPS;
    bool isStatic = allowZUPT && eskf.isStatic(ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps);
    zupt_applied  = isStatic ? eskf.updateZeroVelocity() : false;
}


// ════════════════════════════════════════════════════════════════
//  OUTBOX FILLERS — Core 1. Zero-timeout mutex: if modemTask is
//  mid-publish on that outbox, this frame is simply dropped
//  (latest-wins) and the 100 Hz loop is never stalled.
// ════════════════════════════════════════════════════════════════

bool stageImuHf(){
    if(imu_hf_count==0||!mqtt_up||!hf_enabled) return false;
    if(xSemaphoreTake(hfOut.mtx,0)!=pdTRUE) return false;   // busy → caller keeps buffer
    int pos=0;
    // fs/gs = sensitivity (counts per g / counts per deg-s) so any
    // consumer can convert raw counts back to physical units without
    // needing firmware source — schema-locked, see PAYLOAD_SCHEMA.
    pos+=snprintf(hfOut.buf+pos,sizeof(hfOut.buf)-pos,
        "{\"d\":\"%s\",\"s\":\"%s\",\"v\":\"%s\",\"n\":%d,\"t0\":%lu,"
        "\"fs\":%.0f,\"gs\":%.0f,\"imu\":[",
        device_id.c_str(),session_id.c_str(),FW_VERSION,
        imu_hf_count,(unsigned long)imu_hf_buf[0].t_ms,
        ACCEL_SENS,GYRO_SENS);
    for(uint8_t i=0;i<imu_hf_count&&pos<(int)sizeof(hfOut.buf)-160;i++){
        if(i>0) hfOut.buf[pos++]=',';
        // Row: [t_ms, ax_raw,ay_raw,az_raw, gx_raw,gy_raw,gz_raw,
        //            ax,ay,az (corrected m/s^2), gx,gy,gz (corrected rad/s)]
        pos+=snprintf(hfOut.buf+pos,sizeof(hfOut.buf)-pos,
            "[%lu,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f]",
            (unsigned long)imu_hf_buf[i].t_ms,
            imu_hf_buf[i].ax_raw,imu_hf_buf[i].ay_raw,imu_hf_buf[i].az_raw,
            imu_hf_buf[i].gx_raw,imu_hf_buf[i].gy_raw,imu_hf_buf[i].gz_raw,
            imu_hf_buf[i].ax,imu_hf_buf[i].ay,imu_hf_buf[i].az,
            imu_hf_buf[i].gx,imu_hf_buf[i].gy,imu_hf_buf[i].gz);
    }
    if(pos<(int)sizeof(hfOut.buf)-3){
        hfOut.buf[pos++]=']'; hfOut.buf[pos++]='}'; hfOut.buf[pos]='\0';
    }
    hfOut.len=pos;
    hfOut.pending=true;
    xSemaphoreGive(hfOut.mtx);
    return true;
}

void stageTelemetry(){
    StaticJsonDocument<JSON_BUF> doc;
    doc["device_id"]    =device_id;
    doc["session_id"]   =session_id;
    doc["boot_id"]      =boot_id;
    doc["boot_count"]   =boot_count;
    doc["reboot_reason"]=reboot_reason;
    doc["fw_version"]   =FW_VERSION;
    doc["t_ms"]         =millis();
    uint64_t epoch=getEpochMs_IST();
    doc["t_epoch_ms"]   =epoch>0?epoch:(uint64_t)0;
    doc["ntp_synced"]   =ntp_synced;

    // Live-computed fix age — NOT g_fix.ageMs (which is only refreshed
    // when a new fix arrives and can go stale if GNSS stops updating
    // entirely). This is computed fresh at send time every cycle.
    uint32_t gnssAgeNow = (g_fix.valid && lastGnssFixMs!=0)
                          ? (millis()-lastGnssFixMs) : 999999;

    JsonObject status=doc.createNestedObject("status");
    status["eskf_init"]          =eskf_initialized;
    status["alignment_done"]     =alignment_done;
    status["gnss_fix"]           =g_fix.valid;
    status["gnss_vel_used"]      =gnss_vel_used;
    status["sats"]               =g_fix.totalSv;
    status["hdop"]               =(float)g_fix.hdop;
    status["mqtt_connected"]     =mqtt_up;
    status["lte_rssi_dbm"]       =lte_rssi;        // was wifi_rssi_dbm
    status["net_up"]             =net_up;
    status["carrier"]            =carrier_name.length()?carrier_name.c_str():"?";
    status["hf_enabled"]         =hf_enabled;
    status["ntp_synced"]         =ntp_synced;
    status["imu_calibrated"]     =imu_calibrated;
    status["eskf_config_version"]=ESKF_CFG_VER;
    status["battery_v"]          =bat.valid ? bat.voltage_V : 0.0f;
    status["battery_pct"]        =bat.valid ? (int)bat.soc_pct : -1;
    status["battery_low"]        =bat.valid ? bat.low      : false;
    status["battery_critical"]   =bat.valid ? bat.critical : false;
    status["battery_crate_pct_hr"]=bat.valid ? bat.crate_pct_hr : 0.0f;
    status["gnss_mode"]          =g_fix.mode;
    status["gnss_age_ms"]        =gnssAgeNow;      // live-computed, not cached
    status["sats_gps"]           =g_fix.gpsSv;
    status["sats_glo"]           =g_fix.gloSv;
    status["sats_bds"]           =g_fix.bdsSv;
    status["sats_gal"]           =g_fix.galSv;
    status["sats_irnss"]         =g_fix.irnssSv;

    {
        JsonObject gnss=doc.createNestedObject("gnss");
        bool fix=g_fix.valid&&gnssAgeNow<2000;
        float speedMps=fix?(float)(g_fix.speedKnots*KN_TO_MPS):0.0f;
        gnss["fix_valid"] =fix;
        gnss["mode"]      =g_fix.mode;
        gnss["sats"]      =g_fix.totalSv;
        gnss["hdop"]      =(float)g_fix.hdop;
        gnss["vdop"]      =(float)g_fix.vdop;
        gnss["age_ms"]    =(int)gnssAgeNow;        // live-computed, not cached
        gnss["lat"]       =fix?(float)g_fix.lat      :0.0f;
        gnss["lon"]       =fix?(float)g_fix.lon      :0.0f;
        gnss["alt"]       =fix?(float)g_fix.altM     :0.0f;
        gnss["speed"]     =speedMps;   // CANONICAL — server ingest reads this key
        gnss["speed_mps"] =speedMps;   // alias, identical value, explicit unit
        gnss["course"]    =fix?(float)g_fix.courseDeg:0.0f;
        gnss["fix_type"]  =fix?(g_fix.mode>=3?3:2)  :0;
        gnss["gps_sv"]    =g_fix.gpsSv;
        gnss["glo_sv"]    =g_fix.gloSv;
        gnss["bds_sv"]    =g_fix.bdsSv;
        gnss["gal_sv"]    =g_fix.galSv;
        gnss["irnss_sv"]  =g_fix.irnssSv;

        // Simple inference: any constellation with >=1 SV in view while
        // the fix is currently valid is listed as "used". Not derived
        // from GNGSA (exact per-satellite solution membership) — an
        // approximation good enough for display/diagnostics.
        JsonArray cu=gnss.createNestedArray("constellations_used");
        if(fix){
            if(g_fix.gpsSv  >0) cu.add("GPS");
            if(g_fix.gloSv  >0) cu.add("GLONASS");
            if(g_fix.bdsSv  >0) cu.add("BeiDou");
            if(g_fix.galSv  >0) cu.add("Galileo");
            if(g_fix.irnssSv>0) cu.add("NavIC");
        }
    }
    {
        JsonObject eskfObj=doc.createNestedObject("eskf");
        eskfObj["init_valid"]     =eskf_initialized;
        eskfObj["alignment_valid"]=alignment_done;
        eskfObj["dr_yaw_gain"]=dr_yaw_gain;
        eskfObj["dr_yaw_gain_updates"]=dr_yaw_gain_updates;

        // Yaw-init provenance — recorded once per boot, present whenever
        // the ESKF has been initialized at least once this boot.
        eskfObj["yaw_init_source"]      =yaw_init_source;
        eskfObj["yaw_init_speed_mps"]   =yaw_init_speed_mps;
        eskfObj["yaw_init_course_deg"]  =yaw_init_course_deg;
        eskfObj["yaw_init_roll_rad"]    =yaw_init_roll_rad;
        eskfObj["yaw_init_pitch_rad"]   =yaw_init_pitch_rad;

        if(eskf_initialized){
            float lat_e,lon_e,alt_e,vE_e,vN_e,vU_e,roll_e,pitch_e,yaw_e;
            eskf.getLLA(lat_e,lon_e,alt_e);
            eskf.getVelocity(vE_e,vN_e,vU_e);
            eskf.getEuler(roll_e,pitch_e,yaw_e);
            float dE,dN,dU,dVE,dVN,dVU;
            eskf.getLastPosInnovation(dE,dN,dU);
            eskf.getLastVelInnovation(dVE,dVN,dVU);
            eskfObj["lat"]=lat_e; eskfObj["lon"]=lon_e; eskfObj["alt"]=alt_e;
            eskfObj["vE"]=vE_e;   eskfObj["vN"]=vN_e;   eskfObj["vU"]=vU_e;
            eskfObj["roll"]=roll_e; eskfObj["pitch"]=pitch_e; eskfObj["yaw"]=yaw_e;

            // Per-cycle update accounting — which corrections actually
            // fired, and whether the GNSS position update was gated.
            // Lets a stored session be audited for a gating-rejection
            // cascade directly, instead of inferring it from NIS alone.
            eskfObj["gnss_pos_applied"]=gnss_pos_applied;
            eskfObj["gnss_pos_gated"]  =gnss_pos_gated;
            eskfObj["nhc_applied"]     =nhc_applied;
            eskfObj["zupt_applied"]    =zupt_applied;

            JsonObject innov=eskfObj.createNestedObject("innov");
            innov["pos_norm"]=sqrtf(dE*dE+dN*dN+dU*dU);
            innov["vel_norm"]=sqrtf(dVE*dVE+dVN*dVN+dVU*dVU);
            innov["dE"]=dE; innov["dN"]=dN; innov["dU"]=dU;
            innov["dVE"]=dVE; innov["dVN"]=dVN; innov["dVU"]=dVU;

            // Bias estimates — gyro AND accel. Accel bias was previously
            // invisible; a runaway accel bias is a real IMU-mistuning
            // failure mode that telemetry should be able to show.
            float bgx_e,bgy_e,bgz_e, bax_e,bay_e,baz_e;
            eskf.getGyroBias(bgx_e,bgy_e,bgz_e);
            eskf.getAccelBias(bax_e,bay_e,baz_e);
            JsonArray gb2=eskfObj.createNestedArray("gyro_bias_radps");
            gb2.add(bgx_e); gb2.add(bgy_e); gb2.add(bgz_e);
            JsonArray ab=eskfObj.createNestedArray("accel_bias_mps2");
            ab.add(bax_e); ab.add(bay_e); ab.add(baz_e);

            float cov[15]; eskf.getCovDiag(cov);
            JsonArray P=eskfObj.createNestedArray("P");
            for(int i=0;i<15;i++) P.add(cov[i]);
        }
    }
    {
        JsonObject imuObj=doc.createNestedObject("imu");
        imuObj["calibrated"]=imu_calibrated;
        JsonArray ar=imuObj.createNestedArray("accel_raw");
        ar.add(raw_ax);ar.add(raw_ay);ar.add(raw_az);
        JsonArray gr=imuObj.createNestedArray("gyro_raw");
        gr.add(raw_gx);gr.add(raw_gy);gr.add(raw_gz);
        JsonArray am=imuObj.createNestedArray("accel_mps2");
        am.add(ax_mps2);am.add(ay_mps2);am.add(az_mps2);
        JsonArray gm=imuObj.createNestedArray("gyro_radps");
        gm.add(gx_rps);gm.add(gy_rps);gm.add(gz_rps);
        JsonArray gb=imuObj.createNestedArray("gyro_bias_radps");
        gb.add(bias_gx);gb.add(bias_gy);gb.add(bias_gz);
    }

    if(xSemaphoreTake(teleOut.mtx,0)!=pdTRUE) return;  // busy → drop frame
    teleOut.len=serializeJson(doc,teleOut.buf,sizeof(teleOut.buf));
    teleOut.pending=true;
    xSemaphoreGive(teleOut.mtx);
}


// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup(){
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,LOW);
    pinMode(LED_RED_PIN,   OUTPUT); digitalWrite(LED_RED_PIN,   LOW);
    pinMode(LED_GREEN_PIN, OUTPUT); digitalWrite(LED_GREEN_PIN, LOW);
    pinMode(LED_YELLOW_PIN,OUTPUT); digitalWrite(LED_YELLOW_PIN,LOW);
    // Boot indication: all three on briefly
    digitalWrite(LED_RED_PIN,HIGH); digitalWrite(LED_GREEN_PIN,HIGH);
    digitalWrite(LED_YELLOW_PIN,HIGH); delay(300);
    digitalWrite(LED_RED_PIN,LOW);  digitalWrite(LED_GREEN_PIN,LOW);
    digitalWrite(LED_YELLOW_PIN,LOW);
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.printf("  TRU-TRACK %s\n",FW_VERSION);
    Serial.println("  A7672S 4G + L89HA NavIC GNSS + MPU6050 + MAX17048");
    Serial.println("========================================");

    Wire.begin(21,22);
    delay(100);

    setLed(LED_FAST_BLINK);

#if OFFLINE_BUFFER_MODE > 0
    offlineBufInit();
#endif

    // ── Modem UART ──────────────────────────────────────────────
    Modem.begin(MODEM_BAUD_BOOT,SERIAL_8N1,MODEM_RX_PIN,MODEM_TX_PIN);
    Modem.setRxBufferSize(2048);
    Serial.printf("Modem UART2: RX=GPIO%d TX=GPIO%d\n",
        MODEM_RX_PIN,MODEM_TX_PIN);
    delay(3000);  // modem boot time after power-on

    if(!modemAlive() && !modemBaudSetup()){
        Serial.println("FATAL: Modem not responding. Check wiring + VDD+GND.");
        while(true) delay(1000);
    }
    modemBaudSetup();              // detect / switch to 921600
    modemSend("ATE0","OK",2000);
    modemSend("AT+CMEE=2","OK",2000);

    // ── GNSS: L89HA on UART1 (independent of modem) ─────────────
    gnssInit();

    // ── Identity + credentials (no network needed) ──────────────
    loadCredentials();
    initBootMeta();
    initIdentity();

    // ── Mutexes + modem task (Core 0 owns the UART from here) ───
    gnssMutex   = xSemaphoreCreateMutex();
    teleOut.mtx = xSemaphoreCreateMutex();
    hfOut.mtx   = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        modemTask,
        "modemTask",
        12288,      // GNSS parse + CMQTT handling needs headroom
        nullptr,
        1,
        nullptr,
        0           // Core 0
    );
    Serial.println("[modemTask] Launched on Core 0.");

    // ── IMU ─────────────────────────────────────────────────────
    initIMU();

    // ── Fuel gauge (MAX17048) ───────────────────────────────────
    // Non-blocking: if IC not found, battery fields stay as placeholders.
    // Wire bus already started above; ALRT pin needs pull-up.
    pinMode(MAX17048_ALRT_PIN, INPUT_PULLUP);
    fg_present = fg_begin();
    if(fg_present){
        fg_update();   // get first reading immediately
        Serial.printf("[FUEL] Cell 1: %.3fV  SOC: %.1f%%\n",
                      bat.voltage_V, bat.soc_pct);
    }

    Serial.println("\nLED (GPIO2): fast=boot  medium=wait_fix  triple=wait_motion  double=aligning  solid=ready  rapid=no_link");
    Serial.println("PCB LEDs: RED=error/not-ready  YELLOW=waiting/warning  GREEN=driving  GREEN+YELLOW=ready-stationary  RED+YELLOW=net-down");
    Serial.printf("HF IMU: %d samples/batch → %s\n",
        IMU_HF_BUF_SIZE,mqtt_hf_topic.c_str());
    Serial.println("Ready.\n");
}


// ════════════════════════════════════════════════════════════════
//  LOOP — Core 1. Never touches the modem UART.
// ════════════════════════════════════════════════════════════════
void loop(){
    ledUpdate();
    syncLed();
    syncStatusLeds();

    // ── GNSS: drain L89 NMEA every loop, run ESKF correction ────
    //    gnssFeed() refreshes g_fix from the NMEA stream; whenever a
    //    fresh fix lands, fold it into the ESKF. Prediction itself
    //    runs at 100 Hz below; this is the ~1 Hz GNSS correction.
    gnssFeed();
    {
        // Consume each GPS fix exactly once. lastGnssFixMs is stamped by
        // gnssFeed() whenever a fresh sentence completes a fix (~1 Hz);
        // comparing against the last-consumed value stops the 200ms poll
        // from re-applying one GPS update to the ESKF multiple times.
        static uint32_t lastConsumedFixMs=0;
        if(g_fix.valid && lastGnssFixMs!=lastConsumedFixMs){
            lastConsumedFixMs=lastGnssFixMs;
            processGNSSUpdate();
        }
    }

    // ── GNSS hot-swap watchdog ───────────────────────────────────
    // If no NMEA fix has arrived for >30s AND the module was previously
    // working, assume it was unplugged. Re-init UART + config commands.
    // ESKF keeps dead-reckoning while GNSS is absent — same as a tunnel.
    {
        static uint32_t lastGnssReinitMs=0;
        static bool     gnssEverWorked=false;
        if(g_fix.valid) gnssEverWorked=true;
        uint32_t gnssAge=lastGnssFixMs?(millis()-lastGnssFixMs):999999;
        if(gnssEverWorked && gnssAge>30000 && millis()-lastGnssReinitMs>30000){
            Serial.println("[GNSS] No data >30s — reinitializing (hot-swap?).");
            lastGnssReinitMs=millis();
            xSemaphoreTake(gnssMutex,pdMS_TO_TICKS(50));
            g_fix.valid=false; g_fix.ageMs=999999;
            xSemaphoreGive(gnssMutex);
            gnssInit();
        }
    }


    // ── IMU at 100 Hz (retry init every 5 s if absent) ──────────
    if(!imu_present){
        if(millis()-lastImuRetryMs>=5000){
            lastImuRetryMs=millis();
            initIMU();
        }
    } else if(millis()-lastImuMs>=IMU_PERIOD_MS){
        lastImuMs=millis();
        readIMU();
        // Hot-swap detection: if I2C probe fails, mark absent so
        // initIMU() re-runs automatically on the next 5s retry.
        Wire.beginTransmission(0x68);
        if(Wire.endTransmission()!=0){
            Serial.println("[IMU] Disconnect detected — will reinit.");
            imu_present=false; imu_calibrated=false;
            lastImuRetryMs=millis();
        } else {
            runESKF();
            if(imu_calibrated&&imu_hf_count<IMU_HF_BUF_SIZE){
                imu_hf_buf[imu_hf_count++]={
                    millis(),
                    raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,
                    ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps};
            }
        }
    }

    // ── HF IMU batch every 1 s → outbox ─────────────────────────
    // Only clear the buffer when staging actually succeeded — if the
    // outbox mutex was busy (or MQTT down), keep accumulating so this
    // second's samples aren't silently lost. IMU_HF_BUF_SIZE caps the
    // worst case if staging keeps failing for multiple seconds.
    if(millis()-lastImuHfMs>=IMU_HF_PUB_MS){
        lastImuHfMs=millis();
        if(stageImuHf()) imu_hf_count=0;
    }

    // ── Fuel gauge: poll every 5s, alert check every loop ───────
    static uint32_t lastFgMs=0;
    static uint32_t lastTempCompMs=0;
    static uint32_t lastFgProbeMs=0;
    uint32_t nowMs=millis();
    if(!fg_present){
        // Hot-swap: probe for MAX17048 every 5s and reinit if found
        if(nowMs-lastFgProbeMs>=5000){
            lastFgProbeMs=nowMs;
            uint16_t dummy=0;
            if(fg_read(FG_REG_VERSION,dummy)){
                Serial.println("[FUEL] MAX17048 reconnected — reinitializing.");
                fg_present=fg_begin();
                if(fg_present){ bat.valid=false; fg_update(); }
            }
        }
    } else {
        fg_handleAlert();
        if(nowMs-lastFgMs>=BATTERY_POLL_MS){
            lastFgMs=nowMs;
            if(!fg_update_safe()){   // returns false if I2C fails
                Serial.println("[FUEL] MAX17048 disconnect detected.");
                fg_present=false; bat.valid=false;
            }
        }
        if(nowMs-lastTempCompMs>=60000){
            lastTempCompMs=nowMs;
            int16_t rawT=imu_mpu.getTemperature();
            float chipTemp=(float)rawT/340.0f+36.53f;
            fg_updateTempComp(chipTemp);
        }
    }

    // ── Main telemetry at 5 Hz → outbox ─────────────────────────
    if(millis()-lastPubMs>=PUB_PERIOD_MS){
        lastPubMs=millis();
        stageTelemetry();
    }
}
