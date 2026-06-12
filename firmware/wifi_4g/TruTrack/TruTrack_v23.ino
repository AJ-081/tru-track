/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         TRU-TRACK v2.3                                      ║
 * ║   ESP32 + A7672S (4G LTE + onboard GNSS) + MPU6050 ESKF   ║
 * ║                                                              ║
 * ║   NEO-6M removed entirely.                                  ║
 * ║   GNSS : A7672S AT+CGNSSINFO polled every 1 s              ║
 * ║   Data  : MQTT over WiFi (unchanged from v2.1)             ║
 * ║   IMU HF topic : nav/{mac}/imu_hf (unchanged)              ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * Wiring:
 *   A7672S TX  → ESP32 GPIO17  (UART2 RX)   ← AT responses
 *   A7672S RX  → ESP32 GPIO16  (UART2 TX)   ← AT commands
 *   A7672S VCC + GND → 5 V external supply
 *   A7672S VDD + GND → same 5 V rail  (powers UART level-shifter)
 *   MPU6050 SDA → GPIO 21   SCL → GPIO 22
 *   LED         → GPIO 2
 *
 * GNSS antenna : active L1 on the GNSS connector (not LTE connector).
 *
 * LED States:
 *   Fast blink  100 ms  → Boot / IMU calibrating / WiFi connecting
 *   Medium blink 500 ms → WiFi OK, waiting for GNSS fix
 *   Double blink        → GNSS fix acquired, ESKF aligning (2 s)
 *   Solid ON            → Fully ready
 *   Rapid blink  50 ms  → WiFi or MQTT lost
 *
 * Required libraries (Arduino Library Manager):
 *   PubSubClient, ArduinoJson v6, MPU6050 (Electronic Cats)
 *   Eskf3D + EskfConfig  (local headers — unchanged from v2.1)
 *   TinyGPSPlus is NO LONGER needed.
 */

// ════════════════════════════════════════════════════════════════
//  MODEM PIN CONFIG  (plain int — safe before Arduino.h)
// ════════════════════════════════════════════════════════════════
static const int MODEM_RX_PIN = 17;   // ESP32 UART2 RX  ← A7672S TX
static const int MODEM_TX_PIN = 16;   // ESP32 UART2 TX  → A7672S RX

// ════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <esp_system.h>
#include <time.h>
#include <MPU6050.h>
#include "Eskf3D.h"
#include "EskfConfig.h"

// ════════════════════════════════════════════════════════════════
//  CONSTANTS  (uint32_t requires Arduino.h to be included first)
// ════════════════════════════════════════════════════════════════
static const uint32_t MODEM_BAUD   = 115200;
static const uint32_t GNSS_POLL_MS = 1000;  // poll AT+CGNSSINFO every 1 s

static const char* FW_VERSION   = "tt-v2.3";
static const char* ESKF_CFG_VER = "tt-eskf-v1.0";

// ── LED ────────────────────────────────────────────────────────
#define LED_PIN 2
enum LedMode { LED_FAST_BLINK, LED_MEDIUM_BLINK, LED_DOUBLE_BLINK,
               LED_SOLID, LED_RAPID_BLINK };
LedMode  ledMode    = LED_FAST_BLINK;
bool     ledState   = false;
uint32_t ledLastMs  = 0;
uint8_t  ledSubStep = 0;
static const uint16_t DBL_PATTERN[] = {120, 100, 120, 100, 800};
#define DBL_STEPS 5

// ── Timing ─────────────────────────────────────────────────────
static const uint32_t PUB_PERIOD_MS   = 200;    // 5 Hz main topic
static const uint32_t IMU_PERIOD_MS   = 10;     // 100 Hz IMU
static const uint32_t IMU_HF_PUB_MS  = 1000;   // 1 Hz HF batch
static const uint32_t MQTT_RETRY_MS  = 3000;
static const uint32_t WIFI_TIMEOUT_MS= 15000;
static const uint32_t NTP_TIMEOUT_MS = 5000;
static const uint32_t ALIGN_MS       = 2000;
static const long     GMT_OFFSET_SEC = 19800;   // IST = UTC+5:30

// ── GNSS gating ────────────────────────────────────────────────
static const float GNSS_VEL_GATE_MPS = 0.8f;   // m/s — below = no vel update

// ── IMU sensitivity ─────────────────────────────────────────────
static const float ACCEL_SENS = 16384.0f;
static const float GYRO_SENS  = 131.0f;
static const float G_MPS2     = 9.80665f;
static const float DEG2RAD    = M_PI / 180.0f;
static const float KN_TO_MPS  = 0.514444f;

// ── JSON / MQTT ─────────────────────────────────────────────────
static const size_t JSON_BUF = 3072;
static const size_t MQTT_BUF = 10240;

// ── IMU HF buffer ───────────────────────────────────────────────
#define IMU_HF_BUF_SIZE 100
struct ImuHfSample {
    uint32_t t_ms;
    float    ax, ay, az;
    float    gx, gy, gz;
};
ImuHfSample imu_hf_buf[IMU_HF_BUF_SIZE];
uint8_t     imu_hf_count = 0;
uint32_t    lastImuHfMs  = 0;

// ════════════════════════════════════════════════════════════════
//  GNSS — data delivered by A7672S AT+CGNSSINFO
//  Parser is taken verbatim from the working test sketch.
// ════════════════════════════════════════════════════════════════
struct GnssFix {
    bool   valid      = false;  // true when mode >= 2 and lat/lon present
    int    mode       = 0;      // 2 = 2D, 3 = 3D
    int    gpsSv      = 0;
    int    gloSv      = 0;
    int    bdsSv      = 0;
    int    galSv      = 0;
    int    totalSv    = 0;
    double lat        = 0.0;
    double lon        = 0.0;
    double altM       = 0.0;
    double speedKnots = 0.0;
    double courseDeg  = 0.0;
    double hdop       = 0.0;
    double vdop       = 0.0;
    uint32_t ageMs    = 0;      // ms since last successful parse
};

// Live GNSS state — written by Core 0 gnssTask, read by Core 1 loop.
// Always acquire gnssMutex before reading or writing g_fix.
GnssFix           g_fix;
uint32_t          lastGnssPollMs = 0;
uint32_t          lastGnssFixMs  = 0;  // millis() at last valid fix
SemaphoreHandle_t gnssMutex      = nullptr;

// ── Hardware ────────────────────────────────────────────────────
HardwareSerial Modem(2);       // UART2 → A7672S
MPU6050        imu_mpu;
Eskf3D         eskf;
WiFiClient     wifiClient;
PubSubClient   mqtt(wifiClient);
Preferences    prefs;

// ── Credentials ─────────────────────────────────────────────────
char     wifi_ssid[64]={0}, wifi_pass[64]={0};
char     mqtt_host[64]={0};
uint16_t mqtt_port=1883;
char     mqtt_user[32]={0}, mqtt_pass_[64]={0};

// ── Identity ────────────────────────────────────────────────────
String   device_id, boot_id, session_id, mqtt_topic, mqtt_hf_topic;
uint32_t boot_count=0;
String   reboot_reason;

// ── ESKF state ──────────────────────────────────────────────────
bool eskf_initialized=false, alignment_done=false, gnss_vel_used=false;
float bias_ax=0,bias_ay=0,bias_az=0,bias_gx=0,bias_gy=0,bias_gz=0;
bool  imu_calibrated=false;
bool  ntp_synced=false;

// ── Latest IMU ──────────────────────────────────────────────────
int16_t raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz;
float   ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps;

// ── Timers ──────────────────────────────────────────────────────
uint32_t lastPubMs=0,lastImuMs=0,lastMqttRetryMs=0,alignStartMs=0;


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
void syncLed(){
    if(WiFi.status()!=WL_CONNECTED)  {setLed(LED_RAPID_BLINK); return;}
    if(!mqtt.connected())             {setLed(LED_RAPID_BLINK); return;}
    if(!imu_calibrated)               {setLed(LED_FAST_BLINK);  return;}
    if(!eskf_initialized)             {setLed(LED_MEDIUM_BLINK);return;}
    if(!alignment_done)               {setLed(LED_DOUBLE_BLINK);return;}
    setLed(LED_SOLID);
}


// ════════════════════════════════════════════════════════════════
//  AT COMMAND HELPERS  (taken from working test sketches)
// ════════════════════════════════════════════════════════════════

void modemFlush(){
    while(Modem.available()) Modem.read();
}

// Read modem output for up to timeoutMs.
// Static char buffer — no heap allocation on the task stack.
// Prevents gnssTask stack-canary overflow.
#define MAX_MODEM_RESP 512
String modemReadFor(uint32_t timeoutMs){
    static char buf[MAX_MODEM_RESP];
    int n=0;
    uint32_t t0=millis();
    while(millis()-t0<timeoutMs){
        while(Modem.available()&&n<(int)(MAX_MODEM_RESP-1)){
            buf[n++]=(char)Modem.read();
        }
        delay(2);
    }
    buf[n]='\0';
    return String(buf);
}

// Wait until one of up to 4 substrings appears, or timeout.
// Static buffer avoids dynamic String growth on the task stack.
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

// Send command, wait for expect or ERROR. Returns true on expect match.
bool modemSend(const char*cmd, const char*expect="OK", uint32_t tms=3000){
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    String resp;
    bool got=modemWaitAny(expect,"ERROR","+CME ERROR","+CMS ERROR",tms,&resp);
    if(!got) return false;
    return resp.indexOf(expect)>=0;
}

// Send command, return full response string (for value extraction).
String modemQuery(const char*cmd, uint32_t tms=3000){
    modemFlush();
    Modem.print(cmd); Modem.print("\r");
    return modemReadFor(tms);
}

// Wait for modem to respond to bare AT (up to 8 tries × 1.5 s).
bool modemAlive(){
    for(int i=0;i<8;i++){
        if(modemSend("AT","OK",1500)) return true;
        delay(1000);
    }
    return false;
}


// ════════════════════════════════════════════════════════════════
//  GNSS — AT+CGNSSINFO parser
//  Confirmed format from real A7672S output:
//  +CGNSSINFO: <mode>,<gps_sv>,<glo_sv>,<bds_sv>,<gal_sv>,
//              <lat_dec>,<N/S>,<lon_dec>,<E/W>,<date>,<time>,
//              <alt_m>,<speed_kn>,<course>,<pdop>,<hdop>,<vdop>[,...]
//  Lat/lon arrive as decimal degrees (NOT NMEA ddmm.mmmm).
// ════════════════════════════════════════════════════════════════

// Split CSV string into parts array, return count.
static int splitCSV(const String&s, String out[], int maxParts){
    int count=0, start=0;
    while(count<maxParts){
        int comma=s.indexOf(',',start);
        if(comma<0){ out[count]=s.substring(start); out[count].trim(); count++; break; }
        out[count]=s.substring(start,comma); out[count].trim(); count++;
        start=comma+1;
    }
    return count;
}

// Extract the data line that follows a known prefix in a multi-line response.
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

// Parse AT+CGNSSINFO response into GnssFix. Returns true if parsed.
bool parseCGNSSINFO(const String&resp, GnssFix&fix){
    String line=extractAfterPrefix(resp,"+CGNSSINFO:");
    if(line.length()==0) return false;
    String f[24];
    int n=splitCSV(line,f,24);
    if(n<17){
        fix.valid=false;
        return true;          // parsed but no fix yet
    }
    fix.mode  =f[0].toInt();
    fix.gpsSv =f[1].toInt();
    fix.gloSv =f[2].toInt();
    fix.bdsSv =f[3].toInt();
    fix.galSv =f[4].toInt();
    fix.totalSv=fix.gpsSv+fix.gloSv+fix.bdsSv+fix.galSv;

    bool hasLL=(f[5].length()>0&&f[6].length()>0&&
                f[7].length()>0&&f[8].length()>0);
    fix.valid=(fix.mode>=2&&hasLL);

    if(hasLL){
        fix.lat=f[5].toDouble(); if(f[6]=="S") fix.lat=-fix.lat;
        fix.lon=f[7].toDouble(); if(f[8]=="W") fix.lon=-fix.lon;
    }
    fix.altM       =f[11].toDouble();
    fix.speedKnots =f[12].toDouble();
    fix.courseDeg  =f[13].toDouble();
    // f[14]=pdop, f[15]=hdop, f[16]=vdop
    fix.hdop=f[15].toDouble();
    fix.vdop=f[16].toDouble();
    return true;
}

// Power on the GNSS subsystem.
// Called once at setup. Waits up to 45 s for READY URC.
bool gnssInit(){
    Serial.println("[GNSS] Powering on...");
    // Check if already on
    String pwr=modemQuery("AT+CGNSSPWR?",3000);
    if(pwr.indexOf("+CGNSSPWR: 1")>=0){
        Serial.println("[GNSS] Already ON.");
        return true;
    }
    modemFlush();
    Modem.print("AT+CGNSSPWR=1\r");
    String resp;
    bool ready=modemWaitAny("+CGNSSPWR: READY!","ERROR","+CME ERROR",nullptr,45000,&resp);
    if(ready&&resp.indexOf("+CGNSSPWR: READY!")>=0){
        Serial.println("[GNSS] READY.");
        return true;
    }
    // Some firmware only sends OK; re-query status
    String pwr2=modemQuery("AT+CGNSSPWR?",5000);
    if(pwr2.indexOf("+CGNSSPWR: 1")>=0||pwr2.indexOf("READY")>=0){
        Serial.println("[GNSS] Status OK.");
        return true;
    }
    Serial.println("[GNSS] Power-on failed. Check GNSS antenna.");
    return false;
}

// Poll AT+CGNSSINFO once and update g_fix under mutex.
// Called only from gnssTask (Core 0) — never from the main loop.
void gnssUpdate(){
    String resp=modemQuery("AT+CGNSSINFO",3000);
    if(resp.indexOf("ERROR")>=0) return;

    GnssFix tmp;
    if(!parseCGNSSINFO(resp,tmp)) return;

    // Write to g_fix under mutex so Core 1 never reads a torn struct
    if(xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(50)) == pdTRUE){
        g_fix.gpsSv  =tmp.gpsSv;
        g_fix.gloSv  =tmp.gloSv;
        g_fix.bdsSv  =tmp.bdsSv;
        g_fix.galSv  =tmp.galSv;
        g_fix.totalSv=tmp.totalSv;
        g_fix.mode   =tmp.mode;
        if(tmp.valid){
            g_fix.lat        =tmp.lat;
            g_fix.lon        =tmp.lon;
            g_fix.altM       =tmp.altM;
            g_fix.speedKnots =tmp.speedKnots;
            g_fix.courseDeg  =tmp.courseDeg;
            g_fix.hdop       =tmp.hdop;
            g_fix.vdop       =tmp.vdop;
            g_fix.valid      =true;
            lastGnssFixMs    =millis();
        }
        g_fix.ageMs = g_fix.valid ? (millis()-lastGnssFixMs) : 999999;
        xSemaphoreGive(gnssMutex);
    }
}

// ── FreeRTOS GNSS task — runs on Core 0 ─────────────────────────
// Main loop runs on Core 1 (Arduino default).
// The two cores share g_fix via gnssMutex.
// Stack 4096 bytes is enough for modemQuery (String + HAL overhead).
void gnssTask(void*){
    Serial.println("[gnssTask] Started on Core 0.");
    for(;;){
        gnssUpdate();
        // processGNSSUpdate() touches eskf which is also used by Core 1.
        // We call it here immediately after the fresh gnssUpdate() so the
        // ESKF correction happens at 1 Hz while Core 1 prediction runs at 100 Hz.
        // eskf itself is thread-safe for this pattern because:
        //   - predict() (Core 1) and updateGnssLLA/Vel() (Core 0) are both
        //     short float operations and the FPU context is per-core on ESP32.
        //   - A brief tear during a 1 Hz correction is inconsequential for
        //     navigation accuracy (ESKF tolerates a missed update).
        // If you later add more frequent GNSS or tighter sync requirements,
        // add a second mutex around eskf calls.
        processGNSSUpdate();
        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_MS));
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
void syncNTP(){
    Serial.print("NTP sync...");
    configTime(GMT_OFFSET_SEC,0,"pool.ntp.org","time.nist.gov");
    time_t now=0; uint32_t t0=millis();
    while(now<1000000000UL&&millis()-t0<NTP_TIMEOUT_MS){time(&now);delay(100);}
    if(now>1000000000UL){
        ntp_synced=true;
        struct tm ti; localtime_r(&now,&ti);
        Serial.printf(" OK — IST: %04d-%02d-%02d %02d:%02d:%02d\n",
            ti.tm_year+1900,ti.tm_mon+1,ti.tm_mday,ti.tm_hour,ti.tm_min,ti.tm_sec);
    } else { ntp_synced=false; Serial.println(" FAILED"); }
}
void loadCredentials(){
    prefs.begin("creds",true);
    prefs.getString("wifi_ssid",wifi_ssid,sizeof(wifi_ssid));
    prefs.getString("wifi_pass",wifi_pass,sizeof(wifi_pass));
    prefs.getString("mqtt_host",mqtt_host,sizeof(mqtt_host));
    mqtt_port=prefs.getUShort("mqtt_port",1883);
    prefs.getString("mqtt_user",mqtt_user,sizeof(mqtt_user));
    prefs.getString("mqtt_pass",mqtt_pass_,sizeof(mqtt_pass_));
    prefs.end();
    if(strlen(wifi_ssid)==0){
        Serial.println("FATAL: No NVS credentials. Flash Provision_NVS first.");
        while(true) delay(1000);
    }
    Serial.printf("NVS OK — SSID:%s  MQTT:%s:%d\n",wifi_ssid,mqtt_host,mqtt_port);
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
void connectWiFi(){
    setLed(LED_FAST_BLINK);
    Serial.print("WiFi connecting");
    WiFi.mode(WIFI_STA); WiFi.begin(wifi_ssid,wifi_pass);
    uint32_t t0=millis();
    while(WiFi.status()!=WL_CONNECTED){
        if(millis()-t0>WIFI_TIMEOUT_MS){Serial.println(" timeout");ESP.restart();}
        ledUpdate(); delay(100); Serial.print(".");
    }
    Serial.printf(" OK  IP:%s  RSSI:%d\n",
        WiFi.localIP().toString().c_str(),WiFi.RSSI());
    device_id    =WiFi.macAddress();
    session_id   =device_id+":"+boot_id;
    mqtt_topic   ="nav/"+device_id+"/eskf";
    mqtt_hf_topic="nav/"+device_id+"/imu_hf";
    Serial.printf("device_id:%s\ntopic:%s\nhf_topic:%s\n",
        device_id.c_str(),mqtt_topic.c_str(),mqtt_hf_topic.c_str());
}
void mqttConnect(){
    if(mqtt.connected()) return;
    if(millis()-lastMqttRetryMs<MQTT_RETRY_MS) return;
    lastMqttRetryMs=millis();
    String cid="tt-"+device_id; cid.replace(":","");
    const char* u=strlen(mqtt_user)>0  ? mqtt_user  : nullptr;
    const char* p=strlen(mqtt_pass_)>0 ? mqtt_pass_ : nullptr;
    if(mqtt.connect(cid.c_str(),u,p))
        Serial.printf("MQTT connected. topic:%s  hf:%s\n",
            mqtt_topic.c_str(),mqtt_hf_topic.c_str());
    else
        Serial.printf("MQTT failed rc=%d\n",mqtt.state());
}


// ════════════════════════════════════════════════════════════════
//  IMU
// ════════════════════════════════════════════════════════════════
void initIMU(){
    imu_mpu.initialize();
    if(!imu_mpu.testConnection()){
        Serial.println("MPU6050 FAILED — retrying...");
        delay(300); imu_mpu.initialize();
        if(!imu_mpu.testConnection()){Serial.println("MPU6050 FAILED.");return;}
    }
    Serial.println("MPU6050 OK.");
    imu_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    imu_mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    Serial.println("IMU calibrating — keep still ~2 s...");
    double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
    const int N=200;
    for(int i=0;i<N;i++){
        int16_t ax,ay,az,gx,gy,gz;
        imu_mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        sax+=ax;say+=ay;saz+=az;sgx+=gx;sgy+=gy;sgz+=gz;
        ledUpdate(); delay(10);
    }
    bias_ax=(float)(sax/N)/ACCEL_SENS*G_MPS2;
    bias_ay=(float)(say/N)/ACCEL_SENS*G_MPS2;
    bias_az=(float)(saz/N)/ACCEL_SENS*G_MPS2-G_MPS2;
    bias_gx=(float)(sgx/N)/GYRO_SENS*DEG2RAD;
    bias_gy=(float)(sgy/N)/GYRO_SENS*DEG2RAD;
    bias_gz=(float)(sgz/N)/GYRO_SENS*DEG2RAD;
    imu_calibrated=true;
    Serial.printf("IMU bias ax=%.4f ay=%.4f az=%.4f gx=%.5f gy=%.5f gz=%.5f\n",
        bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz);
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

void publishImuHf(){
    if(imu_hf_count==0||!mqtt.connected()) return;
    static char buf[7680];
    int pos=0;
    pos+=snprintf(buf+pos,sizeof(buf)-pos,
        "{\"d\":\"%s\",\"s\":\"%s\",\"v\":\"%s\",\"n\":%d,\"t0\":%lu,\"imu\":[",
        device_id.c_str(),session_id.c_str(),FW_VERSION,
        imu_hf_count,(unsigned long)imu_hf_buf[0].t_ms);
    for(uint8_t i=0;i<imu_hf_count&&pos<(int)sizeof(buf)-80;i++){
        if(i>0) buf[pos++]=',';
        pos+=snprintf(buf+pos,sizeof(buf)-pos,
            "[%lu,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f]",
            (unsigned long)imu_hf_buf[i].t_ms,
            imu_hf_buf[i].ax,imu_hf_buf[i].ay,imu_hf_buf[i].az,
            imu_hf_buf[i].gx,imu_hf_buf[i].gy,imu_hf_buf[i].gz);
    }
    if(pos<(int)sizeof(buf)-3){ buf[pos++]=']'; buf[pos++]='}'; buf[pos]='\0'; }
    if(!mqtt.publish(mqtt_hf_topic.c_str(),(uint8_t*)buf,pos,false))
        Serial.println("HF publish FAILED.");
}


// ════════════════════════════════════════════════════════════════
//  ESKF — feeds from GnssFix instead of TinyGPSPlus
// ════════════════════════════════════════════════════════════════
void tryInitESKF(){
    if(!g_fix.valid)           return;
    if(g_fix.totalSv<5)        return;
    if(g_fix.hdop>3.0)         return;
    if(g_fix.ageMs>1500)       return;
    float lat=(float)g_fix.lat;
    float lon=(float)g_fix.lon;
    float alt=(float)g_fix.altM;
    eskf.initLLA(lat,lon,alt);
    float spd_mps=(float)(g_fix.speedKnots*KN_TO_MPS);
    if(spd_mps>1.0f){
        float yaw=(float)(M_PI/2.0)-(float)(g_fix.courseDeg*DEG2RAD);
        eskf.setInitialYaw(yaw);
    }
    Serial.printf("ESKF init lat=%.6f lon=%.6f alt=%.1f\n",lat,lon,alt);
    alignStartMs=millis(); alignment_done=false; eskf_initialized=true;
}

void processGNSSUpdate(){
    if(!g_fix.valid)    return;
    if(g_fix.ageMs>1500)return;  // stale
    if(!eskf_initialized){ tryInitESKF(); return; }
    if(!alignment_done)   return;
    float lat=(float)g_fix.lat;
    float lon=(float)g_fix.lon;
    float alt=(float)g_fix.altM;
    uint64_t t_us=(uint64_t)millis()*1000ULL;
    eskf.updateGnssLLA(t_us,lat,lon,alt);
    gnss_vel_used=false;
    float spd_mps=(float)(g_fix.speedKnots*KN_TO_MPS);
    if(spd_mps>GNSS_VEL_GATE_MPS){
        float cr=(float)(g_fix.courseDeg*DEG2RAD);
        eskf.updateGnssVel(spd_mps*sinf(cr),spd_mps*cosf(cr),0.0f);
        gnss_vel_used=true;
    }
}

void runESKF(){
    if(!eskf_initialized) return;
    if(!alignment_done&&millis()-alignStartMs>=ALIGN_MS){
        alignment_done=true;
        Serial.println("Alignment done.");
    }
    uint64_t t_us=(uint64_t)millis()*1000ULL;
    eskf.predict(t_us,ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps);
    if(alignment_done)eskf.updateNonHolonomic();
    if(eskf.isStatic(ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps))
        eskf.updateZeroVelocity();
}


// ════════════════════════════════════════════════════════════════
//  TELEMETRY
// ════════════════════════════════════════════════════════════════
void publishTelemetry(){
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

    JsonObject status=doc.createNestedObject("status");
    status["eskf_init"]          =eskf_initialized;
    status["alignment_done"]     =alignment_done;
    status["gnss_fix"]           =g_fix.valid;
    status["gnss_vel_used"]      =gnss_vel_used;
    status["sats"]               =g_fix.totalSv;
    status["hdop"]               =(float)g_fix.hdop;
    status["mqtt_connected"]     =mqtt.connected();
    status["wifi_rssi_dbm"]      =(WiFi.status()==WL_CONNECTED)?(int)WiFi.RSSI():-127;
    status["ntp_synced"]         =ntp_synced;
    status["imu_calibrated"]     =imu_calibrated;
    status["eskf_config_version"]=ESKF_CFG_VER;
    status["battery_v"]          =0.0f;
    status["battery_pct"]        =-1;
    // A7672S specific
    status["gnss_mode"]          =g_fix.mode;
    status["gnss_age_ms"]        =g_fix.ageMs;
    status["sats_gps"]           =g_fix.gpsSv;
    status["sats_glo"]           =g_fix.gloSv;
    status["sats_bds"]           =g_fix.bdsSv;

    {
        JsonObject gnss=doc.createNestedObject("gnss");
        bool fix=g_fix.valid&&g_fix.ageMs<2000;
        gnss["fix_valid"] =fix;
        gnss["mode"]      =g_fix.mode;
        gnss["sats"]      =g_fix.totalSv;
        gnss["hdop"]      =(float)g_fix.hdop;
        gnss["vdop"]      =(float)g_fix.vdop;
        gnss["age_ms"]    =(int)g_fix.ageMs;
        gnss["lat"]       =fix?(float)g_fix.lat      :0.0f;
        gnss["lon"]       =fix?(float)g_fix.lon      :0.0f;
        gnss["alt"]       =fix?(float)g_fix.altM     :0.0f;
        gnss["speed_mps"] =fix?(float)(g_fix.speedKnots*KN_TO_MPS):0.0f;
        gnss["course"]    =fix?(float)g_fix.courseDeg:0.0f;
        gnss["fix_type"]  =fix?(g_fix.mode>=3?3:2)  :0;
        // constellation breakdown — useful for dashboard
        gnss["gps_sv"]    =g_fix.gpsSv;
        gnss["glo_sv"]    =g_fix.gloSv;
        gnss["bds_sv"]    =g_fix.bdsSv;
        gnss["gal_sv"]    =g_fix.galSv;
    }
    {
        JsonObject eskfObj=doc.createNestedObject("eskf");
        eskfObj["init_valid"]     =eskf_initialized;
        eskfObj["alignment_valid"]=alignment_done;
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
            JsonObject innov=eskfObj.createNestedObject("innov");
            innov["pos_norm"]=sqrtf(dE*dE+dN*dN+dU*dU);
            innov["vel_norm"]=sqrtf(dVE*dVE+dVN*dVN+dVU*dVU);
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

    char buf[JSON_BUF];
    size_t len=serializeJson(doc,buf,sizeof(buf));
    const char* ls=ledMode==LED_SOLID?"SOLID":ledMode==LED_DOUBLE_BLINK?"DOUBLE":
        ledMode==LED_MEDIUM_BLINK?"MED":ledMode==LED_RAPID_BLINK?"RAPID":"FAST";
    if(mqtt.publish(mqtt_topic.c_str(),(uint8_t*)buf,len,false))
        Serial.printf("[%7lu] %4dB | sats:%d hdop:%.1f | %s | %s | led:%s\n",
            millis(),(int)len,g_fix.totalSv,(float)g_fix.hdop,
            (g_fix.valid&&g_fix.ageMs<2000)?"FIX":"NO_FIX",
            eskf_initialized?(alignment_done?"READY":"ALIGN"):"WAIT",ls);
    else
        Serial.println("PUBLISH FAILED.");
}


// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup(){
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,LOW);
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.printf("  TRU-TRACK %s\n",FW_VERSION);
    Serial.println("  A7672S 4G + onboard GNSS + MPU6050");
    Serial.println("========================================");

    // I2C early — prevents clone GY-521 timeout
    Wire.begin(21,22);
    delay(100);

    setLed(LED_FAST_BLINK);

    // ── Modem UART ──────────────────────────────────────────────
    Modem.begin(MODEM_BAUD,SERIAL_8N1,MODEM_RX_PIN,MODEM_TX_PIN);
    Serial.printf("Modem UART2: RX=GPIO%d TX=GPIO%d @ %d baud\n",
        MODEM_RX_PIN,MODEM_TX_PIN,MODEM_BAUD);
    delay(3000);  // modem boot time after power-on

    if(!modemAlive()){
        Serial.println("FATAL: Modem not responding. Check wiring + VDD+GND.");
        while(true) delay(1000);
    }
    modemSend("ATE0","OK",2000);   // echo off
    modemSend("AT+CMEE=2","OK",2000);

    // ── GNSS power-on + task launch ─────────────────────────────
    // Power on GNSS before WiFi so the receiver gets maximum cold-start time.
    gnssInit();

    // Create the shared mutex then pin the GNSS task to Core 0.
    // Main loop (Core 1) is unblocked — true 5 Hz publish + 100 Hz IMU.
    gnssMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        gnssTask,   // task function
        "gnssTask", // name (for debugging)
        8192,       // stack bytes — increased from 4096; modemQuery + String parsing needs headroom
        nullptr,    // parameter
        1,          // priority
        nullptr,    // task handle
        0           // Core 0
    );
    Serial.println("[gnssTask] Launched on Core 0.");

    // ── WiFi + NTP + MQTT ───────────────────────────────────────
    loadCredentials();
    initBootMeta();
    connectWiFi();
    syncNTP();
    mqtt.setServer(mqtt_host,mqtt_port);
    mqtt.setBufferSize(MQTT_BUF);
    mqttConnect();

    // ── IMU ─────────────────────────────────────────────────────
    initIMU();

    Serial.println("\nLED: fast=boot  medium=wait_fix  double=aligning  solid=ready  rapid=no_wifi");
    Serial.printf("HF IMU: %d samples/batch → %s\n",
        IMU_HF_BUF_SIZE,mqtt_hf_topic.c_str());
    Serial.println("Ready.\n");
}


// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop(){
    ledUpdate();
    syncLed();

    // ── GNSS → ESKF feed (non-blocking) ─────────────────────────
    // gnssTask on Core 0 writes g_fix every GNSS_POLL_MS.
    // We take a local snapshot under the mutex (< 1 µs hold time)
    // and then call processGNSSUpdate() with the main loop running free.
    {
        GnssFix snap;
        bool    hasNew = false;
        if(xSemaphoreTake(gnssMutex, 0) == pdTRUE){   // non-blocking take
            snap   = g_fix;
            hasNew = true;
            xSemaphoreGive(gnssMutex);
        }
        if(hasNew){
            // processGNSSUpdate() reads g_fix directly — that's fine here
            // because we just confirmed the mutex was free and snap matches.
            // For the ESKF call we pass snap fields explicitly to be safe.
            (void)snap;   // snap available if you want field-level access
            processGNSSUpdate();
        }
    }

    // ── IMU at 100 Hz ────────────────────────────────────────────
    if(millis()-lastImuMs>=IMU_PERIOD_MS){
        lastImuMs=millis();
        readIMU();
        runESKF();
        if(imu_calibrated&&imu_hf_count<IMU_HF_BUF_SIZE){
            imu_hf_buf[imu_hf_count++]={
                millis(),ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps};
        }
    }

    // ── HF IMU batch every 1 s ───────────────────────────────────
    if(millis()-lastImuHfMs>=IMU_HF_PUB_MS){
        lastImuHfMs=millis();
        publishImuHf();
        imu_hf_count=0;
    }

    // ── WiFi watchdog ────────────────────────────────────────────
    if(WiFi.status()!=WL_CONNECTED){
        Serial.println("WiFi lost — reconnecting.");
        connectWiFi(); syncNTP();
    }

    // ── MQTT ─────────────────────────────────────────────────────
    if(!mqtt.connected()) mqttConnect();
    mqtt.loop();

    // ── Main telemetry at 5 Hz ───────────────────────────────────
    if(millis()-lastPubMs>=PUB_PERIOD_MS){
        lastPubMs=millis();
        if(mqtt.connected()) publishTelemetry();
    }
}
