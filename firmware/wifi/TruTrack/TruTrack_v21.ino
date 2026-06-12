/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         TRU-TRACK v2.1 — Universal Firmware                 ║
 * ║   ESP32 + NEO-6M GNSS + MPU6050 IMU + 15-state ESKF        ║
 * ║                                                              ║
 * ║   IMU high-frequency: publishes 100 samples/sec on          ║
 * ║   nav/{mac}/imu_hf for ESKF tuning replay                   ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * LED States (GPIO2):
 *   Fast blink  100ms  → Boot / IMU calibrating / WiFi connecting
 *   Medium blink 500ms → WiFi OK, waiting for GNSS fix
 *   Double blink       → GNSS fix acquired, ESKF aligning (2s)
 *   Solid ON           → Fully ready — ESKF tracking
 *   Rapid blink  50ms  → WiFi or MQTT lost during operation
 *
 * Wiring:
 *   NEO-6M TX   → GNSS_RX_PIN (see config below)
 *   NEO-6M RX   → GNSS_TX_PIN (see config below)
 *   MPU6050 SDA → GPIO 21    MPU6050 SCL → GPIO 22
 *   Built-in LED  GPIO 2
 *
 * Required Libraries (Arduino Library Manager):
 *   TinyGPSPlus, PubSubClient, ArduinoJson v6, MPU6050 (Electronic Cats)
 *   + Eskf3D (copy from firmware/libraries/Eskf3D/ into Arduino/libraries/)
 */

// ════════════════════════════════════════════════════════════════
//  DEVICE CONFIGURATION — edit before flashing
// ════════════════════════════════════════════════════════════════
static const int GNSS_RX_PIN = 17;   // NEO-6M TX → GPIO16 (swap to 17 if no fix)
static const int GNSS_TX_PIN = 16;   // NEO-6M RX → GPIO17 (swap to 16 if no fix)
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
#include <TinyGPSPlus.h>
#include <MPU6050.h>
#include "Eskf3D.h"
#include "EskfConfig.h"

static const char* FW_VERSION    = "tt-v2.1";
static const char* ESKF_CFG_VER  = "tt-eskf-v1.0";

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
static const uint32_t PUB_PERIOD_MS    = 200;   // 5 Hz main topic
static const uint32_t IMU_PERIOD_MS    = 10;    // 100 Hz IMU
static const uint32_t IMU_HF_PUB_MS   = 1000;  // 1 Hz HF IMU batch topic
static const uint32_t MQTT_RETRY_MS   = 3000;
static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const uint32_t NTP_TIMEOUT_MS  = 5000;
static const uint32_t ALIGN_MS        = 2000;
static const long     GMT_OFFSET_SEC  = 19800;  // IST = UTC+5:30

// ── GNSS ───────────────────────────────────────────────────────
static const int   GNSS_BAUD          = 9600;
static const float GNSS_VEL_GATE_MPS  = 0.8f;

// ── IMU sensitivity ─────────────────────────────────────────────
static const float ACCEL_SENS = 16384.0f;
static const float GYRO_SENS  = 131.0f;
static const float G_MPS2     = 9.80665f;
static const float DEG2RAD    = M_PI / 180.0f;

// ── JSON / MQTT ─────────────────────────────────────────────────
static const size_t JSON_BUF    = 3072;   // main topic
static const size_t MQTT_BUF    = 10240;  // increased for HF IMU batch

// ── IMU HF buffer (100 samples × ~56 bytes JSON = ~5600 bytes) ──
#define IMU_HF_BUF_SIZE 100
struct ImuHfSample {
    uint32_t t_ms;
    float    ax, ay, az;
    float    gx, gy, gz;
};
ImuHfSample imu_hf_buf[IMU_HF_BUF_SIZE];
uint8_t     imu_hf_count = 0;
uint32_t    lastImuHfMs  = 0;

// ── Hardware ────────────────────────────────────────────────────
HardwareSerial GnssSerial(2);
TinyGPSPlus    gps;
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
//  LED — defined first so syncLed() can see all globals
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
        case LED_FAST_BLINK:  period=100;break;
        case LED_MEDIUM_BLINK:period=500;break;
        case LED_RAPID_BLINK: period=50; break;
        default:              period=200;break;
    }
    uint32_t now=millis();
    if(now-ledLastMs>=(uint32_t)period){
        ledLastMs=now; ledState=!ledState;
        digitalWrite(LED_PIN,ledState?HIGH:LOW);
    }
}
void syncLed(){
    if(WiFi.status()!=WL_CONNECTED)  {setLed(LED_RAPID_BLINK);return;}
    if(!mqtt.connected())             {setLed(LED_RAPID_BLINK);return;}
    if(!imu_calibrated)               {setLed(LED_FAST_BLINK); return;}
    if(!eskf_initialized)             {setLed(LED_MEDIUM_BLINK);return;}
    if(!alignment_done)               {setLed(LED_DOUBLE_BLINK);return;}
    setLed(LED_SOLID);
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
    if(!ntp_synced)return 0;
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
    }else{ntp_synced=false;Serial.println(" FAILED");}
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
        Serial.println("FATAL: No NVS credentials.");
        while(true)delay(1000);
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
    Serial.printf("Boot #%d  reason:%s  id:%s\n",boot_count,reboot_reason.c_str(),boot_id.c_str());
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
    Serial.printf(" OK  IP:%s  RSSI:%d\n",WiFi.localIP().toString().c_str(),WiFi.RSSI());
    device_id    = WiFi.macAddress();
    session_id   = device_id+":"+boot_id;
    mqtt_topic   = "nav/"+device_id+"/eskf";
    mqtt_hf_topic= "nav/"+device_id+"/imu_hf";
    Serial.printf("device_id : %s\ntopic     : %s\nhf_topic  : %s\n",
        device_id.c_str(),mqtt_topic.c_str(),mqtt_hf_topic.c_str());
}
void mqttConnect(){
    if(mqtt.connected())return;
    if(millis()-lastMqttRetryMs<MQTT_RETRY_MS)return;
    lastMqttRetryMs=millis();
    String cid="tt-"+device_id; cid.replace(":","");
    const char* u=strlen(mqtt_user)>0?mqtt_user:nullptr;
    const char* p=strlen(mqtt_pass_)>0?mqtt_pass_:nullptr;
    if(mqtt.connect(cid.c_str(),u,p))
        Serial.printf("MQTT connected. Topic:%s  HF:%s\n",mqtt_topic.c_str(),mqtt_hf_topic.c_str());
    else
        Serial.printf("MQTT failed rc=%d\n",mqtt.state());
}

// ── IMU ─────────────────────────────────────────────────────────
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
    Serial.println("IMU calibrating — keep still ~2s...");
    double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
    const int N=200;
    for(int i=0;i<N;i++){
        int16_t ax,ay,az,gx,gy,gz;
        imu_mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
        sax+=ax;say+=ay;saz+=az;sgx+=gx;sgy+=gy;sgz+=gz;
        ledUpdate();delay(10);
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

// ── IMU HF publish ──────────────────────────────────────────────
// Compact array format: saves ~30% space vs object format
// Each sample: [t_ms, ax, ay, az, gx, gy, gz]
// 100 samples ≈ 5600 chars — fits in MQTT_BUF=10240
void publishImuHf(){
    if(imu_hf_count==0||!mqtt.connected())return;

    // Static buffer to avoid stack allocation of 7KB
    static char buf[7680];
    int pos=0;

    pos+=snprintf(buf+pos,sizeof(buf)-pos,
        "{\"d\":\"%s\",\"s\":\"%s\",\"v\":\"%s\",\"n\":%d,\"t0\":%lu,\"imu\":[",
        device_id.c_str(),session_id.c_str(),FW_VERSION,
        imu_hf_count,(unsigned long)imu_hf_buf[0].t_ms);

    for(uint8_t i=0;i<imu_hf_count&&pos<(int)sizeof(buf)-80;i++){
        if(i>0)buf[pos++]=',';
        pos+=snprintf(buf+pos,sizeof(buf)-pos,
            "[%lu,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f]",
            (unsigned long)imu_hf_buf[i].t_ms,
            imu_hf_buf[i].ax,imu_hf_buf[i].ay,imu_hf_buf[i].az,
            imu_hf_buf[i].gx,imu_hf_buf[i].gy,imu_hf_buf[i].gz);
    }

    if(pos<(int)sizeof(buf)-3){
        buf[pos++]=']';
        buf[pos++]='}';
        buf[pos]='\0';
    }

    bool ok=mqtt.publish(mqtt_hf_topic.c_str(),(uint8_t*)buf,pos,false);
    if(!ok) Serial.println("HF publish FAILED (buffer too small?)");
}

// ── ESKF ────────────────────────────────────────────────────────
void tryInitESKF(){
    if(!gps.location.isValid())    return;
    if(gps.satellites.value()<5)   return;
    if(gps.hdop.hdop()>3.0f)      return;
    if(gps.location.age()>1500)    return;
    float lat=(float)gps.location.lat();
    float lon=(float)gps.location.lng();
    float alt=gps.altitude.isValid()?(float)gps.altitude.meters():0.0f;
    eskf.initLLA(lat,lon,alt);
    if(gps.course.isValid()&&gps.speed.mps()>1.0f){
        float yaw=(float)(M_PI/2.0)-(float)(gps.course.deg()*DEG2RAD);
        eskf.setInitialYaw(yaw);
    }
    Serial.printf("ESKF init lat=%.6f lon=%.6f alt=%.1f\n",lat,lon,alt);
    alignStartMs=millis(); alignment_done=false; eskf_initialized=true;
}
void runESKF(){
    if(!eskf_initialized)return;
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
void processGNSSUpdate(){
    if(!gps.location.isUpdated())return;
    if(!gps.location.isValid())  return;
    if(gps.location.age()>1500)  return;
    if(!eskf_initialized){tryInitESKF();return;}
    if(!alignment_done)  return;
    float lat=(float)gps.location.lat();
    float lon=(float)gps.location.lng();
    float alt=gps.altitude.isValid()?(float)gps.altitude.meters():0.0f;
    uint64_t t_us=(uint64_t)millis()*1000ULL;
    eskf.updateGnssLLA(t_us,lat,lon,alt);
    gnss_vel_used=false;
    if(gps.speed.isValid()&&gps.course.isValid()){
        float spd=(float)gps.speed.mps();
        if(spd>GNSS_VEL_GATE_MPS){
            float cr=(float)(gps.course.deg()*DEG2RAD);
            eskf.updateGnssVel(spd*sinf(cr),spd*cosf(cr),0.0f);
            gnss_vel_used=true;
        }
    }
}

// ── Main telemetry publish (5Hz, unchanged) ─────────────────────
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
    status["gnss_fix"]           =gps.location.isValid();
    status["gnss_vel_used"]      =gnss_vel_used;
    status["sats"]               =(int)gps.satellites.value();
    status["hdop"]               =gps.hdop.isValid()?(float)gps.hdop.hdop():99.9f;
    status["mqtt_connected"]     =mqtt.connected();
    status["wifi_rssi_dbm"]      =(WiFi.status()==WL_CONNECTED)?(int)WiFi.RSSI():-127;
    status["ntp_synced"]         =ntp_synced;
    status["imu_calibrated"]     =imu_calibrated;
    status["eskf_config_version"]=ESKF_CFG_VER;
    status["battery_v"]          =0.0f;
    status["battery_pct"]        =-1;

    {
        JsonObject gnss=doc.createNestedObject("gnss");
        bool fix=gps.location.isValid()&&gps.location.age()<2000;
        gnss["fix_valid"]=fix;
        gnss["sats"]     =(int)gps.satellites.value();
        gnss["hdop"]     =gps.hdop.isValid()?(float)gps.hdop.hdop():99.9f;
        gnss["age_ms"]   =(int)gps.location.age();
        gnss["lat"]      =fix?(float)gps.location.lat()    :0.0f;
        gnss["lon"]      =fix?(float)gps.location.lng()    :0.0f;
        gnss["alt"]      =fix&&gps.altitude.isValid()?(float)gps.altitude.meters():0.0f;
        gnss["speed"]    =fix&&gps.speed.isValid()?(float)gps.speed.mps():0.0f;
        gnss["course"]   =fix&&gps.course.isValid()?(float)gps.course.deg():0.0f;
        gnss["fix_type"] =fix?(gps.altitude.isValid()?3:2):0;
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
            for(int i=0;i<15;i++)P.add(cov[i]);
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
    bool fix_d=gps.location.isValid()&&gps.location.age()<2000;
    const char* ls=ledMode==LED_SOLID?"SOLID":ledMode==LED_DOUBLE_BLINK?"DOUBLE":
        ledMode==LED_MEDIUM_BLINK?"MED":ledMode==LED_RAPID_BLINK?"RAPID":"FAST";
    if(mqtt.publish(mqtt_topic.c_str(),(uint8_t*)buf,len,false))
        Serial.printf("[%7lu] %4dB | sats:%d hdop:%.1f | %s | %s | led:%s\n",
            millis(),(int)len,(int)gps.satellites.value(),
            gps.hdop.isValid()?gps.hdop.hdop():99.9f,
            fix_d?"FIX":"NO_FIX",
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
    Serial.printf("  TRU-TRACK %s (IMU HF enabled)\n",FW_VERSION);
    Serial.println("========================================");

    // I2C early — before WiFi — prevents clone GY-521 timeout
    Wire.begin(21,22);
    delay(100);

    setLed(LED_FAST_BLINK);
    loadCredentials();
    initBootMeta();
    connectWiFi();
    syncNTP();

    mqtt.setServer(mqtt_host,mqtt_port);
    mqtt.setBufferSize(MQTT_BUF);
    mqttConnect();

    GnssSerial.begin(GNSS_BAUD,SERIAL_8N1,GNSS_RX_PIN,GNSS_TX_PIN);
    Serial.printf("GNSS UART2: RX=GPIO%d TX=GPIO%d @ %d baud\n",
        GNSS_RX_PIN,GNSS_TX_PIN,GNSS_BAUD);

    initIMU();

    Serial.println("\nLED: fast=boot  medium=wait_fix  double=aligning  solid=ready  rapid=no_wifi");
    Serial.printf("HF IMU: batches of %d samples → %s  (every %ds)\n",
        IMU_HF_BUF_SIZE,mqtt_hf_topic.c_str(),IMU_HF_PUB_MS/1000);
}


// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop(){
    ledUpdate();
    syncLed();

    // Feed GNSS bytes
    while(GnssSerial.available()) gps.encode(GnssSerial.read());
    processGNSSUpdate();

    // IMU at 100Hz
    if(millis()-lastImuMs>=IMU_PERIOD_MS){
        lastImuMs=millis();
        readIMU();
        runESKF();

        // ── Buffer IMU sample for HF topic ──────────────────
        if(imu_calibrated && imu_hf_count < IMU_HF_BUF_SIZE){
            imu_hf_buf[imu_hf_count++]={
                millis(),
                ax_mps2, ay_mps2, az_mps2,
                gx_rps,  gy_rps,  gz_rps
            };
        }
    }

    // Publish HF IMU batch every 1 second
    if(millis()-lastImuHfMs >= IMU_HF_PUB_MS){
        lastImuHfMs=millis();
        publishImuHf();
        imu_hf_count=0;  // reset buffer after publish
    }

    // WiFi watchdog
    if(WiFi.status()!=WL_CONNECTED){
        Serial.println("WiFi lost — reconnecting.");
        connectWiFi(); syncNTP();
    }

    // MQTT reconnect
    if(!mqtt.connected()) mqttConnect();
    mqtt.loop();

    // Main telemetry at 5Hz
    if(millis()-lastPubMs>=PUB_PERIOD_MS){
        lastPubMs=millis();
        if(mqtt.connected()) publishTelemetry();
    }
}
