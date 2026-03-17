
/*
  Regenmengenmesser 🌧️ V4.9 für Wemos D1 mini / ioBroker (mqtt.0)
  - S49E Linear-Hall-Sensor an A0
  - Auto-Erkennung BME280 / BMP280 über Chip-ID
  - BMP280: Temperatur + Druck, keine Luftfeuchte
  - Luftfeuchte-Kachel wird bei BMP280 ausgeblendet
  - WLAN-Informationen im Fenster "System"
  - Obere Kacheln im selben Grid wie die unteren Fenster
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>

const char* WIFI_SSID     = "msg2";
const char* WIFI_PASSWORD = "1629969423743097";
const bool USE_STATIC_IP = true;
IPAddress LOCAL_IP(192, 168, 49, 77);
IPAddress GATEWAY(192, 168, 49, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS1(8, 8, 8, 8);
IPAddress DNS2(1, 1, 1, 1);

const char* MQTT_HOST     = "192.168.49.38";
const uint16_t MQTT_PORT  = 1886;
const char* MQTT_USER     = "mqttuser";
const char* MQTT_PASSWORD = "Beowolf503588";

const char* BASE_TOPIC    = "regenmesser";
const char* DEVICE_ID     = "regenmesser";
const char* DEVICE_NAME   = "Regenmengenmesser 🌧️";
const char* OTA_HOSTNAME  = "regenmesser";
const char* OTA_PASSWORD  = "";

const char* NTP_SERVER_1  = "pool.ntp.org";
const char* NTP_SERVER_2  = "time.nist.gov";
const char* TZ_INFO       = "CET-1CEST,M3.5.0,M10.5.0/3";

const bool ENABLE_FAILSAFE_RESTART = true;
const uint32_t FAILSAFE_RESTART_AFTER_MS = 30UL * 60UL * 1000UL;

static const uint8_t PIN_HALL = A0;
static const uint8_t PIN_SDA  = 4;
static const uint8_t PIN_SCL  = 5;

static const float MM_PER_TIP = 0.64f;

const bool ENABLE_I2C_SCAN_AT_BOOT = true;
uint8_t detectedSensorAddress = 0;
bool sensorOk = false;
bool isBMP280 = false;
bool isBME280 = false;

int hallCenter = 512;
int triggerDelta = 70;
int resetDelta = 30;
const uint32_t hallLockoutMs = 250UL;
const uint8_t hallSamples = 8;
const uint32_t hallSampleIntervalMs = 5UL;

static const uint32_t SENSOR_PUBLISH_MS = 60000UL;
static const uint32_t HEARTBEAT_MS      = 30000UL;
static const uint32_t WIFI_RETRY_MS     = 10000UL;
static const uint32_t MQTT_RETRY_MS     = 5000UL;
static const uint32_t HALL_TELEMETRY_MS = 10000UL;

enum HallState { HALL_UNKNOWN = 0, HALL_AWAY = 1, HALL_NEAR = 2 };

WiFiClient espClient;
PubSubClient mqtt(espClient);
Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
ESP8266WebServer webServer(80);

uint32_t g_tipsTotal = 0, g_tips1m = 0, g_tips10m = 0, g_tips60m = 0, g_tipsToday = 0;
uint32_t last1mWindowMs = 0, last10mWindowMs = 0, last60mWindowMs = 0;
uint32_t lastSensorPublishMs = 0, lastHeartbeatMs = 0, lastWifiRetryMs = 0, lastMqttRetryMs = 0;
uint32_t lastHallReadMs = 0, lastHallTelemetryMs = 0, bootMillis = 0, lastHealthyMs = 0, lastTipMs = 0;

float lastTempC = NAN, lastHumidityPct = NAN, lastPressureHpa = NAN;
bool timeSynced = false;
int lastHallRaw = 0, lastHallDelta = 0, lastHallMin = 1023, lastHallMax = 0, lastDayOfYear = -1;
HallState hallState = HALL_UNKNOWN;

String topic(const char* leaf){ return String(BASE_TOPIC)+"/"+leaf; }
void publishString(const char* leaf, const String& v, bool r=true){ mqtt.publish(topic(leaf).c_str(), v.c_str(), r); }
void publishBool(const char* leaf, bool v, bool r=true){ mqtt.publish(topic(leaf).c_str(), v?"true":"false", r); }
void publishUInt(const char* leaf, uint32_t v, bool r=true){ char b[16]; snprintf(b,sizeof(b),"%lu",(unsigned long)v); mqtt.publish(topic(leaf).c_str(), b, r); }
void publishLong(const char* leaf, long v, bool r=true){ char b[16]; snprintf(b,sizeof(b),"%ld",v); mqtt.publish(topic(leaf).c_str(), b, r); }
void publishFloat(const char* leaf, float v, uint8_t d=2, bool r=true){ if(isnan(v)) return; char b[24]; dtostrf(v,1,d,b); char* t=b; while(*t==' ') t++; mqtt.publish(topic(leaf).c_str(), t, r); }

String isoTimestamp(){
  time_t now=time(nullptr); if(now<100000) return "";
  struct tm tmNow; localtime_r(&now,&tmNow);
  char buf[32]; strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&tmNow); return String(buf);
}
String formatUptime(){
  uint32_t sec=(millis()-bootMillis)/1000UL, days=sec/86400UL; sec%=86400UL;
  uint32_t hours=sec/3600UL; sec%=3600UL; uint32_t mins=sec/60UL; sec%=60UL;
  char buf[32]; snprintf(buf,sizeof(buf),"%luT %02lu:%02lu:%02lu",(unsigned long)days,(unsigned long)hours,(unsigned long)mins,(unsigned long)sec); return String(buf);
}
String fmtFloat(float v, uint8_t dec=2, const String& fb="--"){ return isnan(v)?fb:String(v,dec); }
String statusBadge(bool ok, const String& a, const String& b){ return ok ? "<span class='badge ok'>"+a+"</span>" : "<span class='badge bad'>"+b+"</span>"; }
String hallStateText(){ if(hallState==HALL_NEAR) return "nah"; if(hallState==HALL_AWAY) return "weg"; return "unbekannt"; }
String sensorTypeText(){ if(isBME280) return "BME280"; if(isBMP280) return "BMP280"; return "unbekannt"; }
String sensorAddressText(){ if(detectedSensorAddress==0) return "nicht gefunden"; char buf[8]; snprintf(buf,sizeof(buf),"0x%02X", detectedSensorAddress); return String(buf); }

void scanI2C() {
  Serial.println("I2C Scan gestartet...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C Gerät gefunden auf 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("I2C Scan fertig.");
}

bool readChipIdAt(uint8_t address, uint8_t &chipId) {
  Wire.beginTransmission(address);
  Wire.write(0xD0);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)address, 1) != 1) return false;
  chipId = Wire.read();
  return true;
}

bool initEnvSensor() {
  uint8_t chipId = 0;
  const uint8_t addresses[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t addr = addresses[i];
    if (!readChipIdAt(addr, chipId)) continue;
    Serial.print("Chip-ID an ");
    Serial.print(addr == 0x76 ? "0x76" : "0x77");
    Serial.print(": 0x");
    if (chipId < 16) Serial.print("0");
    Serial.println(chipId, HEX);
    if (chipId == 0x60) {
      if (bme.begin(addr)) { detectedSensorAddress = addr; isBME280 = true; isBMP280 = false; Serial.println("BME280 erfolgreich initialisiert"); return true; }
    } else if (chipId == 0x58) {
      if (bmp.begin(addr)) { detectedSensorAddress = addr; isBMP280 = true; isBME280 = false; Serial.println("BMP280 erfolgreich initialisiert"); return true; }
    }
  }
  detectedSensorAddress = 0; isBMP280 = false; isBME280 = false; Serial.println("Kein BME280/BMP280 initialisiert"); return false;
}

int readHallFiltered(){
  long sum=0; int localMin=1023, localMax=0;
  for(uint8_t i=0;i<hallSamples;i++){ int v=analogRead(PIN_HALL); sum+=v; if(v<localMin) localMin=v; if(v>localMax) localMax=v; delay(1); }
  lastHallMin=localMin; lastHallMax=localMax; return (int)(sum/hallSamples);
}

void registerTip(){ g_tipsTotal++; g_tips1m++; g_tips10m++; g_tips60m++; g_tipsToday++; lastTipMs=millis(); }

HallState classifyHallState(int delta){
  if(abs(delta) >= triggerDelta) return HALL_NEAR;
  if(abs(delta) <= resetDelta) return HALL_AWAY;
  return HALL_UNKNOWN;
}

void handleHallRainGauge(){
  uint32_t now=millis();
  if((uint32_t)(now-lastHallReadMs) < hallSampleIntervalMs) return;
  lastHallReadMs = now;
  lastHallRaw = readHallFiltered();
  lastHallDelta = lastHallRaw - hallCenter;
  HallState newState = classifyHallState(lastHallDelta);
  if(hallState == HALL_UNKNOWN && newState != HALL_UNKNOWN){ hallState = newState; return; }
  if(newState == HALL_UNKNOWN) return;
  if(newState != hallState && (uint32_t)(now - lastTipMs) >= hallLockoutMs){ registerTip(); hallState = newState; }
}

void connectWifi(){ if(WiFi.status()==WL_CONNECTED) return; WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setAutoReconnect(true); if(USE_STATIC_IP) WiFi.config(LOCAL_IP,GATEWAY,SUBNET,DNS1,DNS2); WiFi.begin(WIFI_SSID,WIFI_PASSWORD); }
void setupTime(){ configTime(TZ_INFO,NTP_SERVER_1,NTP_SERVER_2); }
void updateTimeState(){ time_t now=time(nullptr); timeSynced=(now>100000); }

void handleMidnightReset(){
  updateTimeState(); if(!timeSynced) return; time_t now=time(nullptr); struct tm tmNow; localtime_r(&now,&tmNow);
  if(lastDayOfYear<0){ lastDayOfYear=tmNow.tm_yday; return; }
  if(tmNow.tm_yday!=lastDayOfYear){ g_tipsToday=0; lastDayOfYear=tmNow.tm_yday; if(mqtt.connected()){ publishUInt("rain/today_tips",0,true); publishFloat("rain/today_mm",0.0f,2,true); publishString("rain/last_reset",isoTimestamp(),true);} }
}

void setupOTA(){ ArduinoOTA.setHostname(OTA_HOSTNAME); if(strlen(OTA_PASSWORD)>0) ArduinoOTA.setPassword(OTA_PASSWORD); ArduinoOTA.begin(); }

void setupWebInfo(){
  webServer.on("/", [](){
    String html; html.reserve(7600);
    html += "<!doctype html><html lang='de'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><meta http-equiv='refresh' content='15'><title>Regenmengenmesser 🌧️</title><style>";
    html += R"rawliteral(:root{color-scheme:light;--color-base-100:oklch(95.814% 0 0);--color-base-200:oklch(89.107% 0 0);--color-base-300:oklch(82.4% 0 0);--color-base-content:oklch(19.162% 0 0);--color-primary:oklch(40.723% .161 17.53);--color-primary-content:oklch(88.144% .032 17.53);--color-secondary:oklch(61.676% .169 23.865);--color-secondary-content:oklch(12.335% .033 23.865);--color-accent:oklch(73.425% .094 60.729);--color-accent-content:oklch(14.685% .018 60.729);--color-neutral:oklch(54.367% .037 51.902);--color-neutral-content:oklch(90.873% .007 51.902);--color-info:oklch(69.224% .097 207.284);--color-info-content:oklch(13.844% .019 207.284);--color-success:oklch(60.995% .08 174.616);--color-success-content:oklch(12.199% .016 174.616);--color-warning:oklch(70.081% .164 56.844);--color-warning-content:oklch(14.016% .032 56.844);--color-error:oklch(53.07% .241 24.16);--color-error-content:oklch(90.614% .048 24.16);--radius-selector:1rem;--radius-field:.5rem;--radius-box:1rem;--size-selector:.25rem;--size-field:.25rem;--border:1px;--depth:1;--noise:0}*{box-sizing:border-box}body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--color-base-200);color:var(--color-base-content)}.wrap{max-width:1100px;margin:0 auto;padding:20px}.hero{display:flex;justify-content:space-between;align-items:center;gap:16px;flex-wrap:wrap;margin-bottom:18px}.title{font-size:30px;font-weight:700;color:var(--color-primary)}.sub{opacity:.8;margin-top:4px}.toprow{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:14px;margin-bottom:14px;align-items:stretch}.topcard{width:auto;min-width:0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}.card{background:var(--color-base-100);border:1px solid var(--color-base-300);border-radius:var(--radius-box);padding:16px;box-shadow:0 8px 24px rgba(0,0,0,.08);min-width:0}.label{font-size:13px;opacity:.72;margin-bottom:6px}.value{font-size:clamp(22px,2.4vw,28px);font-weight:700;line-height:1.15;min-width:0;overflow-wrap:anywhere;color:var(--color-primary)}.small{font-size:14px;opacity:.88;overflow-wrap:anywhere}.badge{display:inline-block;padding:6px 10px;border-radius:999px;font-size:12px;font-weight:700}.ok{background:var(--color-success);color:var(--color-success-content)}.bad{background:var(--color-error);color:var(--color-error-content)}table{width:100%;border-collapse:collapse;font-size:14px}.meta td{padding:9px 0;border-bottom:1px solid var(--color-base-300)}.muted{opacity:.72}.footer{margin-top:16px;font-size:12px;opacity:.7}</style></head><body><div class='wrap'>)rawliteral";
    html += "<div class='hero'><div><div class='title'>🌧️ Regenmengenmesser 🌧️</div><div class='sub'>Wemos D1 mini · S49E an A0 · BME280/BMP280 Chip-ID Auto-Erkennung · Auto-Refresh alle 15 s</div></div><div>";
    html += statusBadge(WiFi.status()==WL_CONNECTED,"WLAN online","WLAN offline") + " " + statusBadge(mqtt.connected(),"MQTT online","MQTT offline") + " " + statusBadge(sensorOk,sensorTypeText()+" ok","BME/BMP fehlt") + "</div></div>";

    html += "<div class='toprow'>";
    html += "<div class='card topcard'><div class='label'>Regen heute</div><div class='value'>" + String(g_tipsToday*MM_PER_TIP,2) + " <span class='small'>mm</span></div><div class='small'>" + String(g_tipsToday) + " Tips</div></div>";
    html += "<div class='card topcard'><div class='label'>Regen gesamt</div><div class='value'>" + String(g_tipsTotal*MM_PER_TIP,2) + " <span class='small'>mm</span></div><div class='small'>" + String(g_tipsTotal) + " Tips</div></div>";
    html += "<div class='card topcard'><div class='label'>Temperatur</div><div class='value'>" + fmtFloat(lastTempC,1) + " <span class='small'>°C</span></div><div class='small'>" + sensorTypeText() + " " + sensorAddressText() + "</div></div>";
    html += "<div class='card topcard'><div class='label'>Luftdruck</div><div class='value'>" + fmtFloat(lastPressureHpa,1) + " <span class='small'>hPa</span></div><div class='small'>auf Sensorhöhe</div></div>";
    if (!isBMP280) {
      html += "<div class='card topcard'><div class='label'>Luftfeuchte</div><div class='value'>" + fmtFloat(lastHumidityPct,1) + " <span class='small'>%</span></div><div class='small'>relative Feuchte</div></div>";
    }
    html += "</div>";

    html += "<div class='grid'>";
    html += "<div class='card'><div class='label'>Hall-Sensor S49E</div><table class='meta'>";
    html += "<tr><td class='muted'>Rohwert</td><td>" + String(lastHallRaw) + "</td></tr><tr><td class='muted'>Delta</td><td>" + String(lastHallDelta) + "</td></tr><tr><td class='muted'>Center</td><td>" + String(hallCenter) + "</td></tr><tr><td class='muted'>Trigger / Reset</td><td>" + String(triggerDelta) + " / " + String(resetDelta) + "</td></tr><tr><td class='muted'>Min / Max zuletzt</td><td>" + String(lastHallMin) + " / " + String(lastHallMax) + "</td></tr><tr><td class='muted'>Status</td><td>" + hallStateText() + "</td></tr></table></div>";

    html += "<div class='card'><div class='label'>Sensor / I2C</div><table class='meta'><tr><td class='muted'>Sensor erkannt</td><td>" + String(sensorOk ? "ja" : "nein") + "</td></tr><tr><td class='muted'>Typ</td><td>" + sensorTypeText() + "</td></tr><tr><td class='muted'>Adresse</td><td>" + sensorAddressText() + "</td></tr><tr><td class='muted'>I2C SDA / SCL</td><td>D2 / D1</td></tr><tr><td class='muted'>Erkennung</td><td>Chip-ID 0xD0</td></tr><tr><td class='muted'>Boot-Scan</td><td>" + String(ENABLE_I2C_SCAN_AT_BOOT ? "an" : "aus") + "</td></tr></table></div>";

    html += "<div class='card'><div class='label'>System</div><table class='meta'>";
    html += "<tr><td class='muted'>Gerät</td><td>" + String(DEVICE_NAME) + "</td></tr>";
    html += "<tr><td class='muted'>Firmware</td><td>regenmesser_v4_10_layout_fix</td></tr>";
    html += "<tr><td class='muted'>Uptime</td><td>" + formatUptime() + "</td></tr>";
    html += "<tr><td class='muted'>Zeit</td><td>" + isoTimestamp() + "</td></tr>";
    html += "<tr><td class='muted'>OTA Hostname</td><td>" + String(OTA_HOSTNAME) + "</td></tr>";
    html += "<tr><td class='muted'>WLAN SSID</td><td>" + String(WIFI_SSID) + "</td></tr>";
    html += "<tr><td class='muted'>WLAN Signal</td><td>" + String(WiFi.RSSI()) + " dBm</td></tr>";
    html += "<tr><td class='muted'>IP</td><td>" + WiFi.localIP().toString() + "</td></tr>";
    html += "<tr><td class='muted'>Gateway</td><td>" + WiFi.gatewayIP().toString() + "</td></tr>";
    html += "<tr><td class='muted'>MAC</td><td>" + WiFi.macAddress() + "</td></tr>";
    html += "<tr><td class='muted'>Topic</td><td>" + String(BASE_TOPIC) + "/#</td></tr></table></div>";

    html += "</div><div class='footer'>Lokale Statusseite des Sensors. MQTT-Ziel in ioBroker: <b>mqtt.0.regenmesser.*</b></div></div></body></html>";
    webServer.send(200, "text/html; charset=utf-8", html);
  });
  webServer.begin();
}

bool connectMqtt(){
  if(mqtt.connected()) return true; if(WiFi.status()!=WL_CONNECTED) return false; mqtt.setServer(MQTT_HOST, MQTT_PORT);
  bool ok = strlen(MQTT_USER)>0 ? mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASSWORD, topic("status").c_str(),1,true,"offline")
                                : mqtt.connect(DEVICE_ID, topic("status").c_str(),1,true,"offline");
  if(ok){
    publishString("status","online",true); publishString("device/id",DEVICE_ID,true); publishString("device/name",DEVICE_NAME,true);
    publishString("device/ip",WiFi.localIP().toString(),true); publishString("device/mac",WiFi.macAddress(),true);
    publishString("device/fw","regenmesser_v4_10_layout_fix",true); publishBool("device/sensor_ok",sensorOk,true);
    publishBool("device/time_synced",timeSynced,true); publishString("device/last_boot",isoTimestamp(),true); publishString("device/ota_host",OTA_HOSTNAME,true);
    publishString("device/sensor_type", sensorTypeText(), true);
    publishString("device/sensor_address", sensorAddressText(), true);
    publishLong("hall/raw",lastHallRaw,true); publishLong("hall/delta",lastHallDelta,true); publishLong("hall/center",hallCenter,true); publishLong("hall/trigger_delta",triggerDelta,true); publishLong("hall/reset_delta",resetDelta,true);
    publishString("hall/state", hallStateText(), true);
    lastHealthyMs=millis(); return true;
  } return false;
}

void publishEnvValues(){
  if(!sensorOk) return;
  if(isBME280){
    float t=bme.readTemperature(), p=bme.readPressure()/100.0f, h=bme.readHumidity();
    if(!isnan(t)) lastTempC=t;
    if(!isnan(p)) lastPressureHpa=p;
    if(!isnan(h)) lastHumidityPct=h;
  } else if(isBMP280){
    float t=bmp.readTemperature(), p=bmp.readPressure()/100.0f;
    if(!isnan(t)) lastTempC=t;
    if(!isnan(p)) lastPressureHpa=p;
    lastHumidityPct = NAN;
  }
  publishFloat("bme280/temperature_c",lastTempC,2,true);
  publishFloat("bme280/pressure_hpa",lastPressureHpa,2,true);
  if(isBME280) publishFloat("bme280/humidity_pct",lastHumidityPct,2,true);
}

void publishRainTotals(){ publishUInt("rain/tips_total",g_tipsTotal,true); publishFloat("rain/total_mm",g_tipsTotal*MM_PER_TIP,2,true); publishUInt("rain/today_tips",g_tipsToday,true); publishFloat("rain/today_mm",g_tipsToday*MM_PER_TIP,2,true); publishUInt("rain/window_1m_tips_pending",g_tips1m,true); publishUInt("rain/window_10m_tips_pending",g_tips10m,true); publishUInt("rain/window_60m_tips_pending",g_tips60m,true); }
void processRainWindows(){
  uint32_t now=millis();
  if((uint32_t)(now-last1mWindowMs)>=60000UL){ uint32_t c=g_tips1m; g_tips1m=0; publishUInt("rain/rate_1m_tips",c,true); publishFloat("rain/rate_1m_mm_min",c*MM_PER_TIP,2,true); last1mWindowMs=now; }
  if((uint32_t)(now-last10mWindowMs)>=600000UL){ uint32_t c=g_tips10m; g_tips10m=0; publishUInt("rain/rate_10m_tips",c,true); publishFloat("rain/rate_10m_mm_min",(c*MM_PER_TIP)/10.0f,2,true); last10mWindowMs=now; }
  if((uint32_t)(now-last60mWindowMs)>=3600000UL){ uint32_t c=g_tips60m; g_tips60m=0; publishUInt("rain/rate_60m_tips",c,true); publishFloat("rain/rate_60m_mm_h",c*MM_PER_TIP,2,true); last60mWindowMs=now; }
}
void publishHallTelemetry(){ publishLong("hall/raw",lastHallRaw,true); publishLong("hall/delta",lastHallDelta,true); publishLong("hall/min",lastHallMin,true); publishLong("hall/max",lastHallMax,true); publishString("hall/state", hallStateText(), true); }
void publishHeartbeat(){ publishLong("device/rssi",WiFi.RSSI(),true); publishUInt("device/uptime_s",(millis()-bootMillis)/1000UL,true); publishString("device/time",isoTimestamp(),true); publishBool("device/time_synced",timeSynced,true); publishString("status","online",true); publishString("device/sensor_type", sensorTypeText(), true); publishString("device/sensor_address", sensorAddressText(), true); if(WiFi.status()==WL_CONNECTED && mqtt.connected()) lastHealthyMs=millis(); }
void publishJsonSummary(){
  String t=isnan(lastTempC)?"null":String(lastTempC,2), h=(isBMP280||isnan(lastHumidityPct))?"null":String(lastHumidityPct,2), p=isnan(lastPressureHpa)?"null":String(lastPressureHpa,2);
  String json="{";
  json += "\"device\":\""+String(DEVICE_ID)+"\",\"name\":\""+String(DEVICE_NAME)+"\",\"status\":\"online\",\"ip\":\""+WiFi.localIP().toString()+"\",";
  json += "\"rssi\":"+String(WiFi.RSSI())+",\"time\":\""+isoTimestamp()+"\",\"time_synced\":"+String(timeSynced?"true":"false")+",";
  json += "\"tips_total\":"+String(g_tipsTotal)+",\"rain_total_mm\":"+String(g_tipsTotal*MM_PER_TIP,2)+",\"rain_today_mm\":"+String(g_tipsToday*MM_PER_TIP,2)+",";
  json += "\"hall_raw\":"+String(lastHallRaw)+",\"hall_delta\":"+String(lastHallDelta)+",\"hall_state\":\""+hallStateText()+"\",";
  json += "\"sensor_type\":\""+sensorTypeText()+"\",\"sensor_ok\":"+String(sensorOk?"true":"false")+",\"sensor_address\":\""+sensorAddressText()+"\",";
  json += "\"temperature_c\":"+t+",\"humidity_pct\":"+h+",\"pressure_hpa\":"+p+"}";
  mqtt.publish(topic("json").c_str(), json.c_str(), true);
}
void fullPublish(){ publishRainTotals(); publishEnvValues(); publishHallTelemetry(); publishHeartbeat(); publishJsonSummary(); }
void handleFailsafeRestart(){ if(ENABLE_FAILSAFE_RESTART && (uint32_t)(millis()-lastHealthyMs)>FAILSAFE_RESTART_AFTER_MS) ESP.restart(); }

void setup(){
  Serial.begin(115200); delay(50); bootMillis=millis(); lastHealthyMs=millis();
  Wire.begin(PIN_SDA,PIN_SCL);
  if (ENABLE_I2C_SCAN_AT_BOOT) scanI2C();
  sensorOk = initEnvSensor();
  connectWifi(); setupTime(); setupOTA(); setupWebInfo();
  lastHallRaw = readHallFiltered();
  hallCenter = lastHallRaw;
  hallState = classifyHallState(lastHallRaw - hallCenter);
  uint32_t now=millis(); last1mWindowMs=now; last10mWindowMs=now; last60mWindowMs=now; lastSensorPublishMs=now; lastHeartbeatMs=now; lastHallTelemetryMs=now;
}

void loop(){
  yield(); ArduinoOTA.handle(); webServer.handleClient(); handleHallRainGauge();
  uint32_t now=millis();
  if(WiFi.status()!=WL_CONNECTED){ if((uint32_t)(now-lastWifiRetryMs)>=WIFI_RETRY_MS){ lastWifiRetryMs=now; connectWifi(); } handleFailsafeRestart(); delay(5); return; }
  updateTimeState(); handleMidnightReset();
  if(!mqtt.connected()){ if((uint32_t)(now-lastMqttRetryMs)>=MQTT_RETRY_MS){ lastMqttRetryMs=now; if(connectMqtt()) fullPublish(); } handleFailsafeRestart(); delay(5); return; }
  mqtt.loop(); processRainWindows();
  if((uint32_t)(now-lastHallTelemetryMs)>=HALL_TELEMETRY_MS){ publishHallTelemetry(); lastHallTelemetryMs=now; }
  if((uint32_t)(now-lastHeartbeatMs)>=HEARTBEAT_MS){ publishHeartbeat(); lastHeartbeatMs=now; }
  if((uint32_t)(now-lastSensorPublishMs)>=SENSOR_PUBLISH_MS){ fullPublish(); lastSensorPublishMs=now; }
  handleFailsafeRestart(); delay(5);
}
