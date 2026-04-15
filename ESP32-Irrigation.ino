//ESP32/ESP32-s3 - Irrigation Controller  
//OLED or TFT NOW SELECTED IN SETUP

#ifndef ENABLE_DEBUG_ROUTES
  #define ENABLE_DEBUG_ROUTES 0   // set to 1 when you need them
#endif
#ifndef ENABLE_OTA
  #define ENABLE_OTA 0
#endif
#ifndef TFT_MISO
  #define TFT_MISO -1   // ST7789 typically doesn't use MISO
#endif
#ifndef ENABLE_TFT
  #define ENABLE_TFT 1
#endif
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LittleFS.h>
#include <PCF8574.h> // (Must use Library from Github)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Adafruit_ST7789.h> // (Adafruit ST7735 and ST7789 Library)
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
extern "C" {
  #include "esp_log.h"
  #include "esp_wifi.h"
}
#include <time.h>
#include <ESPmDNS.h> 
#include <PubSubClient.h>   // MQTT

// ---------- Hardware ----------
static const uint8_t MAX_ZONES = 16;
#if defined(CONFIG_IDF_TARGET_ESP32)
static const int I2C_SDA_DEFAULT = 21;
static const int I2C_SCL_DEFAULT = 22;
#else
static const int I2C_SDA_DEFAULT = 8;
static const int I2C_SCL_DEFAULT = 9;
#endif
static int i2cSdaPin = I2C_SDA_DEFAULT;
static int i2cSclPin = I2C_SCL_DEFAULT;
#ifndef STATUS_PIXEL_PIN
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define STATUS_PIXEL_PIN 48   // ESP32-S3 DevKitC-1 onboard WS2812
  #else
    #define STATUS_PIXEL_PIN -1   // disable by default on non-S3 targets
  #endif
#endif
static const uint8_t STATUS_PIXEL_COUNT = 1;

TwoWire I2Cbus = TwoWire(0);
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA_DEFAULT, I2C_SCL_DEFAULT);
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA_DEFAULT, I2C_SCL_DEFAULT);
Adafruit_NeoPixel statusPixel(STATUS_PIXEL_COUNT, STATUS_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool statusPixelReady = false;
uint32_t statusPixelLastColor = 0;
extern uint32_t bootMillis;

// ---------- ST7789 (1.9") TFT ----------
// Common ST7789 sizes are 170x320 and 240x320.
// Kept runtime-configurable so the Setup page can persist the panel geometry.
static constexpr int16_t TFT_W_DEFAULT = 240;
static constexpr int16_t TFT_H_DEFAULT = 320;

// Choose pins that do NOT clash with your I2C (4/15) or other IO.
// Defaults (editable from Setup -> SPI (TFT)):
#if defined(CONFIG_IDF_TARGET_ESP32)
static const int TFT_SCLK_DEFAULT = 18;
static const int TFT_MOSI_DEFAULT = 23;
static const int TFT_CS_DEFAULT   = 5;
static const int TFT_DC_DEFAULT   = 2;
static const int TFT_RST_DEFAULT  = 4;
#else
static const int TFT_SCLK_DEFAULT = 41;  // SCL
static const int TFT_MOSI_DEFAULT = 42;  // SDA
static const int TFT_CS_DEFAULT   = 1;   // CS (avoid GPIO1 tank ADC)
static const int TFT_DC_DEFAULT   = 2;   // DC (avoid GPIO2 LED)
static const int TFT_RST_DEFAULT  = 21;  // RST (-1 allowed)
#endif
static const int TFT_BL_DEFAULT   = -1;  // or -1 if tied to 3V3

static int tftSclkPin = TFT_SCLK_DEFAULT;
static int tftMosiPin = TFT_MOSI_DEFAULT;
static int tftCsPin   = TFT_CS_DEFAULT;
static int tftDcPin   = TFT_DC_DEFAULT;
static int tftRstPin  = TFT_RST_DEFAULT;
static int tftBlPin   = TFT_BL_DEFAULT;
static int16_t tftPanelWidth  = TFT_W_DEFAULT;
static int16_t tftPanelHeight = TFT_H_DEFAULT;
static uint8_t tftRotation = 3; // ST7789 rotation: 0..3

Adafruit_ST7789 tft(&SPI, -1, -1, -1);

// ---------- OLED ----------
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);
bool displayUseTft = (ENABLE_TFT != 0); // default mode; can be changed in Setup

// ===================== UI helpers + palette (MUST be before RainScreen/HomeScreen) =====================
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b){
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Palette (RGB565)
static const uint16_t C_BG     = ST77XX_BLACK;
static const uint16_t C_PANEL  = RGB(16, 22, 36);
static const uint16_t C_EDGE   = RGB(40, 55, 85);
static const uint16_t C_TEXT   = RGB(235, 242, 255);
static const uint16_t C_MUTED  = RGB(160, 175, 205);
static const uint16_t C_ACCENT = RGB(250, 210, 79);
static const uint16_t C_GOOD   = RGB(70, 200, 140);
static const uint16_t C_WARN   = RGB(255, 190, 70);
static const uint16_t C_BAD    = RGB(255, 110, 90);

static void drawCard(int x,int y,int w,int h,uint16_t fill,uint16_t edge){
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, edge);
  tft.drawPixel(x, y, fill); tft.drawPixel(x+w-1, y, fill);
  tft.drawPixel(x, y+h-1, fill); tft.drawPixel(x+w-1, y+h-1, fill);
}

static void drawTopBar(const char* title, const char* pill, uint16_t pillColor){
  const int W = tft.width();
  tft.fillRect(0, 0, W, 34, C_PANEL);
  tft.drawFastHLine(0, 34, W, C_EDGE);

  tft.setTextSize(2);
  tft.setTextColor(C_TEXT);
  tft.setCursor(8, 8);
  tft.print(title);

  int pillW = 8 + (int)strlen(pill) * 12; // rough for textSize(2)
  int x = W - pillW - 8;
  tft.fillRect(x, 7, pillW, 20, pillColor);
  tft.drawRect(x, 7, pillW, 20, C_EDGE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(x + 6, 9);
  tft.print(pill);
}

static void fmtMMSS(char* out, size_t n, unsigned long sec){
  unsigned long m = sec / 60;
  unsigned long s = sec % 60;
  snprintf(out, n, "%02lum%02lus", m, s);
}

WiFiManager wifiManager;
WebServer server(80);

// PCF mapping
const uint8_t ALL_P = 6;
const uint8_t PCH[ALL_P] = { P0, P1, P2, P3, P4, P5 };
uint8_t mainsChannel = P4; // reserved for mains relay
uint8_t tankChannel  = P5; // reserved for tank relay
uint8_t zonesCount   = 4;  // 1..MAX_ZONES

// GPIO fallback (configurable polarity)
bool useGpioFallback = false;
bool gpioActiveLow   = false;
bool relayActiveHigh = true;
bool zoneGpioActiveLow[MAX_ZONES] = {false};
bool mainsGpioActiveLow = false;
bool tankGpioActiveLow  = false;
bool tankEnabled     = true;

#if defined(CONFIG_IDF_TARGET_ESP32)
int zonePins[MAX_ZONES] = {
  15, 16, 17, 18, 4, 5,
  25, 26, -1, -1, -1, -1, -1, -1, -1, -1
};
int mainsPin = 32;
int tankPin  = 33;
int tankLevelPin = 34; // ADC1 input on classic ESP32 (works while Wi-Fi is active)
#else
int zonePins[MAX_ZONES] = {
  15, 16, 17, 18, 6, 7,
  10, 11, -1, -1, -1, -1, -1, -1, -1, -1
};
int mainsPin = 39;
int tankPin  = 40;
int tankLevelPin = 19; // ADC input (ESP32-S3: GPIO1..20 are ADC)
#endif

const int LED_PIN  = -1;

// Physical rain sensor
bool rainSensorEnabled = false;
bool rainSensorInvert  = false;
int  rainSensorPin     = 16;

// Physical manual control buttons (disabled by default)
int manualSelectPin = -1;   // cycles the target zone (INPUT_PULLUP, -1 = disabled)
int manualStartPin  = -1;   // toggles start/stop for the selected zone (INPUT_PULLUP, -1 = disabled)
uint8_t manualSelectedZone = 0;
uint32_t manualScreenUntilMs = 0;
const unsigned long MANUAL_BTN_DEBOUNCE_MS = 60;
const unsigned long MANUAL_BTN_REPEAT_GUARD_MS = 180;

struct ManualButtonState {
  int stableState = HIGH;
  int lastRawState = HIGH;
  uint32_t lastRawChangeMs = 0;
  uint32_t lastPressMs = 0;
};

static ManualButtonState g_manualSelectBtn;
static ManualButtonState g_manualStartBtn;

// ---------- Config / State ----------
float  meteoLat = NAN;
float  meteoLon = NAN;
String meteoLocation; // Open-Meteo display label (optional)
String meteoModel = "gfs"; // Open-Meteo model endpoint (e.g., gfs, icon, ecmwf)
String cachedWeatherData;
String lastWeatherError;
String lastForecastError;
int    lastWeatherHttpCode  = 0;
int    lastForecastHttpCode = 0;

// Weather cache / metrics
unsigned long lastWeatherUpdate = 0;
const unsigned long weatherUpdateInterval = 15UL * 60UL * 1000UL; // 15m

// Forecast cache / metrics
String cachedForecastData;
unsigned long lastForecastUpdate = 0;
const unsigned long forecastUpdateInterval = 60UL * 60UL * 1000UL; // 60m
float rainNext12h_mm = NAN;
float rainNext24h_mm = NAN;
int   popNext12h_pct = -1;
int   nextRainIn_h   = -1;
float maxGust24h_ms  = NAN;
float todayMin_C     = NAN, todayMax_C = NAN;
time_t todaySunrise  = 0,   todaySunset = 0;

// Delay controls
bool  rainDelayEnabled = true;
bool  windDelayEnabled = false;
bool  justUseTank = false;
bool  justUseMains = false;

// New saved features
bool     systemPaused = false;
uint32_t pauseUntilEpoch = 0;
bool     rainDelayFromForecastEnabled = true;  // gate for forecast-based rain

// NEW Master/Cooldown/Threshold
bool     systemMasterEnabled = true;     // Master On/Off
uint32_t rainCooldownUntilEpoch = 0;     // when > now => block starts
int      rainCooldownMin = 60;           // minutes to wait after rain clears
int      rainThreshold24h_mm = 5;        // forecast/actual 24h total triggers delay

// NEW Run mode: sequential (false) or concurrent (true)
bool runZonesConcurrent = false;

// Scheduling
bool enableStartTime2[MAX_ZONES] = {false};
bool days[MAX_ZONES][7] = {{false}};
bool zoneActive[MAX_ZONES] = {false};
bool zoneStartedManual[MAX_ZONES] = {false};
bool pendingStart[MAX_ZONES] = {false};
uint8_t lastStartSlot[MAX_ZONES] = {1}; // 1=primary, 2=secondary

bool     windActive = false;
bool     rainActive             = false;
bool     rainByWeatherActive    = false;
bool     rainBySensorActive     = false;
uint8_t  rainCooldownHours      = 24;      // or loaded from config

// Photoresistor (auto backlight)
bool photoAutoEnabled = false;
bool photoInvert = false;
int  photoPin = -1;
int  photoThreshold = 1500; // ADC raw (0-4095)


// legacy (kept for file compat; not used in logic)
float tzOffsetHours = 9.5f;

float windSpeedThreshold = 5.0f;
float lastRainAmount = 0.0f;
uint8_t tankLowThresholdPct = 10;

int startHour [MAX_ZONES] = {0};
int startMin  [MAX_ZONES] = {0};
int startHour2[MAX_ZONES] = {0};
int startMin2 [MAX_ZONES] = {0};
int durationMin[MAX_ZONES] = {0};
int durationSec[MAX_ZONES] = {0};
int duration2Min[MAX_ZONES] = {0};
int duration2Sec[MAX_ZONES] = {0};
int lastCheckedMinute[MAX_ZONES] = {
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1
};

int tankEmptyRaw = 100;
int tankFullRaw  = 900;
String zoneNames[MAX_ZONES] = {
  "Zone 1","Zone 2","Zone 3","Zone 4","Zone 5","Zone 6",
  "Zone 7","Zone 8","Zone 9","Zone 10","Zone 11","Zone 12",
  "Zone 13","Zone 14","Zone 15","Zone 16"
};

unsigned long zoneStartMs[MAX_ZONES] = {0};
unsigned long zoneRunTotalSec[MAX_ZONES] = {0}; // actual duration for current run
unsigned long lastScreenRefresh = 0;

const uint8_t expanderAddrs[] = { 0x22, 0x24 };
const uint8_t I2C_HEALTH_DEBOUNCE = 10;
uint8_t i2cFailCount = 0;

// Timing
static const uint32_t LOOP_SLEEP_MS    = 20;
static const uint32_t I2C_CHECK_MS     = 1000;
static const uint32_t TIME_QUERY_MS    = 1000;
static const uint32_t SCHEDULE_TICK_MS = 1000;
static uint32_t lastI2cCheck     = 0;
static uint32_t lastTimeQuery    = 0;
static uint32_t lastScheduleTick = 0;
static bool midnightDone = false;
static tm cachedTm = {};

// Uptime
uint32_t bootMillis = 0;

// ---------- NEW: Actual rainfall history (rolling 24h) + globals for 1h ----------
static float rainHist[24] = {0};   // last 24 hourly buckets (mm/hour)
static int   rainIdx = 0;          // points to most recent bucket
static time_t lastRainHistHour = 0;
static float last24hActualRain(); // forward
float rain1hNow = 0.0f;  // mm from current precipitation (Open-Meteo)
float rain3hNow = 0.0f;  // kept for compatibility

// Decoded current-weather snapshot (avoids repeated JSON parsing in hot paths)
float curTempC = NAN;
float curFeelsC = NAN;
float curPressureHpa = NAN;
float curWindMs = NAN;
float curGustMs = NAN;
float curWindDirDeg = NAN;
int   curHumidityPct = -1;
int   curWeatherCode = -1;
int   curUtcOffsetSec = 0;
bool  curWeatherValid = false;

// ---------- NEW: guard to avoid fetching while serving HTTP ----------
volatile bool g_inHttp = false;
struct HttpScope {
  HttpScope()  { g_inHttp = true; }
  ~HttpScope() { g_inHttp = false; }
};

#if defined(BOARD_HAS_PSRAM)
static constexpr size_t PSRAM_LARGE_ALLOC_THRESHOLD = 2048;

static void configurePsramForLargeBuffers() {
  if (!psramFound()) {
    Serial.println("[BOOT] PSRAM board selected, but no PSRAM was detected.");
    return;
  }
  heap_caps_malloc_extmem_enable(PSRAM_LARGE_ALLOC_THRESHOLD);
  Serial.printf("[BOOT] PSRAM ready: total=%u free=%u threshold=%u\n",
                (unsigned)ESP.getPsramSize(),
                (unsigned)ESP.getFreePsram(),
                (unsigned)PSRAM_LARGE_ALLOC_THRESHOLD);
}
#else
static void configurePsramForLargeBuffers() {}
#endif

// ---------- Prototypes ----------
void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
void tickWeather();                 // NEW
void HomeScreen();
void RainScreen();
void updateLCDForZone(int zone);
bool shouldStartZone(int zone);
bool hasDurationCompleted(int zone);
void turnOnZone(int zone);
void turnOffZone(int zone);
void turnOnValveManual(int z);
void turnOffValveManual(int z);
void handleRoot();
void handleSubmit();
void handleSetupPage();
void handleConfigure();
void handleLogPage();
void handleClearEvents();
void handleTankCalibration();
String fetchWeather();
String fetchWeatherHourlyForCurrent(const String& model, float lat, float lon, bool useForecastEndpoint);
bool buildCurrentFromHourlyPayload(const String& hourlyPayload, String& outPayload);
String fetchForecast(float lat, float lon);
bool checkWindRain();
void checkI2CHealth();
void initGpioFallback();
void initGpioPinsForZones();
bool initExpanders();
void toggleBacklight();
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed);
void printCurrentTime();
int    tankPercent();
bool   isTankLow();
String sourceModeText();
bool refreshCurrentWeatherSnapshotFromCache();
void initManualButtons();
void tickManualButtons();
void showManualSelection();
void drawManualSelection();
bool isValidAdcPin(int pin);
bool isValidGpioPin(int pin);
bool isValidPhotoPin(int pin);
void updateStatusPixel();
void statusPixelSet(uint8_t r,uint8_t g,uint8_t b);
uint8_t statusPixelPulseLevel(uint16_t periodMs, uint8_t low, uint8_t high);
bool statusPixelWindowOn(uint16_t periodMs, uint16_t startMs, uint16_t widthMs);
bool initTempSensor();
bool readChipTempC(float& outC);
bool physicalRainNowRaw();


// ===================== Timezone config =====================
enum TZMode : uint8_t { TZ_POSIX = 0, TZ_IANA = 1, TZ_FIXED = 2 };
TZMode tzMode = TZ_POSIX;
String tzPosix = "ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3"; // default
String tzIANA  = "Australia/Adelaide";
int16_t tzFixedOffsetMin = 570;

static void applyTimezoneAndSNTP() {
  const char* ntp1 = "pool.ntp.org";
  const char* ntp2 = "time.google.com";
  const char* ntp3 = "time.cloudflare.com";

  switch (tzMode) {
    case TZ_IANA: configTzTime(tzIANA.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_POSIX: configTzTime(tzPosix.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_FIXED: {
      long offSec = (long)tzFixedOffsetMin * 60L;
      configTime(offSec, 0, ntp1, ntp2, ntp3);
      int m = tzFixedOffsetMin; int sign = (m >= 0) ? -1 : 1; m = abs(m);
      int hh = m/60, mm = m%60; char buf[32];
      snprintf(buf, sizeof(buf), "GMT%+d:%02d", sign*hh, sign*mm);
      setenv("TZ", buf, 1); tzset(); break;
    }
  }
  time_t now = time(nullptr);
  for (int i=0; i<50 && now < 1000000000; ++i) { delay(200); now = time(nullptr); }
}

// ---------- MQTT ----------
bool   mqttEnabled = false;
String mqttBroker  = "";
uint16_t mqttPort  = 1883;
String mqttUser    = "";
String mqttPass    = "";
String mqttBase    = "espirrigation";

WiFiClient   _mqttNetCli;
PubSubClient _mqtt(_mqttNetCli);
uint32_t     _lastMqttPub = 0;
uint32_t     _lastMqttAttempt = 0;

void mqttSetup(){
  if (!mqttEnabled || mqttBroker.length()==0) return;
  _mqtt.setServer(mqttBroker.c_str(), mqttPort);
  _mqtt.setBufferSize(2048);
  _mqtt.setKeepAlive(30);
  _mqtt.setSocketTimeout(5);
  _mqtt.setCallback([](char* topic, byte* payload, unsigned int len){
    String t(topic), msg; msg.reserve(len);
    for (unsigned i=0;i<len;i++) msg += (char)payload[i];

    if (t.endsWith("/cmd/master")) {
      systemMasterEnabled = (msg=="on"||msg=="ON"||msg=="1");
      saveConfig();
    } else if (t.endsWith("/cmd/pause")) {
      uint32_t sec = msg.toInt();
      systemPaused = true; pauseUntilEpoch = sec ? (time(nullptr)+sec) : 0; saveConfig();
    } else if (t.endsWith("/cmd/resume")) {
      systemPaused=false; pauseUntilEpoch=0; saveConfig();
    } else if (t.endsWith("/cmd/stop_all")) {
      for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    } else if (t.indexOf("/cmd/zone/")!=-1) {
      int p=t.lastIndexOf('/'); int z=(p>=0)? t.substring(p+1).toInt():-1;
      if (z>=0 && z<(int)zonesCount){
        if (msg=="on"||msg=="ON"||msg=="1") turnOnValveManual(z);
        else                                turnOffValveManual(z);
      }
    }
  });
}
void mqttEnsureConnected(){
  if (!mqttEnabled || mqttBroker.length()==0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (_mqtt.connected()) return;
  const uint32_t now = millis();
  if (now - _lastMqttAttempt < 5000) return;
  _lastMqttAttempt = now;
  String cid = "espirrigation-" + WiFi.macAddress();
  bool ok = false;
  if (mqttUser.length()) {
    ok = _mqtt.connect(cid.c_str(), mqttUser.c_str(), mqttPass.c_str());
  } else {
    ok = _mqtt.connect(cid.c_str());
  }
  if (ok) {
    _mqtt.subscribe( (mqttBase + "/cmd/#").c_str() );
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", _mqtt.state());
  }
}

void mqttPublishStatus(){
  if (!mqttEnabled || !_mqtt.connected()) return;
  if (millis() - _lastMqttPub < 3000) return;
  _lastMqttPub = millis();

  JsonDocument d;
  d["masterOn"] = systemMasterEnabled;
  d["paused"]   = (systemPaused && (pauseUntilEpoch==0 || time(nullptr)<(time_t)pauseUntilEpoch));
  d["cooldownRemaining"] = (rainCooldownUntilEpoch>time(nullptr)? (rainCooldownUntilEpoch - time(nullptr)) : 0);
  d["rainActive"] = rainActive;
  d["windActive"] = windActive;
  d["tankPct"]    = tankPercent();
  d["sourceMode"] = sourceModeText();
  d["rain24hActual"] = last24hActualRain();  // NEW
  d["runConcurrent"] = runZonesConcurrent;   // NEW
  float chipTemp = NAN;
  if (readChipTempC(chipTemp)) d["chipTempC"] = chipTemp;
  else                         d["chipTempC"] = nullptr;
  JsonArray arr = d["zones"].to<JsonArray>();
  for (int i=0;i<zonesCount;i++){
    JsonObject z = arr.add<JsonObject>();
    z["name"] = zoneNames[i];
    z["active"] = zoneActive[i];
  }
  String out;
  out.reserve(900);
  serializeJson(d, out);
  _mqtt.publish( (mqttBase + "/status").c_str(), out.c_str(), true);
}

static inline int i_min(int a, int b) { return (a < b) ? a : b; }

static inline bool isPausedNow() {
  time_t now = time(nullptr);
  return systemPaused && (pauseUntilEpoch == 0 || now < (time_t)pauseUntilEpoch);
}

static inline bool isCooldownActiveNow() {
  time_t now = time(nullptr);
  return (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch);
}

static inline bool isBlockedNow(){
  if (!systemMasterEnabled) return true;
  if (isPausedNow()) return true;
  if (isCooldownActiveNow()) return true;
  return false;
}

static bool hasActiveManualZone() {
  for (int z = 0; z < (int)zonesCount; ++z) {
    if (zoneActive[z] && zoneStartedManual[z]) return true;
  }
  return false;
}

static void stopAutoZonesForBlock() {
  for (int z = 0; z < (int)zonesCount; ++z) {
    if (zoneActive[z] && !zoneStartedManual[z]) turnOffZone(z);
  }
}

void updateStatusPixel() {
  if (!statusPixelReady) return;

  // Default: ready/idle = soft green breathe.
  uint8_t r = 0, g = statusPixelPulseLevel(1800, 4, 18), b = 0;

  // Is any zone currently active?
  bool anyZoneOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyZoneOn = true; break; }
  }

  const uint32_t sinceBootMs = millis() - bootMillis;
  if (sinceBootMs < 3000U) {
    // Boot: cyan breathe.
    uint8_t level = statusPixelPulseLevel(1200, 6, 24);
    r = 0; g = level; b = level;
  } else if (WiFi.status() != WL_CONNECTED) {
    // Wi-Fi disconnected: violet double flash.
    bool on = statusPixelWindowOn(1100, 0, 120) || statusPixelWindowOn(1100, 180, 120);
    r = on ? 22 : 1;
    g = 0;
    b = on ? 14 : 0;
  } else if (anyZoneOn) {
    // Watering in progress: solid blue.
    r = 0; g = 0; b = 28;
  } else if (!systemMasterEnabled || isPausedNow()) {
    // Master off / pause: red slow pulse.
    r = statusPixelPulseLevel(1400, 2, 26);
    g = 0;
    b = 0;
  } else if (rainActive || windActive || isBlockedNow()) {
    // Weather / cooldown blocking: amber heartbeat.
    bool on = statusPixelWindowOn(1300, 0, 110) || statusPixelWindowOn(1300, 170, 110);
    r = on ? 28 : 2;
    g = on ? 14 : 1;
    b = 0;
  } else if (useGpioFallback) {
    // Fallback mode: teal pulse to distinguish degraded I/O from normal ready.
    uint8_t level = statusPixelPulseLevel(1500, 4, 22);
    r = 0;
    g = level;
    b = level / 3;
  }

  statusPixelSet(r,g,b);
}

// --- Cancel helper: cancel a (would-be) start instead of queueing ---
static inline void cancelStart(int z, const char* reason, bool dueToRain) {
  if (z >= 0 && z < (int)MAX_ZONES) pendingStart[z] = false; // ensure not queued
  logEvent(z, "CANCELLED", reason, dueToRain);               // record why it was cancelled
}

static bool i2cPing(uint8_t addr) {
  I2Cbus.beginTransmission(addr);
  return (I2Cbus.endTransmission() == 0);
}

static String cleanName(String s) {
  s.trim(); s.replace("\r",""); s.replace("\n","");
  if (s.length() > 32) s = s.substring(0,32);
  return s;
}

static String cleanMeteoModel(String s) {
  s.trim();
  s.toLowerCase();
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-') {
      out += c;
    }
  }
  if (!out.length()) out = "gfs";
  return out;
}

static bool isKnownMeteoModel(const String& s) {
  return (s == "gfs" || s == "icon" || s == "ecmwf" || s == "meteofrance" ||
          s == "jma" || s == "cma" || s == "gem" || s == "icon_seamless" ||
          s == "icon_global" || s == "icon_eu" || s == "bom" ||
          s == "bom_access_global" || s == "ukmo_seamless");
}

static bool isMeteoErrorPayload(const String& payload) {
  if (payload.indexOf("\"error\"") == -1) return false;
  JsonDocument js;
  if (deserializeJson(js, payload) != DeserializationError::Ok) return false;
  return (js["error"] | false) == true;
}

static String meteoErrorReason(const String& payload) {
  JsonDocument js;
  if (deserializeJson(js, payload) != DeserializationError::Ok) return "";
  if ((js["error"] | false) != true) return "";
  return js["reason"] | "";
}

static String meteoBaseUrl(const String& model, bool useForecastEndpoint) {
  if (useForecastEndpoint) return "https://api.open-meteo.com/v1/forecast";
  return "https://api.open-meteo.com/v1/" + model;
}

static String httpGetMeteo(const String& url, int& code, uint16_t timeoutMs) {
  HTTPClient http;
  WiFiClientSecure secure;
  secure.setInsecure();
  http.setTimeout(timeoutMs);
  http.begin(secure, url);
  code = http.GET();
  String payload = (code > 0) ? http.getString() : "";
  http.end();
  return payload;
}

static inline bool isValidLatLon(float lat, float lon) {
  return isfinite(lat) && isfinite(lon) && lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
}

static bool parseLatLon(const String& s, float& lat, float& lon) {
  String t = s; t.trim();
  if (!t.length()) return false;
  int sep = t.indexOf(',');
  if (sep < 0) sep = t.indexOf(' ');
  if (sep < 0) return false;
  String a = t.substring(0, sep);
  String b = t.substring(sep + 1);
  a.trim(); b.trim();
  if (!a.length() || !b.length()) return false;
  float la = a.toFloat();
  float lo = b.toFloat();
  if (!isValidLatLon(la, lo)) return false;
  lat = la; lon = lo;
  return true;
}

static String meteoLocationLabel() {
  if (meteoLocation.length()) return meteoLocation;
  if (isValidLatLon(meteoLat, meteoLon)) {
    return String(meteoLat, 4) + "," + String(meteoLon, 4);
  }
  return String("-");
}

static const char* meteoCodeToMain(int code) {
  if (code == 0) return "Clear";
  if (code >= 1 && code <= 3) return "Cloudy";
  if (code == 45 || code == 48) return "Fog";
  if (code >= 51 && code <= 57) return "Drizzle";
  if (code >= 61 && code <= 67) return "Rain";
  if (code >= 71 && code <= 77) return "Snow";
  if (code >= 80 && code <= 82) return "Showers";
  if (code >= 85 && code <= 86) return "Snow";
  if (code >= 95) return "Thunder";
  return "Unknown";
}

static const char* meteoCodeToDesc(int code) {
  switch (code) {
    case 0:  return "Clear sky";
    case 1:  return "Mainly clear";
    case 2:  return "Partly cloudy";
    case 3:  return "Overcast";
    case 45: return "Fog";
    case 48: return "Rime fog";
    case 51: return "Light drizzle";
    case 53: return "Moderate drizzle";
    case 55: return "Dense drizzle";
    case 56: return "Freezing drizzle";
    case 57: return "Dense freezing drizzle";
    case 61: return "Slight rain";
    case 63: return "Moderate rain";
    case 65: return "Heavy rain";
    case 66: return "Freezing rain";
    case 67: return "Heavy freezing rain";
    case 71: return "Slight snow";
    case 73: return "Moderate snow";
    case 75: return "Heavy snow";
    case 77: return "Snow grains";
    case 80: return "Rain showers";
    case 81: return "Heavy showers";
    case 82: return "Violent showers";
    case 85: return "Snow showers";
    case 86: return "Heavy snow showers";
    case 95: return "Thunderstorm";
    case 96: return "Thunderstorm hail";
    case 99: return "Thunderstorm heavy hail";
    default: return "Unknown";
  }
}

static inline bool meteoCodeIsWet(int code) {
  return (code >= 51 && code <= 67) || (code >= 71 && code <= 77) ||
         (code >= 80 && code <= 86) || (code >= 95);
}

static float normalizeDegrees360(float deg) {
  if (!isfinite(deg)) return NAN;
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static const char* meteoWindDirectionToCompass(float deg) {
  static const char* DIRS[16] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };
  float norm = normalizeDegrees360(deg);
  if (!isfinite(norm)) return "";
  int idx = (int)floor((norm + 11.25f) / 22.5f) & 15;
  return DIRS[idx];
}

static String formatWindDirection(float deg) {
  float norm = normalizeDegrees360(deg);
  if (!isfinite(norm)) return String("--");
  char buf[24];
  snprintf(buf, sizeof(buf), "%s (%d deg)", meteoWindDirectionToCompass(norm), (int)lroundf(norm));
  return String(buf);
}

static time_t parseLocalIsoTime(const char* s) {
  if (!s) return 0;
  if (strlen(s) < 16) return 0;
  struct tm tmv = {};
  tmv.tm_year = atoi(s) - 1900;
  tmv.tm_mon  = atoi(s + 5) - 1;
  tmv.tm_mday = atoi(s + 8);
  tmv.tm_hour = atoi(s + 11);
  tmv.tm_min  = atoi(s + 14);
  tmv.tm_sec  = 0;
  tmv.tm_isdst = -1;
  time_t t = mktime(&tmv);
  return (t < 0) ? 0 : t;
}

int tankPercent() {
  if (!isValidAdcPin(tankLevelPin)) {
    static bool warned = false;
    if (!warned) {
      #if defined(CONFIG_IDF_TARGET_ESP32)
      Serial.printf("[TANK] Invalid ADC pin %d. Set tankLevelPin to GPIO32-39 for ESP32.\n", tankLevelPin);
      #else
      Serial.printf("[TANK] Invalid ADC pin %d. Set tankLevelPin to GPIO1-20 for ESP32-S3.\n", tankLevelPin);
      #endif
      warned = true;
    }
    return 0;
  }
  const int N=8; uint32_t acc=0;
  for (int i=0;i<N;i++){ acc += analogRead(tankLevelPin); delayMicroseconds(200); }
  int raw=acc/N;
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isTankLow() {
  if (!tankEnabled) return true; // tank disabled => treat as low to force mains
  return tankPercent() <= tankLowThresholdPct;
}

String sourceModeText() {
  if (!tankEnabled) return "Tank Disabled";
  if (justUseTank)  return "Force: Tank";
  if (justUseMains) return "Force: Mains";
  return isTankLow() ? "Auto: Mains" : "Auto: Tank";
}

static inline void chooseWaterSource(const char*& src, bool& mainsOn, bool& tankOn) {
  mainsOn = false; tankOn = false;
  if (!tankEnabled) { src = "Tank Disabled"; mainsOn = true; return; }
  if (justUseTank)  { src = "Tank";  tankOn  = true; return; }
  if (justUseMains) { src = "Mains"; mainsOn = true; return; }
  if (isTankLow())  { src = "Mains"; mainsOn = true; return; }
  src = "Tank"; tankOn = true;
}

bool physicalRainNowRaw() {
  if (!rainSensorEnabled) return false;
  if (rainSensorPin < 0 || rainSensorPin > 39) {
    static bool warned = false;
    if (!warned) {
      Serial.printf("[RAIN] Invalid rainSensorPin=%d; ignoring sensor.\n", rainSensorPin);
      warned = true;
    }
    return false;
  }
  pinMode(rainSensorPin, INPUT_PULLUP);
  int v = digitalRead(rainSensorPin); // LOW=dry, HIGH=wet (NC default)
  bool wet = (v == HIGH);
  if (rainSensorInvert) wet = !wet;
  return wet;
}

String rainDelayCauseText() {
  if (!rainDelayEnabled) return "Disabled";

  time_t now = time(nullptr);

  // Not currently raining, check for active cooldown with countdown
  if (!rainActive) {
    if (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch) {
      uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - now); // seconds left
      // Round up to minutes so "59s" still shows as "1m"
      uint32_t mins = (rem + 59U) / 60U;

      char buf[40];

      if (mins >= 60U) {
        uint32_t h = mins / 60U;
        uint32_t m = mins % 60U;
        if (m > 0U) {
          // Example: "Cooldown 1h 05m"
          snprintf(buf, sizeof(buf), "Cooldown %luh %lum",
                   (unsigned long)h, (unsigned long)m);
        } else {
          // Example: "Cooldown 2h"
          snprintf(buf, sizeof(buf), "Cooldown %luh",
                   (unsigned long)h);
        }
      } else {
        // Under 1 hour: "Cooldown 23m"
        snprintf(buf, sizeof(buf), "Cooldown %lum",
                 (unsigned long)mins);
      }

      return String(buf);
    }

    if (!systemMasterEnabled) return "Master Off";
    if (isPausedNow())        return "Paused";
    if (windActive)           return "Windy";
    return "None";
  }

  // Currently in a rain-delay state
  if (rainByWeatherActive && rainBySensorActive) return "Both";
  if (rainByWeatherActive) return "Raining";
  if (rainBySensorActive)  return "Sensor Wet";
  return "Active";
}

static const char* kHost = "espirrigation";

static void mdnsStart() {
  MDNS.end(); // in case it was running
  if (MDNS.begin(kHost)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("arduino", "tcp", 3232); // keep OTA discoverable after restart
    Serial.println("[mDNS] started: http://espirrigation.local/");
  } else {
    Serial.println("[mDNS] begin() failed");
  }
}

// ---------- GPIO fallback helpers ----------
inline bool isValidOutputPin(int pin);

inline int gpioLevelForPolarity(bool on, bool activeLow) {
  if (activeLow) {
    // Active-LOW: LOW = ON, HIGH = OFF
    return on ? LOW : HIGH;
  } else {
    // Active-HIGH: HIGH = ON, LOW = OFF
    return on ? HIGH : LOW;
  }
}

inline int gpioLevel(bool on) {
  return gpioLevelForPolarity(on, gpioActiveLow);
}

inline void gpioInitOutput(int pin, bool activeLow) {
  if (!isValidOutputPin(pin)) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, gpioLevelForPolarity(false, activeLow));  // ensure OFF
}

inline void gpioZoneWrite(int z, bool on) {
  if (z < 0 || z >= (int)MAX_ZONES) return;
  int pin = zonePins[z];
  if (!isValidOutputPin(pin)) return;
  digitalWrite(pin, gpioLevelForPolarity(on, zoneGpioActiveLow[z]));
}

inline void gpioSourceWrite(bool mainsOn, bool tankOn) {
  bool mainsOk = isValidOutputPin(mainsPin);
  bool tankOk  = isValidOutputPin(tankPin);
  if (mainsOk) digitalWrite(mainsPin, gpioLevelForPolarity(mainsOn, mainsGpioActiveLow));
  if (tankOk)  digitalWrite(tankPin,  gpioLevelForPolarity(tankOn, tankGpioActiveLow));
}

inline void setWaterSourceRelays(bool mainsOn, bool tankOn) {
  // Prefer PCF outputs for the classic wiring; otherwise drive the configured GPIO pins.
  if (!useGpioFallback) {
    // PCF8574: active-LOW
    pcfOut.digitalWrite(mainsChannel, mainsOn ? LOW : HIGH);
    pcfOut.digitalWrite(tankChannel,  tankOn  ? LOW : HIGH);
  } else {
    gpioSourceWrite(mainsOn, tankOn);
  }
}

inline bool useExpanderForZone(int z) {
  if (useGpioFallback) return false;
  // Reserve PCF channels P4/P5 for mains/tank; only zones 0-3 use the expander
  return (z >= 0 && z < 4);
}

// Duration helper: slot=1 (primary) or 2 (secondary) with fallback to primary
static inline unsigned long durationForSlot(int z, int slot) {
  if (z < 0 || z >= (int)MAX_ZONES) return 0;
  if (slot == 2 && enableStartTime2[z]) {
    unsigned long m = (duration2Min[z] >= 0) ? (unsigned long)duration2Min[z] : 0;
    unsigned long s = (duration2Sec[z] >= 0) ? (unsigned long)duration2Sec[z] : 0;
    unsigned long tot = m * 60UL + s;
    if (tot > 0) return tot;
  }
  unsigned long m = (durationMin[z] >= 0) ? (unsigned long)durationMin[z] : 0;
  unsigned long s = (durationSec[z] >= 0) ? (unsigned long)durationSec[z] : 0;
  return m * 60UL + s;
}

inline bool isValidAdcPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32)
  // Classic ESP32: use ADC1 only so readings still work while Wi-Fi is active.
  return (pin >= 32 && pin <= 39);
  #else
  // ESP32-S3 ADC pins: GPIO1..20 (ADC1: 1-10, ADC2: 11-20)
  return (pin >= 1 && pin <= 20);
  #endif
}

inline bool isValidPhotoPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
  return (pin >= 1 && pin <= 40) && isValidGpioPin(pin);
  #else
  return isValidAdcPin(pin);
  #endif
}

inline bool isValidGpioPin(int pin) {
  if (pin < 0) return false;
  if (!GPIO_IS_VALID_GPIO((gpio_num_t)pin)) return false;
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (pin >= 6 && pin <= 11) return false; // SPI flash/PSRAM pins on classic ESP32
  #endif
  return true;
}

inline bool isValidOutputPin(int pin) {
  if (pin < 0) return false;
  if (!GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)pin)) return false;
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (pin >= 6 && pin <= 11) return false; // SPI flash/PSRAM pins on classic ESP32
  #endif
  return true;
}

inline bool isUnsafeTftPin(int pin) {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
  // ESP32-S3:
  // - 0/3/45/46 are strapping pins
  // - 26..32 are typically used for flash/PSRAM
  // - 33..37 are also reserved on octal PSRAM modules
  // GPIO19/20 are USB by default, but still usable as GPIO if USB is not needed.
  return (pin == 0 || pin == 3 || pin == 45 || pin == 46 ||
          (pin >= 26 && pin <= 32) || (pin >= 33 && pin <= 37));
  #else
  return (pin == 0 || pin == 19 || pin == 20 || pin == 45 || pin == 46 ||
          (pin >= 9 && pin <= 14) || (pin >= 35 && pin <= 38));
  #endif
}

inline bool isValidTftSignalPin(int pin) {
  return isValidOutputPin(pin) && !isUnsafeTftPin(pin);
}

inline bool isValidOptionalTftPin(int pin) {
  return (pin == -1) || isValidTftSignalPin(pin);
}

inline bool isValidTftDimension(int v) {
  return (v >= 120 && v <= 400);
}

static bool tryParseStrictInt(const String& raw, int &out) {
  String s = raw;
  s.trim();
  if (!s.length()) return false;
  char* end = nullptr;
  long v = strtol(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') return false;
  out = (int)v;
  return true;
}

static bool parseRequiredIntArg(const String& raw, int &out, String &err, const __FlashStringHelper* label) {
  String s = raw;
  s.trim();
  if (!s.length()) {
    err += String(label) + " is blank\n";
    return false;
  }
  if (!tryParseStrictInt(s, out)) {
    err += String(label) + " is not a valid integer: " + s + "\n";
    return false;
  }
  return true;
}

static void warnPinConflict(const char* aName, int aPin, const char* bName, int bPin) {
  if (aPin < 0 || bPin < 0) return;
  if (aPin != bPin) return;
  Serial.printf("[PIN] Conflict: %s and %s both on GPIO%d\n", aName, bName, aPin);
}

static void sanitizePinConfig() {
  auto clampGpio = [](int &p) {
    if (p != -1 && !isValidGpioPin(p)) p = -1;
  };
  auto clampOutputGpio = [](int &p) {
    if (p != -1 && !isValidOutputPin(p)) p = -1;
  };

  // Zone + source GPIOs
  for (uint8_t i = 0; i < MAX_ZONES; i++) {
    clampOutputGpio(zonePins[i]);
  }
  clampOutputGpio(mainsPin);
  clampOutputGpio(tankPin);

  // Optional inputs
  clampGpio(rainSensorPin);
  clampGpio(manualSelectPin);
  clampGpio(manualStartPin);

  // ADC-only pins
  #if defined(CONFIG_IDF_TARGET_ESP32)
  if (!isValidAdcPin(tankLevelPin)) tankLevelPin = 34; // safe default on classic ESP32
  #else
  if (!isValidAdcPin(tankLevelPin)) tankLevelPin = 1;  // safe default on ESP32-S3
  #endif
  if (!isValidPhotoPin(photoPin)) photoPin = -1;

  // TFT pins: fall back to safe defaults if invalid/unsafe
  if (!isValidTftSignalPin(tftSclkPin)) tftSclkPin = TFT_SCLK_DEFAULT;
  if (!isValidTftSignalPin(tftMosiPin)) tftMosiPin = TFT_MOSI_DEFAULT;
  if (!isValidTftSignalPin(tftCsPin))   tftCsPin   = TFT_CS_DEFAULT;
  if (!isValidTftSignalPin(tftDcPin))   tftDcPin   = TFT_DC_DEFAULT;

  // Optional TFT pins
  if (!isValidOptionalTftPin(tftRstPin)) tftRstPin = TFT_RST_DEFAULT;
  if (!isValidOptionalTftPin(tftBlPin))  tftBlPin  = TFT_BL_DEFAULT;

  if (!isValidTftDimension(tftPanelWidth))  tftPanelWidth  = TFT_W_DEFAULT;
  if (!isValidTftDimension(tftPanelHeight)) tftPanelHeight = TFT_H_DEFAULT;

  // I2C pins
  if (!isValidGpioPin(i2cSdaPin)) i2cSdaPin = I2C_SDA_DEFAULT;
  if (!isValidGpioPin(i2cSclPin)) i2cSclPin = I2C_SCL_DEFAULT;
}

static void validatePinMap() {
  // I2C vs sensors
  warnPinConflict("I2C_SDA", i2cSdaPin, "RainSensor", rainSensorPin);
  warnPinConflict("I2C_SCL", i2cSclPin, "RainSensor", rainSensorPin);
  warnPinConflict("I2C_SDA", i2cSdaPin, "TankLevel", tankLevelPin);
  warnPinConflict("I2C_SCL", i2cSclPin, "TankLevel", tankLevelPin);

  // TFT vs board IO
  warnPinConflict("TFT_CS",  tftCsPin,  "TankLevel", tankLevelPin);
  warnPinConflict("TFT_DC",  tftDcPin,  "LED",       LED_PIN);
  warnPinConflict("TFT_CS",  tftCsPin,  "LED",       LED_PIN);
  warnPinConflict("TFT_DC",  tftDcPin,  "TankLevel", tankLevelPin);
  warnPinConflict("TFT_SCLK",tftSclkPin,"RainSensor",rainSensorPin);
  warnPinConflict("TFT_MOSI",tftMosiPin,"RainSensor",rainSensorPin);

  // TFT self-collisions
  warnPinConflict("TFT_SCLK",tftSclkPin,"TFT_MOSI", tftMosiPin);
  warnPinConflict("TFT_CS",  tftCsPin,  "TFT_DC",   tftDcPin);
  warnPinConflict("TFT_CS",  tftCsPin,  "TFT_RST",  tftRstPin);
  warnPinConflict("TFT_CS",  tftCsPin,  "TFT_BL",   tftBlPin);
}

void statusPixelSet(uint8_t r,uint8_t g,uint8_t b) {
  if (!statusPixelReady) return;
  uint32_t color = statusPixel.Color(r,g,b);
  if (color == statusPixelLastColor) return;
  statusPixelLastColor = color;
  statusPixel.setPixelColor(0, color);
  statusPixel.show();
}

uint8_t statusPixelPulseLevel(uint16_t periodMs, uint8_t low, uint8_t high) {
  if (high <= low || periodMs < 2) return high;
  const uint32_t halfPeriod = periodMs / 2U;
  if (halfPeriod == 0) return high;
  const uint32_t phase = millis() % periodMs;
  const uint32_t ramp = (phase < halfPeriod) ? phase : (periodMs - phase);
  const uint32_t span = (uint32_t)(high - low);
  return (uint8_t)(low + ((span * ramp) / halfPeriod));
}

bool statusPixelWindowOn(uint16_t periodMs, uint16_t startMs, uint16_t widthMs) {
  if (periodMs == 0 || widthMs == 0 || startMs >= periodMs) return false;
  const uint32_t phase = millis() % periodMs;
  const uint32_t endMs = (uint32_t)startMs + (uint32_t)widthMs;
  if (endMs <= periodMs) return phase >= startMs && phase < endMs;
  return phase >= startMs || phase < (endMs - periodMs);
}

#if __has_include("soc/soc_caps.h")
  #include "soc/soc_caps.h"
#endif

// SDK compatibility: some IDF/Arduino builds expose SOC_TEMP_SENSOR_SUPPORTED
// instead of SOC_TEMPERATURE_SENSOR_SUPPORTED.
#if !defined(SOC_TEMPERATURE_SENSOR_SUPPORTED) && defined(SOC_TEMP_SENSOR_SUPPORTED)
  #define SOC_TEMPERATURE_SENSOR_SUPPORTED SOC_TEMP_SENSOR_SUPPORTED
#endif

#if defined(SOC_TEMPERATURE_SENSOR_SUPPORTED) && SOC_TEMPERATURE_SENSOR_SUPPORTED && __has_include("driver/temperature_sensor.h")
  #include "driver/temperature_sensor.h"

  static bool tempSensorReady = false;
  static temperature_sensor_handle_t gTempHandle = nullptr;

  bool initTempSensor() {
    if (tempSensorReady && gTempHandle) return true;

    // Choose a realistic range the driver can operate in.
    // This is *chip* temperature, not ambient.
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 60);

    esp_err_t err = temperature_sensor_install(&cfg, &gTempHandle);
    if (err != ESP_OK || !gTempHandle) {
      tempSensorReady = false;
      return false;
    }

    err = temperature_sensor_enable(gTempHandle);
    if (err != ESP_OK) {
      tempSensorReady = false;
      return false;
    }

    tempSensorReady = true;
    return true;
  }

  bool readChipTempC(float& outC) {
    if (!tempSensorReady && !initTempSensor()) return false;

    float c = NAN;
    esp_err_t err = temperature_sensor_get_celsius(gTempHandle, &c);
    if (err == ESP_OK) {
      outC = c;
      return true;
    }
    return false;
  }

#else
  // Boards/chips without supported internal temp sensor
  static bool tempSensorReady = false;

  bool initTempSensor() { tempSensorReady = false; return false; }
  bool readChipTempC(float& outC) { outC = NAN; return false; }
#endif

// ---------- Next Water type + forward decl ----------
struct NextWaterInfo {
  time_t   epoch;    // local epoch for the next start
  int      zone;     // zone index
  uint32_t durSec;   // duration in seconds
};
static NextWaterInfo computeNextWatering();

// ---------- I2C init ----------
bool initExpanders() {
  bool haveIn  = i2cPing(0x22);
  bool haveOut = i2cPing(0x24);
  Serial.printf("[I2C] ping 0x22=%d 0x24=%d\n", haveIn, haveOut);
  if (!haveOut) return false;

  for (int i=0;i<3 && !pcfOut.begin();++i) delay(5);
  for (int i=0;i<3 && !pcfIn.begin(); ++i) delay(5);

  for (uint8_t ch=P0; ch<=P5; ch++) { pcfOut.pinMode(ch, OUTPUT); pcfOut.digitalWrite(ch, HIGH); }
  for (uint8_t ch=P0; ch<=P5; ch++) { pcfIn.pinMode (ch, INPUT);  pcfIn.digitalWrite(ch, HIGH); }
  return true;
}

static bool g_tftInvert = false;
static bool g_tftBlOn   = true;
static bool g_tftDisplayOn = true;
static bool g_tftPwmReady = false;
static uint8_t g_tftBrightness = 125; // 0-255 duty when ON
static const int TFT_PWM_CH = 7;      // LEDC channel for TFT BL
static bool g_forceHomeReset = false; // force full HomeScreen repaint
static bool g_forceRainReset = false; // force full RainScreen repaint
static bool g_forceManualReset = false; // force full Manual screen repaint
static bool g_forceRunReset = false; // force full Running screen repaint

// ---------- LEDC PWM compatibility (ESP32 Arduino core 2.x vs 3.x) ----------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static bool ledcAttachCompat(int pin, uint32_t freq, uint8_t resBits) {
  return ledcAttach(pin, freq, resBits);
}
static void ledcWriteCompat(int pin, uint32_t duty) {
  ledcWrite(pin, duty);
}
#else
static bool ledcAttachCompat(int pin, uint32_t freq, uint8_t resBits) {
  double actual = ledcSetup(TFT_PWM_CH, freq, resBits);
  if (actual <= 0) return false;
  ledcAttachPin(pin, TFT_PWM_CH);
  return true;
}
static void ledcWriteCompat(int /*pin*/, uint32_t duty) {
  ledcWrite(TFT_PWM_CH, duty);
}
#endif

static void tftInitBacklightPwm() {
  if (!displayUseTft) return;
  g_tftPwmReady = false;
  if (tftBlPin < 0) return;
  if (!isValidGpioPin(tftBlPin)) {
    Serial.printf("[TFT] Invalid tftBlPin=%d; disabling backlight pin.\n", tftBlPin);
    tftBlPin = -1;
    return;
  }
  const uint32_t freq = 5000;
  const uint8_t resBits = 8;
  if (!ledcAttachCompat(tftBlPin, freq, resBits)) {
    Serial.println("[TFT] LEDC setup failed; PWM backlight disabled.");
    return;
  }
  g_tftPwmReady = true;
  ledcWriteCompat(tftBlPin, g_tftBlOn ? g_tftBrightness : 0);
}

static inline void tftDisplay(bool on){
  if (!displayUseTft) return;
  if (g_tftDisplayOn == on) return;
  if (on) {
    // Sleep OUT then Display ON (common ST7789/ILI9341 sequence).
    tft.sendCommand(0x11);
    delay(120);
    tft.sendCommand(0x29);
    delay(20);
  } else {
    // Display OFF before Sleep IN to reduce panel power.
    tft.sendCommand(0x28);
    delay(10);
    tft.sendCommand(0x10);
    delay(120);
  }
  g_tftDisplayOn = on;
}

static inline void tftBacklight(bool on){
  if (!displayUseTft) return;
  if (tftBlPin >= 0) {
    if (!isValidGpioPin(tftBlPin)) {
      static bool warned = false;
      if (!warned) {
        Serial.printf("[TFT] Invalid tftBlPin=%d; disabling backlight pin.\n", tftBlPin);
        warned = true;
      }
      tftBlPin = -1;
      // Continue and use panel sleep/wake commands.
    }
    if (tftBlPin >= 0) {
      if (g_tftPwmReady) {
        ledcWriteCompat(tftBlPin, on ? g_tftBrightness : 0);
      } else {
        pinMode(tftBlPin, OUTPUT);
        digitalWrite(tftBlPin, on ? HIGH : LOW);   // most modules: HIGH = on
      }
      g_tftBlOn = on;
    }
  }
  tftDisplay(on);
}

static void tftSetBrightness(uint8_t pct){ // pct: 0-100
  if (!displayUseTft) return;
  if (pct > 100) pct = 100;
  uint8_t duty = map(pct, 0, 100, 0, 255);
  g_tftBrightness = duty;
  if (tftBlPin >= 0) {
    if (g_tftPwmReady) {
      ledcWriteCompat(tftBlPin, g_tftBlOn ? duty : 0);
    } else {
      // if no PWM, fall back to on/off at threshold
      digitalWrite(tftBlPin, (pct > 0) ? HIGH : LOW);
    }
  }
}

static void tickAutoBacklight(){
  if (!displayUseTft) return;
  if (!photoAutoEnabled) return;
  if (!isValidPhotoPin(photoPin)) return;

  static uint32_t lastMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 1000) return;
  lastMs = nowMs;

  const int N = 4;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) { acc += analogRead(photoPin); delayMicroseconds(200); }
  int raw = (int)(acc / N);

  const int hysteresis = 50;
  int low = photoThreshold - hysteresis;
  int high = photoThreshold + hysteresis;
  if (low < 0) low = 0;
  if (high > 4095) high = 4095;

  static bool autoOn = true;
  static uint32_t darkSinceMs = 0;
  const uint32_t DARK_HOLD_MS = 2000; // keep screen on for 10s after it goes dark

  if (!photoInvert) {
    if (raw < low) {
      if (darkSinceMs == 0) darkSinceMs = nowMs;
      if ((nowMs - darkSinceMs) >= DARK_HOLD_MS) autoOn = false;
    } else {
      darkSinceMs = 0;
      if (raw > high) autoOn = true;
    }
  } else {
    if (raw > high) {
      if (darkSinceMs == 0) darkSinceMs = nowMs;
      if ((nowMs - darkSinceMs) >= DARK_HOLD_MS) autoOn = false;
    } else {
      darkSinceMs = 0;
      if (raw < low) autoOn = true;
    }
  }

  bool curOn = (tftBlPin >= 0) ? g_tftBlOn : g_tftDisplayOn;
  if (autoOn != curOn) {
    tftBacklight(autoOn);
  }
}
// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.printf("[BOOT] %s %s\n", __DATE__, __TIME__);

  // Clamp noisy logs (IDF)
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c_master", ESP_LOG_NONE);

  // Route larger heap allocations to PSRAM before long-lived page/cache buffers reserve.
  configurePsramForLargeBuffers();

  // I2C bus
  I2Cbus.begin(i2cSdaPin, i2cSclPin, 100000);
  I2Cbus.setTimeOut(20);

  bootMillis = millis();

  // Pre-size hot Strings to reduce heap churn over long runtimes.
  meteoLocation.reserve(64);
  meteoModel.reserve(24);
  cachedWeatherData.reserve(4096);
  cachedForecastData.reserve(16384);
  lastWeatherError.reserve(96);
  lastForecastError.reserve(96);

  // LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed; formatting...");
    if (!(LittleFS.format() && LittleFS.begin())) {
      Serial.println("LittleFS unavailable; halt.");
      while (true) delay(1000);
    }
  }

  // Config + schedule
  loadConfig();
  sanitizePinConfig();
  validatePinMap();
  if (!LittleFS.exists("/schedule.txt")) saveSchedule();
  loadSchedule();
  initManualButtons();

  mainsChannel = P4; 
  tankChannel  = P5;

  // PCF8574 expanders (or GPIO fallback)
  bool expandersOk = initExpanders();
  if (!expandersOk) {
    Serial.println("PCF8574 relays not found; GPIO fallback.");
    initGpioFallback();
    useGpioFallback = true;
  } else {
    useGpioFallback = false;
    checkI2CHealth();
  }
  initGpioPinsForZones();

  if (LED_PIN >= 0 && isValidGpioPin(LED_PIN)) {
    pinMode(LED_PIN, OUTPUT);
  }

  // Status pixel (WS2812)
  if (STATUS_PIXEL_PIN >= 0) {
    statusPixel.begin();
    statusPixel.setBrightness(32);
    statusPixel.clear();
    statusPixel.show();
    statusPixelLastColor = 0;
    statusPixelReady = true;
    statusPixelSet(0, 0, 20); // boot = blue
  } else {
    statusPixelReady = false;
    statusPixelLastColor = 0;
  }

  initTempSensor(); // try to bring up the internal temp sensor (ESP32-S3)

  // ---------- Display init ----------
  if (displayUseTft) {
    new (&tft) Adafruit_ST7789(&SPI, tftCsPin, tftDcPin, tftRstPin);
    SPI.begin(tftSclkPin, -1, tftMosiPin, tftCsPin);

    tftInitBacklightPwm();

    tft.init(tftPanelWidth, tftPanelHeight);
    tft.setRotation(tftRotation);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(true);
    tft.setTextColor(ST77XX_WHITE);
    tftBacklight(true);

    // Boot splash
    tft.setCursor(10, 20);
    tft.setTextSize(3);
    tft.print("Irrigation");

    tft.setTextSize(2);
    tft.setCursor(10, 70);
    tft.print("AP:");
    tft.setCursor(10, 95);
    tft.setTextSize(2);
    tft.print("ESPIrrigationAP");

    tft.setTextSize(2);
    tft.setCursor(10, 140);
    tft.print("http://192.168.4.1");
  } else {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println("SSD1306 init failed");
      while (true) delay(100);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2); display.setCursor(0, 8);  display.print("Irrigation");
    display.setTextSize(1); display.setCursor(0, 36); display.print("AP: ESPIrrigationAP");
    display.setCursor(0, 50); display.print("http://192.168.4.1");
    display.display();
  }

  // Hostname + WiFi events
  WiFi.setHostname(kHost);
  WiFi.persistent(true);            // keep credentials in NVS across resets
  WiFi.setAutoReconnect(true);      // let core retry silently after power cycles

  // WiFiManager connect
  wifiManager.setWiFiAutoReconnect(true); // ESP32S3 sometimes drops without this
  wifiManager.setConnectRetries(6);       // more chances before giving up
  wifiManager.setConnectTimeout(20);      // seconds per connect attempt
  wifiManager.setSaveConnect(true);       // reconnect immediately after saving
  wifiManager.setTimeout(180);
  wifiManager.setConfigPortalTimeout(0);      // keep portal open until user configures
  wifiManager.setBreakAfterConfig(true);      // exit once credentials are saved
  WiFi.mode(WIFI_STA);                        // ensure STA mode for S2/S3
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("WiFi connect failed -> opening config portal (ESPIrrigationAP).");
    wifiManager.startConfigPortal("ESPIrrigationAP");  // block here until configured
  }

  // Improve HTTP responsiveness
  WiFi.setSleep(false); // NEW: disable modem sleep for snappier responses
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // boost TX within legal limit
  WiFi.enableLongRange(true);          // trade speed for sensitivity
  {
    esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if (err != ESP_OK) {
      Serial.printf("[WiFi] set_protocol failed: %d\n", (int)err);
    }
  }

  if (displayUseTft) {
    tft.fillScreen(ST77XX_BLACK);

    // Connected screen (cleaner layout)
    tft.setTextColor(C_ACCENT);
    tft.setTextSize(3);
    tft.setCursor(10, 16);
    tft.print("Connected");

    tft.setTextColor(C_TEXT);
    tft.setTextSize(2);
    tft.setCursor(10, 58);
    tft.print("IP:");
    tft.setCursor(10, 80);
    tft.print(WiFi.localIP().toString());

    tft.setTextColor(C_MUTED);
    tft.setCursor(10, 112);
    tft.print("mDNS:");
    tft.setTextColor(C_TEXT);
    tft.setCursor(10, 134);
    tft.print("espirrigation.local");

    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(10, 160);
    tft.print("RSSI ");
    tft.print(WiFi.RSSI());
    tft.print(" dBm");

    delay(4000);
  } else {
    display.clearDisplay();
    display.setTextSize(2); display.setCursor(0, 0);  display.print("Connected!");
    display.setTextSize(1); display.setCursor(0, 20); display.print(WiFi.localIP().toString());
    display.setCursor(0, 32); display.print("espirrigation.local");
    display.display();
    delay(2500);
  }

  // Timezone + SNTP
  delay(250);
  applyTimezoneAndSNTP();
  {
    time_t now = time(nullptr);
    struct tm tcheck; localtime_r(&now, &tcheck);
    Serial.printf("[TIME] NTP ok: %04d-%02d-%02d %02d:%02d:%02d tz=%s mode=%d\n",
      tcheck.tm_year+1900, tcheck.tm_mon+1, tcheck.tm_mday,
      tcheck.tm_hour, tcheck.tm_min, tcheck.tm_sec,
      (tcheck.tm_isdst>0) ? "DST" : "STD", (int)tzMode);
  }

  // OTA
  #if ENABLE_OTA
    ArduinoOTA.setHostname(kHost);
    ArduinoOTA.begin();
  #endif

  mdnsStart();

  // -------- Routes --------
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);

  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);

  server.on("/events", HTTP_GET, handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);

  server.on("/tank", HTTP_GET, handleTankCalibration);

  // /status JSON
  server.on("/status", HTTP_GET, [](){
    HttpScope _scope; // NEW: avoid heavy work while serving
    JsonDocument doc;

  doc["rainDelayActive"] = rainActive;
  doc["windDelayActive"] = windActive;
  doc["rainDelayCause"]  = rainDelayCauseText();
  doc["zonesCount"]      = zonesCount;
  doc["tankPct"]         = tankPercent();
  doc["sourceMode"]      = sourceModeText();
  doc["rssi"]            = WiFi.RSSI();
  doc["uptimeSec"]       = (millis() - bootMillis) / 1000;
  doc["heapFree"]        = ESP.getFreeHeap();
  doc["heapMin"]         = ESP.getMinFreeHeap();
  doc["heapMaxAlloc"]    = ESP.getMaxAllocHeap();
  float chipTemp = NAN;
  if (readChipTempC(chipTemp)) doc["chipTempC"] = chipTemp;
  else                         doc["chipTempC"] = nullptr;
  const int tftPct = (int)lroundf((float)g_tftBrightness * 100.0f / 255.0f);
  doc["tftBlPin"]         = tftBlPin;
  doc["tftPwm"]           = g_tftPwmReady;
  doc["tftBlOn"]          = g_tftBlOn;
  doc["tftDisplayOn"]     = g_tftDisplayOn;
  doc["tftBrightnessPct"] = tftPct;
  doc["tftWidth"]         = tftPanelWidth;
  doc["tftHeight"]        = tftPanelHeight;
  doc["displayType"]      = displayUseTft ? "tft" : "oled";

    // Current rain (actuals)
    doc["rain1hNow"] = rain1hNow;
    doc["rain3hNow"] = rain3hNow;

    // Forecast fields
    doc["rain12h"]     = isnan(rainNext12h_mm) ? 0.0f : rainNext12h_mm;
    doc["rain24h"]     = isnan(rainNext24h_mm) ? 0.0f : rainNext24h_mm;
    doc["pop12h"]      = (popNext12h_pct < 0 ? 0 : popNext12h_pct);
    doc["nextRainInH"] = (nextRainIn_h < 0 ? 255 : nextRainIn_h);
    doc["gust24h"]     = isnan(maxGust24h_ms) ? 0.0f : maxGust24h_ms;
    doc["tmin"]        = isnan(todayMin_C) ? 0.0f : todayMin_C;
    doc["tmax"]        = isnan(todayMax_C) ? 0.0f : todayMax_C;
    doc["sunrise"]     = (uint32_t)todaySunrise;
    doc["sunset"]      = (uint32_t)todaySunset;

    // local vs UTC offset
    time_t nowEpoch = time(nullptr);
    struct tm ltm; localtime_r(&nowEpoch, &ltm);
    struct tm gtm; gmtime_r(&nowEpoch,  &gtm);
    int localMin = ltm.tm_hour*60 + ltm.tm_min;
    int utcMin   = gtm.tm_hour*60 + gtm.tm_min;
    int deltaMin = localMin - utcMin;
    if (deltaMin >  12*60) deltaMin -= 24*60;
    if (deltaMin < -12*60) deltaMin += 24*60;

    char sunriseLocal[6] = "--:--";
    char sunsetLocal[6] = "--:--";
    if (todaySunrise > 0) {
      struct tm tsr;
      localtime_r(&todaySunrise, &tsr);
      strftime(sunriseLocal, sizeof(sunriseLocal), "%H:%M", &tsr);
    }
    if (todaySunset > 0) {
      struct tm tss;
      localtime_r(&todaySunset, &tss);
      strftime(sunsetLocal, sizeof(sunsetLocal), "%H:%M", &tss);
    }

    doc["deviceEpoch"]  = (uint32_t)nowEpoch;
    doc["utcOffsetMin"] = deltaMin;
    doc["isDST"]        = (ltm.tm_isdst > 0);
    doc["tzAbbrev"]     = (ltm.tm_isdst>0) ? "DST" : "STD";
    doc["sunriseLocal"] = sunriseLocal;
    doc["sunsetLocal"]  = sunsetLocal;
    doc["weatherHttp"]  = lastWeatherHttpCode;
    doc["forecastHttp"] = lastForecastHttpCode;
    if (lastWeatherError.length())  doc["weatherError"]  = lastWeatherError;
    if (lastForecastError.length()) doc["forecastError"] = lastForecastError;

    // Feature gates
    doc["masterOn"]          = systemMasterEnabled;
    doc["cooldownUntil"]     = rainCooldownUntilEpoch;
    doc["cooldownRemaining"] = (rainCooldownUntilEpoch>nowEpoch) ? (rainCooldownUntilEpoch - nowEpoch) : 0;
    doc["rainThresh24h"]     = rainThreshold24h_mm;
    doc["rainCooldownMin"]   = rainCooldownMin;
    doc["rainCooldownHours"] = rainCooldownMin / 60;
    doc["runConcurrent"] = runZonesConcurrent;

    // Zones snapshot
    JsonArray zones = doc["zones"].to<JsonArray>();
    for (int i=0; i<zonesCount; i++){
      JsonObject z = zones.add<JsonObject>();
      z["active"] = zoneActive[i];
      z["name"]   = zoneNames[i];
      unsigned long rem = 0;
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
        unsigned long total   = zoneRunTotalSec[i];
        if (total == 0) total = durationForSlot(i,1);
        rem = (elapsed < total ? total - elapsed : 0);
      }
      z["remaining"] = rem;
      unsigned long total = zoneRunTotalSec[i];
      if (total == 0) total = durationForSlot(i,1);
      z["totalSec"]  = total;
    }

    // Current weather pass-through from decoded snapshot.
    doc["temp"]       = isfinite(curTempC) ? curTempC : 0.0f;
    doc["feels_like"] = isfinite(curFeelsC) ? curFeelsC : 0.0f;
    doc["humidity"]   = (curHumidityPct >= 0) ? curHumidityPct : 0;
    doc["pressure"]   = isfinite(curPressureHpa) ? curPressureHpa : 0.0f;
    doc["wind"]       = isfinite(curWindMs) ? curWindMs : 0.0f;
    doc["gustNow"]    = isfinite(curGustMs) ? curGustMs : 0.0f;
    doc["windDirText"]= formatWindDirection(curWindDirDeg);
    doc["condMain"]   = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "";
    doc["condDesc"]   = (curWeatherCode >= 0) ? meteoCodeToDesc(curWeatherCode) : "";
    doc["icon"]       = "";
    doc["owmTzSec"]   = curUtcOffsetSec;
    // Always expose location for UI
    doc["cityName"] = meteoLocationLabel();
    doc["meteoModel"] = meteoModel;
    if (isValidLatLon(meteoLat, meteoLon)) {
      doc["lat"] = meteoLat;
      doc["lon"] = meteoLon;
    }

    // Next Water (queue-first)
    {
      NextWaterInfo nw = computeNextWatering();
      if (nw.zone >= 0) {
        doc["nextWaterEpoch"]  = (uint32_t)nw.epoch;
        doc["nextWaterZone"]   = nw.zone;
        doc["nextWaterName"]   = zoneNames[nw.zone];
        doc["nextWaterDurSec"] = nw.durSec;
      } else {
        doc["nextWaterEpoch"]  = 0;
        doc["nextWaterZone"]   = 255;
        doc["nextWaterName"]   = "";
        doc["nextWaterDurSec"] = 0;
      }
    }

    // Pause / delay toggles
    doc["systemPaused"]          = isPausedNow();
    doc["pauseUntil"]            = pauseUntilEpoch;
    doc["rainForecastEnabled"]   = rainDelayFromForecastEnabled;
    doc["rainSensorEnabled"]     = rainSensorEnabled;

    // NEW: actual rolling 24h rainfall
    doc["rain24hActual"] = last24hActualRain();

    String out;
    out.reserve(2200);
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // Time API
  server.on("/api/time", HTTP_GET, [](){
    HttpScope _scope;
    time_t nowEpoch = time(nullptr);
    struct tm lt; localtime_r(&nowEpoch, &lt);
    struct tm gt; gmtime_r(&nowEpoch,  &gt);
    JsonDocument d;
    d["epoch"] = (uint32_t)nowEpoch;
    char buf[32];
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&lt); d["local"] = buf;
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&gt); d["utc"]   = buf;
    d["isDST"] = (lt.tm_isdst>0);
    d["tz"]    = (lt.tm_isdst>0) ? "DST" : "STD";
    String out;
    out.reserve(180);
    serializeJson(d, out);
    server.send(200,"application/json",out);
  });

  // Tank calibration endpoints
  server.on("/setTankEmpty", HTTP_POST, []() {
    HttpScope _scope;
    if (isValidAdcPin(tankLevelPin)) {
      tankEmptyRaw = analogRead(tankLevelPin);
      saveConfig();
      server.sendHeader("Location", "/tank", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(400, "text/plain", "Invalid ADC pin for tank sensor");
    }
  });
  server.on("/setTankFull", HTTP_POST, []() {
    HttpScope _scope;
    if (isValidAdcPin(tankLevelPin)) {
      tankFullRaw = analogRead(tankLevelPin);
      saveConfig();
      server.sendHeader("Location", "/tank", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(400, "text/plain", "Invalid ADC pin for tank sensor");
    }
  });

  // Manual control per zone
  for (int i=0; i<MAX_ZONES; i++){
    server.on(String("/valve/on/")+i,  HTTP_POST, [i](){ HttpScope _s; turnOnValveManual(i);  server.send(200,"text/plain","OK"); });
    server.on(String("/valve/off/")+i, HTTP_POST, [i](){ HttpScope _s; turnOffValveManual(i); server.send(200,"text/plain","OK"); });
  }
  server.on("/stopall", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<MAX_ZONES; ++z) if (zoneActive[z]) turnOffZone(z);
    server.send(200,"text/plain","OK");
  });
  server.on("/toggleBacklight", HTTP_POST, [](){
    HttpScope _scope;
    toggleBacklight();
    server.send(200,"text/plain","Display toggled");
  });

  // I2C tools
#if ENABLE_DEBUG_ROUTES
  server.on("/i2c-test", HTTP_GET, [](){
    HttpScope _scope;
    if (useGpioFallback) { server.send(500,"text/plain","Fallback active"); return; }
    for (uint8_t ch : PCH) { pcfOut.digitalWrite(ch, LOW); delay(100); pcfOut.digitalWrite(ch, HIGH); delay(60); }
    server.send(200,"text/plain","PCF8574 pulse OK");
  });
  server.on("/whereami", HTTP_GET, [](){
    HttpScope _scope;
    String out;
    out.reserve(96);
    out += "ip=";   out += WiFi.localIP().toString(); out += "\n";
    out += "ssid="; out += WiFi.SSID();               out += "\n";
    out += "mac=";  out += WiFi.macAddress();
    server.send(200,"text/plain",out);
  });
#endif
  
  // Downloads / Admin
  server.on("/download/config.txt", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/config.txt")){ File f=LittleFS.open("/config.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/schedule.txt", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/schedule.txt")){ File f=LittleFS.open("/schedule.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/events.csv", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/events.csv")){ File f=LittleFS.open("/events.csv","r"); server.streamFile(f,"text/csv"); f.close(); }
    else server.send(404,"text/plain","No event log");
  });
  server.on("/tft_selftest", HTTP_GET, [](){
    HttpScope _scope;
    if (!displayUseTft) {
      server.send(400, "text/plain", "Display mode is OLED");
      return;
    }
    String msg;
    bool pinsOk = true;
    auto chk = [&](const char* name, int pin, bool allowNeg){
      if (pin == -1 && allowNeg) return;
      if ((allowNeg && !isValidOptionalTftPin(pin)) || (!allowNeg && !isValidTftSignalPin(pin))) {
        pinsOk = false;
        msg += String(name) + " invalid/unsafe: " + String(pin) + "\n";
      }
    };
    chk("TFT_SCLK", tftSclkPin, false);
    chk("TFT_MOSI", tftMosiPin, false);
    chk("TFT_CS",   tftCsPin,   false);
    chk("TFT_DC",   tftDcPin,   false);
    chk("TFT_RST",  tftRstPin,  true);
    chk("TFT_BL",   tftBlPin,   true);

    if (!pinsOk) {
      server.send(500, "text/plain", msg.length() ? msg : "Invalid TFT pins");
      return;
    }

    const bool oldBlOn = g_tftBlOn;
    const bool oldDisp = g_tftDisplayOn;
    const uint8_t oldDuty = g_tftBrightness;

    tftInitBacklightPwm();

    tft.fillScreen(ST77XX_RED);   delay(120);
    tft.fillScreen(ST77XX_GREEN); delay(120);
    tft.fillScreen(ST77XX_BLUE);  delay(120);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.print("TFT SELF-TEST");
    tft.setTextSize(1);
    tft.setCursor(10, 36);
    tft.print("SCLK "); tft.print(tftSclkPin);
    tft.print(" MOSI "); tft.print(tftMosiPin);
    tft.print(" CS "); tft.print(tftCsPin);
    tft.setCursor(10, 52);
    tft.print("DC "); tft.print(tftDcPin);
    tft.print(" RST "); tft.print(tftRstPin);
    tft.print(" BL "); tft.print(tftBlPin);

    if (tftBlPin >= 0) {
      if (g_tftPwmReady) {
        tftSetBrightness(100); delay(120);
        tftSetBrightness(20);  delay(120);
        tftSetBrightness(80);  delay(120);
      } else {
        tftBacklight(false); delay(150);
        tftBacklight(true);  delay(150);
      }
    }

    if (tftBlPin >= 0) {
      const int oldPct = (int)lroundf((float)oldDuty * 100.0f / 255.0f);
      tftSetBrightness((uint8_t)oldPct);
      tftBacklight(oldBlOn);
    } else {
      tftDisplay(oldDisp);
    }
    g_forceHomeReset = true;

    msg = "TFT self-test OK\n";
    msg += "PWM: "; msg += (g_tftPwmReady ? "yes" : "no");
    msg += "\nBacklight: ";
    msg += (tftBlPin >= 0 ? (g_tftBlOn ? "on" : "off") : (g_tftDisplayOn ? "display-on" : "display-off"));
    server.send(200, "text/plain", msg);
  });
  server.on("/tft_brightness", HTTP_POST, [](){
    HttpScope _scope;
    if (!displayUseTft) { server.send(400, "text/plain", "Display mode is OLED"); return; }
    if (tftBlPin < 0) { server.send(500,"text/plain","No backlight pin"); return; }
    int lvl = server.hasArg("level") ? server.arg("level").toInt() : 100;
    if (lvl < 0) lvl = 0; if (lvl > 100) lvl = 100;
    tftSetBrightness((uint8_t)lvl);
    server.send(200,"text/plain","OK");
  });
  server.on("/reboot", HTTP_POST, [](){
    HttpScope _scope;
    server.send(200,"text/plain","restarting"); delay(200); ESP.restart();
  }); 
  // Pause/Resume/Delays/Forecast toggle
  server.on("/clear_delays", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<(int)zonesCount; ++z) pendingStart[z] = false;
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    for (int z=0; z<(int)zonesCount; ++z) lastCheckedMinute[z] = -1;
    server.send(200,"text/plain","OK");
  });
  server.on("/pause", HTTP_POST, [](){
    HttpScope _scope;
    time_t nowEp = time(nullptr);
    String secStr = server.arg("sec");
    uint32_t sec = secStr.length()? secStr.toInt() : (24u*3600u);
    systemPaused = true;
    pauseUntilEpoch = sec ? (nowEp + sec) : 0;
    saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/resume", HTTP_POST, [](){
    HttpScope _scope;
    systemPaused = false; pauseUntilEpoch = 0; saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/set_rain_forecast", HTTP_POST, [](){
    HttpScope _scope;
    rainDelayFromForecastEnabled = server.hasArg("on");
    saveConfig();
    server.send(200,"text/plain", rainDelayFromForecastEnabled ? "on" : "off");
  });

  // Master and cooldown
  server.on("/master", HTTP_POST, [](){
    HttpScope _scope;
    systemMasterEnabled = server.hasArg("on");  // Dashboard only (not in Setup)
    saveConfig();
    server.send(200,"text/plain", systemMasterEnabled ? "on" : "off");
  });
  server.on("/clear_cooldown", HTTP_POST, [](){
    HttpScope _scope;
    rainCooldownUntilEpoch = 0;
    server.send(200,"text/plain","OK");
  });

  server.begin();

  // MQTT
  mqttSetup();
  mqttEnsureConnected();
}

// ---------- Loop ----------
void loop() {
  const uint32_t now = millis();
  #if ENABLE_OTA
  ArduinoOTA.handle();
  #endif

  server.handleClient();  // serve UI first to keep it responsive
  wifiCheck();
  checkWindRain();
  mqttEnsureConnected();
  if (mqttEnabled) _mqtt.loop();
  mqttPublishStatus(); 
  tickWeather(); // Timed weather fetch after servicing HTTP to avoid blocking the UI
  tickManualButtons();
  tickAutoBacklight();

  // Track transitions to clear/refresh screens
  static bool lastWasDelayScreen = false;

  const bool hardBlock = (!systemMasterEnabled || isPausedNow());
  const bool cooldownActive = isCooldownActiveNow();
  const bool manualDelayBypassActive = hasActiveManualZone();
  const bool delayScreenActive = !manualDelayBypassActive &&
                                 (hardBlock || cooldownActive || rainActive || windActive);

  if (hardBlock || cooldownActive || rainActive || windActive) {
    stopAutoZonesForBlock();
  }

  if (delayScreenActive) {
    if (!lastWasDelayScreen) g_forceRainReset = true;
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); lastWasDelayScreen = true; }
  } else if (lastWasDelayScreen) {
    // Only clear once delay has actually ended
    lastWasDelayScreen = false;
    if (displayUseTft) {
      tft.fillScreen(C_BG);
    } else {
      display.clearDisplay();
      display.display();
    }
    lastScreenRefresh = 0; // force immediate HomeScreen paint
    g_forceHomeReset = true;
  }

  if (now - lastTimeQuery >= TIME_QUERY_MS) {
    lastTimeQuery = now;
    time_t nowTime = time(nullptr);
    localtime_r(&nowTime, &cachedTm);
  }

  if (cachedTm.tm_hour == 0 && cachedTm.tm_min == 0) {
    if (!midnightDone) {
      memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
      todayMin_C = NAN;
      todayMax_C = NAN;
      midnightDone = true;
    }
  } else midnightDone = false;

  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now; checkI2CHealth();
  }

  // WS2812 status pixel (non-blocking, light refresh)
  static uint32_t lastPixelUpdate = 0;
  if (statusPixelReady && now - lastPixelUpdate >= 100) {
    lastPixelUpdate = now;
    updateStatusPixel();
  }

  bool anyActive=false;
  for (int z=0; z<(int)zonesCount; z++) if (zoneActive[z]) { anyActive=true; break; }

  // -------- SCHEDULER (supports sequential or concurrent) --------
  if (now - lastScheduleTick >= SCHEDULE_TICK_MS) {
    lastScheduleTick = now;

    for (int z=0; z<(int)zonesCount; z++) {
      if (zoneActive[z] && hasDurationCompleted(z)) turnOffZone(z);
    }

    if (!isBlockedNow()) {
      for (int z=0; z<(int)zonesCount; z++) {
        if (shouldStartZone(z)) {
          // Cancel when blocked/rain; queue during wind so it can run later.
          if (isBlockedNow()) { cancelStart(z, "BLOCKED", false); continue; }
          if (rainActive)     { cancelStart(z, "RAIN",    true ); continue; }
          if (windActive)     { pendingStart[z] = true; logEvent(z,"QUEUED","WIND",false); continue; }

          if (!runZonesConcurrent) {
            // sequential: only start if nothing is running; else queue (still allowed)
            if (anyActive) { pendingStart[z] = true; logEvent(z,"QUEUED","ACTIVE RUN",false); }
            else { turnOnZone(z); anyActive = true; }
          } else {
            // concurrent: start regardless of other active zones
            turnOnZone(z); anyActive = true;
          }
        }
      }

      if (!runZonesConcurrent) {
        // sequential: drain one queued zone if nothing currently running
        if (!anyActive && !rainActive && !windActive) {
          for (int z=0; z<(int)zonesCount; z++) if (pendingStart[z]) { pendingStart[z]=false; turnOnZone(z); break; }
        }
      } else if (!rainActive && !windActive) {
        // concurrent: start any queued zones once delays clear
        for (int z=0; z<(int)zonesCount; z++) {
          if (pendingStart[z]) { pendingStart[z]=false; turnOnZone(z); anyActive = true; }
        }
      }
    }
  }

  bool manualActive = (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - now) > 0);
  if (!manualActive && manualScreenUntilMs != 0) {
    manualScreenUntilMs = 0;
    g_forceHomeReset = true;
    lastScreenRefresh = 0;
  }

  if (!delayScreenActive && now - lastScreenRefresh >= 1000) {
    lastScreenRefresh = now;
    if (manualActive) {
      drawManualSelection();
    } else {
      int activeZone = -1;
      for (int z = 0; z < (int)zonesCount; ++z) {
        if (zoneActive[z]) { activeZone = z; break; }
      }
      if (activeZone >= 0) updateLCDForZone(activeZone);
      else HomeScreen();
    }
  }
  delay(LOOP_SLEEP_MS);
}

// ---------- Connectivity / I2C health ----------
void wifiCheck() {
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect(false, false);   // do NOT clear saved credentials
    WiFi.reconnect();

    // wait up to ~8s for reconnect using stored creds (no portal)
    unsigned long t0 = millis();
    while (WiFi.status()!=WL_CONNECTED && (millis() - t0) < 8000UL) {
      delay(250);
    }

    if (WiFi.status()==WL_CONNECTED) {
      Serial.println("Reconnected.");
      WiFi.setSleep(false); // keep disabled after reconnect
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      WiFi.enableLongRange(true);
      esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
      if (err != ESP_OK) {
        Serial.printf("[WiFi] set_protocol failed: %d\n", (int)err);
      }
    } else {
      Serial.println("Reconnection failed (kept creds, not opening portal).");
    }
  }
}

void checkI2CHealth() {
  delay(20);
  bool anyErr=false;
  for (uint8_t addr : expanderAddrs) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission()!=0) { anyErr=true; break; }
  }
  if (anyErr) {
    i2cFailCount++;
    if (i2cFailCount >= I2C_HEALTH_DEBOUNCE) {
      Serial.println("I2C unstable ? GPIO fallback");
      useGpioFallback = true; initGpioFallback();
    }
  } else i2cFailCount = 0;
}

void initGpioFallback() {
  useGpioFallback = true;

  for (uint8_t i = 0; i < MAX_ZONES; i++) {
    gpioInitOutput(zonePins[i], zoneGpioActiveLow[i]);
  }

  gpioInitOutput(mainsPin, mainsGpioActiveLow);
  gpioInitOutput(tankPin, tankGpioActiveLow);
}

void initGpioPinsForZones() {
  // Initialise GPIO outputs for any zones not handled by the PCF expanders.
  for (uint8_t i = 0; i < zonesCount && i < MAX_ZONES; i++) {
    if (useExpanderForZone(i)) continue;
    gpioInitOutput(zonePins[i], zoneGpioActiveLow[i]);
  }

  // Prepare mains/tank GPIO pins when using fallback or when running more than 4 zones
  // (so PCF pins remain dedicated to zone control).
  if (useGpioFallback || zonesCount > 4) {
    gpioInitOutput(mainsPin, mainsGpioActiveLow);
    gpioInitOutput(tankPin, tankGpioActiveLow);
  }
}

// ---------- Manual hardware buttons (select + start/stop) ----------
static void resetManualButtonState(ManualButtonState& state, int pin) {
  int level = HIGH;
  if (pin >= 0) level = digitalRead(pin);
  state.stableState = level;
  state.lastRawState = level;
  state.lastRawChangeMs = millis();
  state.lastPressMs = 0;
}

static bool consumeManualButtonPress(int pin, ManualButtonState& state, uint32_t nowMs) {
  if (pin < 0) return false;

  const int raw = digitalRead(pin);
  if (raw != state.lastRawState) {
    state.lastRawState = raw;
    state.lastRawChangeMs = nowMs;
  }
  if ((uint32_t)(nowMs - state.lastRawChangeMs) < MANUAL_BTN_DEBOUNCE_MS) return false;
  if (raw == state.stableState) return false;

  state.stableState = raw;
  if (raw != LOW) return false;
  if ((uint32_t)(nowMs - state.lastPressMs) < MANUAL_BTN_REPEAT_GUARD_MS) return false;

  state.lastPressMs = nowMs;
  return true;
}

static int firstActiveZoneIndex() {
  for (int i = 0; i < (int)zonesCount; ++i) {
    if (zoneActive[i]) return i;
  }
  return -1;
}

void showManualSelection() {
  manualScreenUntilMs = millis() + 15000UL;
  g_forceManualReset = true;
  drawManualSelection();
}

void drawManualSelection() {
  const int selectedZone = manualSelectedZone % zonesCount;
  const bool selectedActive = zoneActive[selectedZone];
  const char* selectedZoneName = zoneNames[selectedZone].c_str();

  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Manual");
    display.setCursor(0, 24);
    display.print(selectedZoneName);
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("State: ");
    display.print(selectedActive ? "ON" : "OFF");
    display.display();
    return;
  }

  const int W = tft.width();
  const int H = tft.height();
  static bool init = false;
  static int lastW = -1;
  static int lastH = -1;
  static int lastZone = -1;
  static int lastStatusId = -1;
  static int lastActiveZone = -2;
  static bool lastQueued = false;
  static bool lastPinReady = false;
  static bool lastSelectedActive = false;

  const bool layoutChanged = g_forceManualReset || !init || lastW != W || lastH != H;
  const int activeZone = firstActiveZoneIndex();
  const bool selectedQueued = pendingStart[selectedZone];
  const bool pinReady = useExpanderForZone(selectedZone) ||
                        (zonePins[selectedZone] >= 0 && zonePins[selectedZone] <= 39);

  int statusId = 0;
  const char* statusText = "READY";
  uint16_t statusColor = C_GOOD;
  char detailLine[64] = "Press Start to toggle";

  if (!pinReady) {
    statusId = 1;
    statusText = "NO PIN";
    statusColor = C_BAD;
    snprintf(detailLine, sizeof(detailLine), "Assign an output pin for %s", selectedZoneName);
  } else if (selectedActive) {
    statusId = 2;
    statusText = "ACTIVE";
    statusColor = C_GOOD;
    snprintf(detailLine, sizeof(detailLine), "Running now");
  } else if (selectedQueued) {
    statusId = 3;
    statusText = "QUEUED";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "Waiting for wind or active run");
  } else if (isBlockedNow()) {
    statusId = 4;
    statusText = "BLOCKED";
    statusColor = C_BAD;
    if (!systemMasterEnabled) snprintf(detailLine, sizeof(detailLine), "Master switch is off");
    else if (isPausedNow()) snprintf(detailLine, sizeof(detailLine), "System pause is active");
    else snprintf(detailLine, sizeof(detailLine), "Rain cooldown is active");
  } else if (rainActive) {
    statusId = 5;
    statusText = "RAIN";
    statusColor = C_BAD;
    snprintf(detailLine, sizeof(detailLine), "Rain delay is active");
  } else if (windActive) {
    statusId = 7;
    statusText = "WIND";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "Start will queue until wind clears");
  } else if (!runZonesConcurrent && activeZone >= 0 && activeZone != selectedZone) {
    statusId = 6;
    statusText = "BUSY";
    statusColor = C_WARN;
    snprintf(detailLine, sizeof(detailLine), "%s is already running", zoneNames[activeZone].c_str());
  }

  if (layoutChanged) {
    tft.fillScreen(C_BG);
    drawTopBar("Manual", "BTN", C_ACCENT);

    int cardW = W - 24;
    int cardH = 108;
    int cardX = 12;
    int cardY = (H - cardH) / 2 - 10;
    if (cardY < 44) cardY = 44;
    drawCard(cardX, cardY, cardW, cardH, C_PANEL, C_EDGE);
    tft.drawFastHLine(cardX + 1, cardY + 58, cardW - 2, C_EDGE);
    tft.drawFastHLine(cardX + 1, cardY + 84, cardW - 2, C_EDGE);

    init = true;
    lastW = W;
    lastH = H;
    lastZone = -1;
    lastStatusId = -1;
    lastActiveZone = -2;
    lastQueued = false;
    lastPinReady = false;
    lastSelectedActive = false;
    g_forceManualReset = false;
  }

  if (layoutChanged || lastZone != selectedZone || lastStatusId != statusId ||
      lastActiveZone != activeZone || lastQueued != selectedQueued || lastPinReady != pinReady ||
      lastSelectedActive != selectedActive) {
    int cardW = W - 24;
    int cardH = 108;
    int cardX = 12;
    int cardY = (H - cardH) / 2 - 10;
    if (cardY < 44) cardY = 44;

    int pillW = 10 + (int)strlen(statusText) * 6;
    int pillX = cardX + cardW - pillW - 12;
    int titleX = cardX + 12;
    int maxTitleW = pillX - titleX - 8;
    int16_t titleX1 = 0, titleY1 = 0;
    uint16_t titleW = 0, titleH = 0;
    uint8_t titleSize = 2;
    tft.setTextSize(titleSize);
    tft.getTextBounds(selectedZoneName, 0, 0, &titleX1, &titleY1, &titleW, &titleH);
    if ((int)titleW > maxTitleW) {
      titleSize = 1;
      tft.setTextSize(titleSize);
      tft.getTextBounds(selectedZoneName, 0, 0, &titleX1, &titleY1, &titleW, &titleH);
    }

    tft.fillRect(cardX + 10, cardY + 10, cardW - 20, cardH - 20, C_PANEL);
    tft.setTextColor(C_TEXT);
    tft.setTextSize(titleSize);
    tft.setCursor(titleX, cardY + (titleSize == 2 ? 18 : 22));
    tft.print(selectedZoneName);

    tft.fillRect(pillX, cardY + 16, pillW, 18, statusColor);
    tft.drawRect(pillX, cardY + 16, pillW, 18, C_EDGE);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(pillX + 5, cardY + 21);
    tft.print(statusText);

    tft.setTextColor(C_MUTED);
    tft.setCursor(cardX + 12, cardY + 66);
    tft.print(detailLine);

    const char* stateText = selectedActive ? "ON" : "OFF";
    uint16_t stateColor = selectedActive ? C_GOOD : C_BAD;
    int statePillW = 24;
    int statePillX = cardX + 12;
    int statePillY = cardY + 88;
    tft.fillRect(statePillX, statePillY, statePillW, 14, stateColor);
    tft.drawRect(statePillX, statePillY, statePillW, 14, C_EDGE);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(statePillX + 5, statePillY + 4);
    tft.print(stateText);
    tft.setTextColor(C_TEXT);
    tft.setCursor(cardX + 42, cardY + 92);
    tft.print("Selected zone is ");
    tft.print(stateText);
    tft.setTextColor(C_MUTED);
    tft.setCursor(cardX + 168, cardY + 92);
    tft.print("| Start toggles");

    lastZone = selectedZone;
    lastStatusId = statusId;
    lastActiveZone = activeZone;
    lastQueued = selectedQueued;
    lastPinReady = pinReady;
    lastSelectedActive = selectedActive;
  }
}

void initManualButtons() {
  if (manualSelectedZone >= zonesCount) manualSelectedZone = 0;

  if (manualSelectPin >= 0 && manualSelectPin <= 39) {
    pinMode(manualSelectPin, INPUT_PULLUP);
    resetManualButtonState(g_manualSelectBtn, manualSelectPin);
  } else {
    manualSelectPin = -1;
    resetManualButtonState(g_manualSelectBtn, -1);
  }

  if (manualStartPin >= 0 && manualStartPin <= 39) {
    pinMode(manualStartPin, INPUT_PULLUP);
    resetManualButtonState(g_manualStartBtn, manualStartPin);
  } else {
    manualStartPin = -1;
    resetManualButtonState(g_manualStartBtn, -1);
  }
}

void tickManualButtons() {
  const uint32_t nowMs = millis();
  if (zonesCount == 0) return;
  manualSelectedZone = manualSelectedZone % zonesCount;

  if (consumeManualButtonPress(manualSelectPin, g_manualSelectBtn, nowMs)) {
    manualSelectedZone = (manualSelectedZone + 1) % zonesCount;
    Serial.printf("[BTN] Manual select -> Z%d\n", manualSelectedZone + 1);
    showManualSelection();
  }

  if (consumeManualButtonPress(manualStartPin, g_manualStartBtn, nowMs)) {
    uint8_t z = manualSelectedZone % zonesCount;
    if (zoneActive[z]) {
      turnOffValveManual(z);
    } else {
      turnOnValveManual(z);
    }
  }
}

// ---------- Weather / Forecast ----------
String fetchWeather() {
  if (!isValidLatLon(meteoLat, meteoLon)) return "";
  String model = cleanMeteoModel(meteoModel);
  lastWeatherError = "";
  lastWeatherHttpCode = 0;

  auto buildUrl = [&](bool useForecastEndpoint) -> String {
    String url = meteoBaseUrl(model, useForecastEndpoint);
    url += "?latitude=" + String(meteoLat,6) + "&longitude=" + String(meteoLon,6);
    url += "&current=temperature_2m,relative_humidity_2m,apparent_temperature,pressure_msl,surface_pressure,"
           "wind_speed_10m,wind_gusts_10m,wind_direction_10m,precipitation,weather_code";
    if (useForecastEndpoint) url += "&models=" + model;
    url += "&temperature_unit=celsius&wind_speed_unit=ms&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
    return url;
  };

  for (int pass = 0; pass < 2; ++pass) {
    bool useForecastEndpoint = (pass == 1);
    String url = buildUrl(useForecastEndpoint);
    int code = 0;
    String payload = httpGetMeteo(url, code, 2500);
    lastWeatherHttpCode = code;

    if (code == 200 && payload.length() && !isMeteoErrorPayload(payload)) {
      if (payload.indexOf("\"current\"") != -1) return payload;
      lastWeatherError = "No current in response";
    } else {
      String reason = meteoErrorReason(payload);
      if (reason.length()) lastWeatherError = reason;
      else if (code != 200) lastWeatherError = "HTTP " + String(code);
      else lastWeatherError = "Empty payload";
    }
  }

  // Fallback: derive current from hourly
  String hourlyPayload = fetchWeatherHourlyForCurrent(model, meteoLat, meteoLon, false);
  if (hourlyPayload.length()) {
    String out;
    if (buildCurrentFromHourlyPayload(hourlyPayload, out)) return out;
  }
  String hourlyPayloadForecast = fetchWeatherHourlyForCurrent(model, meteoLat, meteoLon, true);
  if (hourlyPayloadForecast.length()) {
    String out;
    if (buildCurrentFromHourlyPayload(hourlyPayloadForecast, out)) return out;
  }
  return "";
}

String fetchWeatherHourlyForCurrent(const String& model, float lat, float lon, bool useForecastEndpoint) {
  if (!isValidLatLon(lat, lon)) return "";
  String url = meteoBaseUrl(model, useForecastEndpoint);
  url += "?latitude=" + String(lat,6) + "&longitude=" + String(lon,6);
  url += "&hourly=temperature_2m,relative_humidity_2m,apparent_temperature,pressure_msl,surface_pressure,"
         "wind_speed_10m,wind_gusts_10m,wind_direction_10m,precipitation,weather_code";
  if (useForecastEndpoint) url += "&models=" + model;
  url += "&forecast_hours=48&timeformat=unixtime&temperature_unit=celsius&wind_speed_unit=ms"
         "&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
  int code = 0;
  String payload = httpGetMeteo(url, code, 3000);
  if (code != 200 || isMeteoErrorPayload(payload)) return "";
  return payload;
}

bool buildCurrentFromHourlyPayload(const String& hourlyPayload, String& outPayload) {
  if (isMeteoErrorPayload(hourlyPayload)) return false;
  JsonDocument js;
  if (deserializeJson(js, hourlyPayload) != DeserializationError::Ok) {
    return false;
  }
  JsonObject hourly = js["hourly"].as<JsonObject>();
  JsonArray timeArr = hourly["time"].as<JsonArray>();
  if (!timeArr.size()) return false;

  time_t now = time(nullptr);
  int bestIdx = 0;
  long bestDiff = LONG_MAX;
  for (int i = 0; i < (int)timeArr.size(); ++i) {
    long t = timeArr[i].as<long>();
    long d = labs(t - (long)now);
    if (d < bestDiff) {
      bestDiff = d;
      bestIdx = i;
    }
  }

  auto pick = [&](const char* key) -> float {
    JsonArray arr = hourly[key].as<JsonArray>();
    if (!arr.size() || bestIdx >= (int)arr.size()) return NAN;
    return arr[bestIdx].as<float>();
  };

  JsonDocument out;
  out["utc_offset_seconds"] = js["utc_offset_seconds"] | 0;
  JsonObject cur = out["current"].to<JsonObject>();
  cur["time"] = timeArr[bestIdx] | 0;
  cur["temperature_2m"]       = pick("temperature_2m");
  cur["relative_humidity_2m"] = pick("relative_humidity_2m");
  cur["apparent_temperature"] = pick("apparent_temperature");
  cur["pressure_msl"]         = pick("pressure_msl");
  cur["surface_pressure"]     = pick("surface_pressure");
  cur["wind_speed_10m"]       = pick("wind_speed_10m");
  cur["wind_gusts_10m"]       = pick("wind_gusts_10m");
  cur["wind_direction_10m"]   = pick("wind_direction_10m");
  cur["precipitation"]        = pick("precipitation");
  float wcode = pick("weather_code");
  cur["weather_code"]         = isfinite(wcode) ? (int)wcode : -1;

  outPayload = "";
  serializeJson(out, outPayload);
  return outPayload.length() > 0;
}

String fetchForecast(float lat, float lon) {
  if (!isValidLatLon(lat, lon)) return "";
  String model = cleanMeteoModel(meteoModel);
  lastForecastError = "";
  lastForecastHttpCode = 0;

  auto buildUrl = [&](bool useForecastEndpoint) -> String {
    String url = meteoBaseUrl(model, useForecastEndpoint);
    url += "?latitude=" + String(lat,6) + "&longitude=" + String(lon,6);
    url += "&hourly=precipitation,precipitation_probability,wind_gusts_10m"
           "&daily=temperature_2m_min,temperature_2m_max,sunrise,sunset"
           "&forecast_hours=24&forecast_days=2";
    if (useForecastEndpoint) url += "&models=" + model;
    url += "&temperature_unit=celsius&wind_speed_unit=ms&precipitation_unit=mm&pressure_unit=hPa&timezone=auto";
    return url;
  };

  for (int pass = 0; pass < 2; ++pass) {
    bool useForecastEndpoint = (pass == 1);
    String url = buildUrl(useForecastEndpoint);
    int code = 0;
    String payload = httpGetMeteo(url, code, 3000);
    lastForecastHttpCode = code;
    if (code == 200 && payload.length() && !isMeteoErrorPayload(payload)) {
      return payload;
    }
    String reason = meteoErrorReason(payload);
    if (reason.length()) lastForecastError = reason;
    else if (code != 200) lastForecastError = "HTTP " + String(code);
    else lastForecastError = "Empty payload";
  }
  return "";
}

// ---------- NEW helpers for rain history ----------
// Rolling 24h rain history (ACTUAL ONLY, no forecast).
// Record once per elapsed hour instead of relying on a narrow HH:00 window.
static void tickActualRainHistory() {
  time_t now = time(nullptr);
  if (now <= 0) return;

  const time_t thisHour = now - (now % 3600);
  float v = rain1hNow;

  // Sanity: NaN / negative => 0
  if (!isfinite(v) || v < 0.0f) {
    v = 0.0f;
  }

  // Seed the current hour immediately so reboots or loop jitter do not
  // require hitting a tiny HH:00 window before history starts moving again.
  if (lastRainHistHour == 0) {
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = v;
    lastRainHistHour = thisHour;
    return;
  }

  if (thisHour <= lastRainHistHour) {
    return;
  }

  int hoursElapsed = (int)((thisHour - lastRainHistHour) / 3600);
  if (hoursElapsed > 24) hoursElapsed = 24;

  // Advance one slot per elapsed hour to preserve a true rolling 24h window.
  // If several hours were skipped, fill the gap with zeros and store the
  // latest observed rain value in the newest hour slot.
  for (int h = 1; h <= hoursElapsed; ++h) {
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = (h == hoursElapsed) ? v : 0.0f;
  }

  lastRainHistHour = thisHour;
}

// Sum rolling 24h actual rainfall from the ring buffer
static float last24hActualRain() {
  float total = 0.0f;
  for (int i = 0; i < 24; ++i) {
    float v = rainHist[i];
    if (!isfinite(v) || v < 0.0f) {
      v = 0.0f;
    }
    total += v;
  }
  return total;
}

bool refreshCurrentWeatherSnapshotFromCache() {
  curTempC = NAN;
  curFeelsC = NAN;
  curPressureHpa = NAN;
  curWindMs = NAN;
  curGustMs = NAN;
  curWindDirDeg = NAN;
  curHumidityPct = -1;
  curWeatherCode = -1;
  curUtcOffsetSec = 0;
  curWeatherValid = false;
  rain1hNow = 0.0f;
  rain3hNow = 0.0f;

  if (!cachedWeatherData.length()) return false;

  JsonDocument js;
  if (deserializeJson(js, cachedWeatherData) != DeserializationError::Ok) {
    return false;
  }

  JsonObject cur = js["current"].as<JsonObject>();
  curTempC = cur["temperature_2m"] | NAN;
  curFeelsC = cur["apparent_temperature"] | NAN;
  curHumidityPct = cur["relative_humidity_2m"] | -1;
  float pmsl = cur["pressure_msl"] | NAN;
  float psfc = cur["surface_pressure"] | NAN;
  curPressureHpa = isfinite(pmsl) ? pmsl : psfc;
  curWindMs = cur["wind_speed_10m"] | NAN;
  curGustMs = cur["wind_gusts_10m"] | NAN;
  curWindDirDeg = cur["wind_direction_10m"] | NAN;
  curWeatherCode = cur["weather_code"] | -1;
  curUtcOffsetSec = js["utc_offset_seconds"] | 0;

  float r1 = cur["precipitation"] | 0.0f;
  if (!isfinite(r1) || r1 < 0.0f) r1 = 0.0f;
  rain1hNow = r1;
  rain3hNow = 0.0f;

  curWeatherValid = true;
  return true;
}



void updateCachedWeather() {
  // NEW: never start fetches while handling HTTP requests
  if (g_inHttp) return;

  unsigned long nowms = millis();
  bool needCur = (cachedWeatherData == "" ||
                  (nowms - lastWeatherUpdate >= weatherUpdateInterval));
  bool haveCoord = isValidLatLon(meteoLat, meteoLon);
  bool weatherChanged = false;

  if (needCur) {
    String fresh = fetchWeather();
    if (fresh.length() > 0) {
      weatherChanged = (fresh != cachedWeatherData);
      cachedWeatherData = fresh;
      lastWeatherUpdate = nowms;
    }
  }

  // Decode once when weather payload changes (or first run).
  static bool snapshotReady = false;
  if (!snapshotReady || weatherChanged || !curWeatherValid) {
    refreshCurrentWeatherSnapshotFromCache();
    snapshotReady = true;
  }

  // ---- Forecast fetch / parse ----
  if (haveCoord && (cachedForecastData == "" || (nowms - lastForecastUpdate >= forecastUpdateInterval))) {
    String freshFc = fetchForecast(meteoLat, meteoLon);
    if (freshFc.length() > 0) {
      cachedForecastData = freshFc;
      lastForecastUpdate = nowms;

      rainNext12h_mm = 0; 
      rainNext24h_mm = 0; 
      popNext12h_pct = 0; 
      nextRainIn_h   = -1;
      maxGust24h_ms  = 0; 
      todaySunrise   = 0;   
      todaySunset    = 0;

      JsonDocument fc;
      if (deserializeJson(fc, cachedForecastData) == DeserializationError::Ok) {
        JsonObject daily = fc["daily"].as<JsonObject>();
        JsonArray sunr = daily["sunrise"].as<JsonArray>();
        JsonArray suns = daily["sunset"].as<JsonArray>();
        JsonArray tmin = daily["temperature_2m_min"].as<JsonArray>();
        JsonArray tmax = daily["temperature_2m_max"].as<JsonArray>();

        if (sunr.size() > 0) todaySunrise = parseLocalIsoTime(sunr[0] | "");
        if (suns.size() > 0) todaySunset  = parseLocalIsoTime(suns[0] | "");
        if (tmin.size() > 0) todayMin_C   = tmin[0] | todayMin_C;
        if (tmax.size() > 0) todayMax_C   = tmax[0] | todayMax_C;

        JsonObject hourly = fc["hourly"].as<JsonObject>();
        JsonArray prec = hourly["precipitation"].as<JsonArray>();
        JsonArray pop  = hourly["precipitation_probability"].as<JsonArray>();
        JsonArray gust = hourly["wind_gusts_10m"].as<JsonArray>();

        int hrs = max((int)prec.size(), max((int)pop.size(), (int)gust.size()));
        int L24 = min(24, hrs);
        int L12 = min(12, L24);
        for (int i = 0; i < L24; i++) {
          float r = (prec.size() > i) ? prec[i].as<float>() : 0.0f;
          if (!isfinite(r) || r < 0.0f) r = 0.0f;
          if (i < L12) {
            rainNext12h_mm += r;
            int p = (pop.size() > i) ? pop[i].as<int>() : 0;
            if (p > popNext12h_pct) popNext12h_pct = p;
          }
          rainNext24h_mm += r;
          if (nextRainIn_h < 0) {
            int p = (pop.size() > i) ? pop[i].as<int>() : 0;
            if (r > 0.1f || p >= 50) nextRainIn_h = i;
          }
          float g = (gust.size() > i) ? gust[i].as<float>() : 0.0f;
          if (g > maxGust24h_ms) maxGust24h_ms = g;
        }
      }
      if (isnan(rainNext12h_mm) || rainNext12h_mm < 0) rainNext12h_mm = 0.0f;
      if (isnan(rainNext24h_mm) || rainNext24h_mm < 0) rainNext24h_mm = 0.0f;
      if (isnan(maxGust24h_ms)  || maxGust24h_ms  < 0) maxGust24h_ms  = 0.0f;
    }
  }

  // Fallback min/max from current weather snapshot
  if (isfinite(curTempC)) {
    if (!isfinite(todayMin_C) || curTempC < todayMin_C) todayMin_C = curTempC;
    if (!isfinite(todayMax_C) || curTempC > todayMax_C) todayMax_C = curTempC;
  }

  // Roll hourly history
  tickActualRainHistory();
}

// NEW: called only from loop()
void tickWeather(){
  static uint32_t lastTick = 0;
  uint32_t nowMs = millis();
  const uint32_t WEATHER_TICK_MS = 10000UL; // every 10s is plenty

  if (nowMs - lastTick < WEATHER_TICK_MS) {
    return;
  }
  lastTick = nowMs;

  updateCachedWeather();
}

// ---------- Rain/Wind logic with cooldown & threshold ----------
bool checkWindRain() {
  // Throttle to at most once per second
  static uint32_t lastCheckMs = 0;
  static bool     lastResult  = false;
  // Track only the kind of "rain" that is allowed to start cooldown
  static bool     prevCoolRain = false;

  uint32_t nowMs = millis();
  if (nowMs - lastCheckMs < 1000UL) {
    // Reuse last computed state (rainActive/windActive already set)
    return lastResult;
  }
  lastCheckMs = nowMs;

  bool newWeatherRainActual = false;   // raw "is it raining now" from Open-Meteo
  bool newSensorRainActual  = false;   // raw sensor state
  bool newWindActual        = false;   // raw wind above threshold

  // --- 1) Use decoded current-weather snapshot (no JSON parse in fast loop) ---
  if (curWeatherValid) {
    float rainNow = rain1hNow;
    if (!isfinite(rainNow) || rainNow < 0.0f) rainNow = 0.0f;
    const float MAX_RAIN1H_FOR_LOGIC = 20.0f;  // mm
    if (rainNow > MAX_RAIN1H_FOR_LOGIC) rainNow = MAX_RAIN1H_FOR_LOGIC;

    if (rainNow > 0.0f) {
      newWeatherRainActual = true;
    } else if (curWeatherCode >= 0 && meteoCodeIsWet(curWeatherCode)) {
      newWeatherRainActual = true;
    }

    if (windSpeedThreshold > 0.0f && isfinite(curWindMs)) {
      newWindActual = (curWindMs >= windSpeedThreshold);
    }
  }

  // --- 2) Physical rain sensor (raw state) ---
  newSensorRainActual = physicalRainNowRaw();

  // --- 3) Accumulated rainfall (24h ACTUAL ONLY) ---
  float actual24 = last24hActualRain();
  bool aboveThreshold = false;
  if (rainThreshold24h_mm > 0) {
    if (actual24 >= rainThreshold24h_mm) {
      aboveThreshold = true;
    }
  }

  // --- 4) Apply feature gates and build "effective" flags ---
  bool effectiveInstantRain =
      rainDelayEnabled &&
      rainDelayFromForecastEnabled &&
      newWeatherRainActual;

  // 24h threshold rain: only when actual 24h rainfall >= user threshold.
  bool effectiveThresholdRain =
      rainDelayEnabled &&
      rainDelayFromForecastEnabled &&
      (rainThreshold24h_mm > 0) &&
      aboveThreshold;

  // Physical sensor rain: used for both blocking + cooldown.
  bool effectiveSensorRain =
      rainDelayEnabled &&
      rainSensorEnabled &&
      newSensorRainActual;

  // Public flags / state
  rainByWeatherActive = effectiveInstantRain || effectiveThresholdRain;
  rainBySensorActive  = effectiveSensorRain;
  rainActive          = (rainByWeatherActive || rainBySensorActive);

  windActive = (windDelayEnabled && newWindActual);

  // --- 5) Cooldown logic (ONLY threshold/sensor-based rain) ---
  // Define which rain sources are allowed to start cooldown when they clear:
  //  - 24h threshold actual rainfall
  //  - Physical rain sensor
  bool coolRainNow = (effectiveThresholdRain || effectiveSensorRain);

  time_t now = time(nullptr);

  // (a) If we *previously* had threshold/sensor rain, and now it's clear,
  //     start cooldown window.
  if (prevCoolRain && !coolRainNow) {
    if (rainDelayEnabled && rainCooldownHours > 0) {
      rainCooldownUntilEpoch =
          (uint32_t)now + (uint32_t)rainCooldownHours * 3600UL;
    } else {
      rainCooldownUntilEpoch = 0;
    }
  }

  // (b) While threshold/sensor rain is active, there is no cooldown.
  if (coolRainNow) {
    rainCooldownUntilEpoch = 0;
  }

  // (c) When cooldown expires, clear it.
  if (!coolRainNow &&
      rainCooldownUntilEpoch > 0 &&
      (uint32_t)now >= rainCooldownUntilEpoch) {
    rainCooldownUntilEpoch = 0;
  }

  // Track last 24h amount for UI (RainScreen "Last: x.xx mm")
  lastRainAmount = actual24;

  // Remember for next edge detection
  prevCoolRain = coolRainNow;

  // Final combined state that the rest of the code cares about
  lastResult = (rainActive || windActive);
  return lastResult;
}

// ---------- Event log ----------
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed) {
  updateCachedWeather(); // safe early-out if g_inHttp==true, keeps details recent enough
  float temp = curTempC;
  float wind = curWindMs;
  int hum = (curHumidityPct >= 0) ? curHumidityPct : 0;
  String cond = (curWeatherCode >= 0) ? meteoCodeToMain(curWeatherCode) : "?";
  String cname = meteoLocationLabel();

  File f = LittleFS.open("/events.csv","a");
  if (!f) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char ts[32];
  sprintf(ts,"%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

  String line; line.reserve(200);
  line += ts; line += ","; line += (zone>=0?zoneNames[zone]:String("n/a")); line += ",";
  line += eventType; line += ","; line += source; line += ",";
  line += (rainDelayed?"Active":"Off"); line += ",";
  line += String(temp,1); line += ","; line += String(hum); line += ",";
  line += String(wind,1); line += ","; line += cond; line += ","; line += cname; line += "\n";

  f.print(line); f.close();
}

// ---------- OLED UI ----------
void toggleBacklight(){
  if (!displayUseTft) {
    static bool inverted=false;
    inverted=!inverted;
    display.invertDisplay(inverted);
    return;
  }
  static bool on = true;
  on = !on;
  tftBacklight(on);
}

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate=0; unsigned long now=millis();
  const unsigned long minUiInterval = displayUseTft ? 180UL : 1000UL;
  if (!g_forceRunReset && (now - lastUpdate < minUiInterval)) return;
  lastUpdate = now;

  unsigned long elapsed=(now - zoneStartMs[zone]) / 1000;
  unsigned long total = zoneRunTotalSec[zone];
  if (total == 0) total = durationForSlot(zone,1);
  unsigned long rem = (elapsed<total ? total - elapsed : 0);

  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0); display.print(zoneNames[zone]); display.print(" ");
    display.print(elapsed/60); display.print(":"); if ((elapsed%60)<10) display.print('0'); display.print(elapsed%60);
    display.setCursor(0,12);
    if (elapsed<total){ display.print(rem/60); display.print("m Remaining"); }
    else display.print("Complete");
    display.display();
    return;
  }

  unsigned long em = elapsed / 60;
  unsigned long es = elapsed % 60;
  unsigned long rm = rem / 60;
  unsigned long rs = rem % 60;
  int pct = (total > 0) ? (int)((elapsed * 100UL) / total) : 0;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  const int W = tft.width();
  const int H = tft.height();
  const int pad = 8;
  const int topY = 40;
  int statsH = 64;
  int meterH = 74;
  int infoH = 60;
  int statsY = topY;
  int meterY = statsY + statsH + 6;
  int infoY = meterY + meterH + 6;
  int bottomGap = H - infoY - infoH - 8;
  if (bottomGap < 0) {
    meterH += bottomGap;
    if (meterH < 58) meterH = 58;
    infoY = meterY + meterH + 6;
    bottomGap = H - infoY - infoH - 8;
    if (bottomGap < 0) {
      infoH += bottomGap;
      if (infoH < 50) infoH = 50;
      infoY = meterY + meterH + 6;
    }
  }
  const int barX = pad + 14;
  const int barY = meterY + 40;
  const int barW = W - (pad + 14) * 2;
  const int barH = 18;
  const uint16_t C_RUN_INFO = RGB(20, 29, 47);
  const uint16_t C_RUN_STATS = RGB(17, 25, 40);
  const uint16_t C_RUN_METER = RGB(14, 21, 34);
  const uint16_t C_LIVE = RGB(95, 182, 255);

  static bool runInit = false;
  static int lastZone = -1;
  static int lastW = -1, lastH = -1;
  static unsigned long lastElapsed = 0xFFFFFFFFUL;
  static unsigned long lastRemain = 0xFFFFFFFFUL;
  static unsigned long lastTotal = 0xFFFFFFFFUL;
  static int lastPct = -1;
  static int lastPulseFrame = -1;

  bool layoutChanged = (g_forceRunReset || !runInit || lastZone != zone || lastW != W || lastH != H);
  if (layoutChanged) {
    tft.fillScreen(C_BG);
    drawTopBar("RUNNING", "LIVE", C_GOOD);
    drawCard(pad, statsY, W - 2 * pad, statsH, C_RUN_STATS, C_EDGE);
    drawCard(pad, meterY, W - 2 * pad, meterH, C_RUN_METER, C_EDGE);
    drawCard(pad, infoY, W - 2 * pad, infoH, C_RUN_INFO, C_EDGE);
    tft.drawFastHLine(pad + 1, statsY + 24, W - 2 * pad - 2, C_EDGE);
    tft.drawFastHLine(pad + 1, meterY + 24, W - 2 * pad - 2, C_EDGE);
    tft.drawFastHLine(pad + 1, infoY + 22, W - 2 * pad - 2, C_EDGE);
    tft.drawRect(barX, barY, barW, barH, C_EDGE);
    for (int tick = 1; tick < 10; ++tick) {
      int tx = barX + (tick * (barW - 2)) / 10;
      tft.drawFastVLine(tx, barY + 2, barH - 4, RGB(44, 62, 90));
    }
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 10, statsY + 8); tft.print("ELAPSED");
    tft.setCursor(pad + 112, statsY + 8); tft.print("REMAIN");
    tft.setCursor(pad + 10, meterY + 8); tft.print("FLOW PROGRESS");
    tft.setCursor(pad + 10, infoY + 8);  tft.print("ACTIVE ZONE");
    tft.setCursor(W - 78, infoY + 8);    tft.print("LIVE");
    runInit = true;
    lastZone = zone;
    lastW = W;
    lastH = H;
    lastElapsed = 0xFFFFFFFFUL;
    lastRemain = 0xFFFFFFFFUL;
    lastTotal = 0xFFFFFFFFUL;
    lastPct = -1;
    lastPulseFrame = -1;
    g_forceRunReset = false;
  }

  char elapsedBuf[8], remainBuf[8], totalBuf[8];
  snprintf(elapsedBuf, sizeof(elapsedBuf), "%02lu:%02lu", em, es);
  snprintf(remainBuf, sizeof(remainBuf), "%02lu:%02lu", rm, rs);
  unsigned long tm = total / 60;
  unsigned long ts = total % 60;
  snprintf(totalBuf, sizeof(totalBuf), "%02lu:%02lu", tm, ts);

  if (layoutChanged || lastZone != zone) {
    tft.fillRect(pad + 10, infoY + 28, W - 2 * pad - 20, infoH - 36, C_RUN_INFO);
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT);
    tft.setCursor(pad + 12, infoY + 30);
    tft.print("Z");
    tft.print(zone + 1);
    tft.print("  ");
    tft.print(zoneNames[zone]);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 12, infoY + infoH - 14);
    tft.print("TOTAL ");
    tft.setTextColor(C_TEXT);
    tft.print(totalBuf);
    lastZone = zone;
  }

  if (layoutChanged || elapsed != lastElapsed || rem != lastRemain || total != lastTotal) {
    tft.fillRect(pad + 10, statsY + 30, W - 2 * pad - 20, statsH - 38, C_RUN_STATS);
    tft.setTextSize(2);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(pad + 12, statsY + 34);
    tft.print(elapsedBuf);
    tft.setTextColor(C_GOOD);
    tft.setCursor(pad + 116, statsY + 34);
    tft.print(remainBuf);
    lastElapsed = elapsed;
    lastRemain = rem;
    lastTotal = total;
  }

  if (layoutChanged || pct != lastPct) {
    tft.fillRect(pad + 10, meterY + 28, W - 2 * pad - 20, 18, C_RUN_METER);
    tft.setTextSize(2);
    tft.setTextColor(C_LIVE);
    tft.setCursor(pad + 12, meterY + 28);
    tft.print("Progress");
    tft.setTextColor(C_TEXT);
    tft.setCursor(W - 78, meterY + 28);
    tft.print(pct);
    tft.print("%");

    tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, C_BG);
    int fillW = (pct * (barW - 2)) / 100;
    if (fillW > 0) {
      tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, C_GOOD);
      if (fillW > 6) tft.fillRect(barX + 1, barY + 1, fillW - 4, 4, RGB(145, 234, 192));
    }
    lastPct = pct;
  }

  int pulseFrame = (int)((now / 450UL) & 1UL);
  if (layoutChanged || pulseFrame != lastPulseFrame) {
    const int px = W - 26;
    const int py = infoY + 12;
    tft.fillRect(px - 7, py - 7, 20, 16, C_RUN_INFO);
    tft.fillCircle(px, py, 4, pulseFrame ? C_GOOD : RGB(34, 60, 48));
    tft.drawCircle(px, py, 6, C_EDGE);
    lastPulseFrame = pulseFrame;
  }
}

void RainScreen(){
  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2); display.setCursor(0,0); display.print(isPausedNow()? "System Pause" : "Rain/Wind");
    display.setTextSize(1);
    display.setCursor(0,20); display.printf("Last: %.2f mm", lastRainAmount);
    display.setCursor(0,32); display.print("Cause: "); display.print(rainDelayCauseText());
    int delayed=0; for (int i=0;i<(int)zonesCount;i++) if (pendingStart[i]) delayed++;
    display.setCursor(0,46); display.printf("Queued: %d", delayed);
    display.display();
    return;
  }

  const int W = tft.width();
  const int H = tft.height();
  const int pad = 8;
  const int topY = 38; // start a bit higher for compact cards
  const int rainH = 60;
  const int rainY = topY;
  const int statusY = rainY + rainH + 6;
  int causeH = H - statusY - 8;
  if (causeH < 96) causeH = 96;

  static bool init = false;
  static int lastW = -1;
  static int lastH = -1;
  static int lastStatusId = -1;
  static char lastR1[12] = "";
  static char lastR24[12] = "";
  static char lastWind[16] = "";
  static char lastCause[40] = "";
  static char lastCooldown[24] = "";
  static char lastPauseMaster[32] = "";
  static int lastQueued = -1;
  static int lastRunning = -1;
  static int lastRssi = 9999;
  static bool lastMqtt = false;

  if (g_forceRainReset) {
    init = false;
    g_forceRainReset = false;
  }

  const bool paused = isPausedNow();
  const bool masterOff = !systemMasterEnabled;
  const bool delay = (rainActive || windActive);
  const int statusId = masterOff ? 3 : (paused ? 2 : (delay ? 1 : 0));

  bool layoutChanged = (!init || lastW != W || lastH != H);
  if (layoutChanged) {
    tft.fillScreen(C_BG);
    lastW = W;
    lastH = H;
    init = true;
  }

  if (layoutChanged || statusId != lastStatusId) {
    const char* pill = "READY";
    uint16_t pillColor = C_GOOD;
    if (masterOff) { pill = "MASTER OFF"; pillColor = C_BAD; }
    else if (paused) { pill = "PAUSED"; pillColor = C_WARN; }
    else if (delay) { pill = "DELAY"; pillColor = C_WARN; }
    drawTopBar("RAIN / WIND", pill, pillColor);
    lastStatusId = statusId;
  }

  int y = topY;
  if (layoutChanged) {
    drawCard(pad, y, W - 2*pad, rainH, C_PANEL, C_EDGE);
    const int colW = (W - 2 * pad) / 3;
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 4, y + 4); tft.print("1h");
    tft.setCursor(pad + colW + 4, y + 4); tft.print("24h");
    tft.setCursor(pad + 2 * colW + 4, y + 4); tft.print("Wind");

    y += rainH + 6;
    drawCard(pad, y, W - 2*pad, causeH, C_PANEL, C_EDGE);
    tft.setTextSize(2);
    tft.setTextColor(C_MUTED);
    tft.setCursor(pad + 6, y + 4);
    tft.print("Status");
  }

  const int colW = (W - 2 * pad) / 3;
  const int text2H = 18;
  const int text1H = 10;

  float windNow = curWindMs;

  char r1buf[12];
  char r24buf[12];
  char windbuf[16];

  if (isfinite(rain1hNow)) snprintf(r1buf, sizeof(r1buf), "%.1fmm", rain1hNow);
  else strncpy(r1buf, "--", sizeof(r1buf));

  if (isfinite(lastRainAmount)) snprintf(r24buf, sizeof(r24buf), "%.1fmm", lastRainAmount);
  else strncpy(r24buf, "--", sizeof(r24buf));

  if (isfinite(windNow)) snprintf(windbuf, sizeof(windbuf), "%.1f/%.1f", windNow, windSpeedThreshold);
  else snprintf(windbuf, sizeof(windbuf), "--/%.1f", windSpeedThreshold);

  r1buf[sizeof(r1buf) - 1] = '\0';
  r24buf[sizeof(r24buf) - 1] = '\0';
  windbuf[sizeof(windbuf) - 1] = '\0';

  auto updateColValue = [&](int idx, const char* val, char* last, size_t lastLen) {
    if (!layoutChanged && strcmp(val, last) == 0) return;
    int x = pad + idx * colW + 4;
    int yv = rainY + 24;
    int w = colW - 8;
    tft.fillRect(x, yv, w, text2H, C_PANEL);
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT);
    tft.setCursor(x, yv);
    tft.print(val);
    strncpy(last, val, lastLen);
    last[lastLen - 1] = '\0';
  };

  updateColValue(0, r1buf, lastR1, sizeof(lastR1));
  updateColValue(1, r24buf, lastR24, sizeof(lastR24));
  updateColValue(2, windbuf, lastWind, sizeof(lastWind));

  String causeS = rainDelayCauseText();
  const time_t now = time(nullptr);
  const bool cooldownActive = (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch);
  char causeBuf[40];
  if (cooldownActive) {
    strncpy(causeBuf, "Cooldown", sizeof(causeBuf));
  } else if (causeS.length()) {
    strncpy(causeBuf, causeS.c_str(), sizeof(causeBuf));
  } else {
    strncpy(causeBuf, "--", sizeof(causeBuf));
  }
  causeBuf[sizeof(causeBuf) - 1] = '\0';

  if (layoutChanged || strcmp(causeBuf, lastCause) != 0) {
    int x = pad + 6;
    int yv = statusY + 20;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text2H, C_PANEL);
    uint16_t causeColor = C_TEXT;
    if (masterOff) causeColor = C_BAD;
    else if (paused || delay || cooldownActive) causeColor = C_WARN;
    else causeColor = C_GOOD;
    tft.setTextSize(2);
    tft.setTextColor(causeColor);
    tft.setCursor(x, yv);
    tft.print(causeBuf);
    strncpy(lastCause, causeBuf, sizeof(lastCause));
    lastCause[sizeof(lastCause) - 1] = '\0';
  }

  char cooldownBuf[24];
  if (cooldownActive) {
    uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - now);
    uint32_t mins = (rem + 59U) / 60U;
    if (mins >= 60U) {
      uint32_t h = mins / 60U;
      uint32_t m = mins % 60U;
      if (m > 0U) snprintf(cooldownBuf, sizeof(cooldownBuf), "%luh %lum", (unsigned long)h, (unsigned long)m);
      else snprintf(cooldownBuf, sizeof(cooldownBuf), "%luh", (unsigned long)h);
    } else {
      snprintf(cooldownBuf, sizeof(cooldownBuf), "%lum", (unsigned long)mins);
    }
  } else {
    strncpy(cooldownBuf, "None", sizeof(cooldownBuf));
  }
  cooldownBuf[sizeof(cooldownBuf) - 1] = '\0';

  if (layoutChanged || strcmp(cooldownBuf, lastCooldown) != 0) {
    int x = pad + 6;
    int yv = statusY + 38;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("Cooldown: ");
    tft.setTextColor(C_TEXT);
    tft.print(cooldownBuf);
    strncpy(lastCooldown, cooldownBuf, sizeof(lastCooldown));
    lastCooldown[sizeof(lastCooldown) - 1] = '\0';
  }

  char pmBuf[32];
  snprintf(pmBuf, sizeof(pmBuf), "Pause %s | Master %s", paused ? "Yes" : "No", systemMasterEnabled ? "On" : "Off");
  if (layoutChanged || strcmp(pmBuf, lastPauseMaster) != 0) {
    int x = pad + 6;
    int yv = statusY + 50;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("Pause ");
    tft.setTextColor(C_TEXT);
    tft.print(paused ? "Yes" : "No");
    tft.setTextColor(C_MUTED);
    tft.print(" | Master ");
    tft.setTextColor(C_TEXT);
    tft.print(systemMasterEnabled ? "On" : "Off");
    strncpy(lastPauseMaster, pmBuf, sizeof(lastPauseMaster));
    lastPauseMaster[sizeof(lastPauseMaster) - 1] = '\0';
  }

  int queued = 0;
  int running = 0;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (pendingStart[i]) queued++;
    if (zoneActive[i]) running++;
  }

  if (layoutChanged || running != lastRunning || queued != lastQueued) {
    int x = pad + 6;
    int yv = statusY + 62;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("Run ");
    tft.setTextColor(C_TEXT);
    tft.print(running);
    tft.setTextColor(C_MUTED);
    tft.print(" | Q ");
    tft.setTextColor(C_TEXT);
    tft.print(queued);
    lastRunning = running;
    lastQueued = queued;
  }

  int rssi = WiFi.RSSI();
  bool mqttOk = _mqtt.connected();
  bool rssiChanged = (abs(rssi - lastRssi) >= 4);
  if (layoutChanged || rssiChanged || mqttOk != lastMqtt) {
    int x = pad + 6;
    int yv = statusY + 74;
    int w = W - 2 * pad - 12;
    tft.fillRect(x, yv, w, text1H, C_PANEL);
    tft.setTextSize(1);
    uint16_t rssiColor = (rssi > -60) ? C_GOOD : (rssi > -75 ? C_WARN : C_BAD);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, yv);
    tft.print("WiFi ");
    tft.setTextColor(rssiColor);
    tft.print(rssi);
    tft.setTextColor(C_MUTED);
    tft.print("dBm | MQTT ");
    tft.setTextColor(mqttOk ? C_GOOD : C_BAD);
    tft.print(mqttOk ? "up" : "down");
    lastRssi = rssi;
    lastMqtt = mqttOk;
  }
}

void HomeScreen() {
  if (!displayUseTft) {
    float tempOled = curTempC;
    int   humOled  = curHumidityPct;
    int   pctOled  = tankPercent();
    time_t nowOled = time(nullptr);
    struct tm* tOled = localtime(&nowOled);

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0); display.printf("%02d:%02d", tOled ? tOled->tm_hour : 0, tOled ? tOled->tm_min : 0);
    display.setTextSize(1);
    if (tOled) {
      display.setCursor(80,3); display.printf("%02d/%02d", tOled->tm_mday, tOled->tm_mon+1);
      display.setCursor(66,3); display.print((tOled->tm_isdst>0) ? "DST" : "STD");
    }
    display.setCursor(0,20);
    if (!isnan(tempOled) && humOled >= 0) display.printf("T:%2.0fC H:%02d%%", tempOled, humOled);
    else display.print("T:-- H:--");
    display.setCursor(0,32);
    if (tankEnabled) { display.printf("Tank:%3d%% ", pctOled); display.print(isTankLow()? "(Mains)":"(Tank)"); }
    else { display.print("Zones: "); display.print(zonesCount); }
    for (int i=0;i<i_min(2,(int)zonesCount);i++){
      int x=(i==0)?0:64;
      display.setCursor(x,44);
      if (zoneActive[i]) {
        unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
        unsigned long total=zoneRunTotalSec[i];
        if (total==0) total = durationForSlot(i,1);
        unsigned long rem=(elapsed<total?total-elapsed:0);
        display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
      } else display.printf("Z%d:Off", i+1);
    }
    for (int i=2;i<i_min(4,(int)zonesCount);i++){
      int x=(i==2)?0:64;
      display.setCursor(x,56);
      if (zoneActive[i]) {
        unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
        unsigned long total=zoneRunTotalSec[i];
        if (total==0) total = durationForSlot(i,1);
        unsigned long rem=(elapsed<total?total-elapsed:0);
        display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
      } else display.printf("Z%d:Off", i+1);
    }
    display.display();
    return;
  }

  float temp = curTempC;
  int   hum  = curHumidityPct;
  float windNow = curWindMs;
  int   pct  = tankPercent();

  time_t now = time(nullptr);
  struct tm* tmv = localtime(&now);
  uint32_t nowMs = millis();

  const bool delayed = (rainActive || windActive || isPausedNow() || !systemMasterEnabled);

  const int W = tft.width();
  const int H = tft.height();

  const bool landscape = (H < W);

  // Common metrics
  const int pad = 6;
  const int gap = 6;
  const int topY = 44;

  // Header: big date + status pill + RSSI
  static char lastTopDate[16] = "";
  static int  lastTopRssi = 9999;
  static int  lastTopMinute = -1;
  static int  lastTopStatus = -1;
  static uint32_t lastTopDrawMs = 0;
  char topDate[16] = "--/--/----";
  if (tmv) snprintf(topDate, sizeof(topDate), "%02d/%02d/%04d", tmv->tm_mday, tmv->tm_mon + 1, tmv->tm_year + 1900);
  int topRssi = WiFi.RSSI();
  int curMinute = tmv ? tmv->tm_min : -1;

  // Only redraw when date changes OR minute ticks OR RSSI changes notably (>=2dB) with a 15s debounce
  bool minuteChanged = (tmv && curMinute != lastTopMinute);
  bool dateChanged   = (strcmp(topDate, lastTopDate) != 0);
  bool rssiChanged   = (abs(topRssi - lastTopRssi) >= 2) && (nowMs - lastTopDrawMs >= 15000U);
  int statusId = !systemMasterEnabled ? 3 : (isPausedNow() ? 2 : ((rainActive || windActive) ? 1 : 0));
  bool statusChanged = (statusId != lastTopStatus);

  if (g_forceHomeReset) {
    lastTopDate[0] = '\0';
    lastTopRssi = 9999;
    lastTopMinute = -1;
    lastTopStatus = -1;
    lastTopDrawMs = 0;
  }

  if (dateChanged || minuteChanged || rssiChanged || statusChanged) {
    tft.fillRect(0, 0, W, topY - 8, C_BG);

    // Date (left)
    tft.setTextSize(3);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(6, 6);
    tft.print(topDate);

    // Status pill (right)
    const char* statusText = "READY";
    uint16_t statusColor = C_GOOD;
    if (!systemMasterEnabled) { statusText = "MASTER OFF"; statusColor = C_BAD; }
    else if (isPausedNow())   { statusText = "PAUSED";     statusColor = C_WARN; }
    else if (rainActive || windActive) { statusText = "DELAY"; statusColor = C_WARN; }

    tft.setTextSize(1);
    int16_t pbx, pby; uint16_t pbw, pbh;
    tft.getTextBounds(statusText, 0, 0, &pbx, &pby, &pbw, &pbh);
    int pillW = (int)pbw + 10;
    int pillH = (int)pbh + 6;
    int pillX = W - pillW - 6;
    int pillY = 8;
    tft.fillRoundRect(pillX, pillY, pillW, pillH, 4, statusColor);
    tft.drawRoundRect(pillX, pillY, pillW, pillH, 4, C_EDGE);
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(pillX + 5, pillY + 3);
    tft.print(statusText);

    // RSSI (small, left of pill if space)
    tft.setTextSize(1);
    uint16_t rssiColor = (topRssi > -60) ? C_GOOD : (topRssi > -75 ? C_WARN : C_BAD);
    tft.setTextColor(rssiColor);
    char rssiTxt[16]; snprintf(rssiTxt, sizeof(rssiTxt), "%ddBm", topRssi);
    int16_t rbx, rby; uint16_t rbw, rbh;
    tft.getTextBounds(rssiTxt, 0, 0, &rbx, &rby, &rbw, &rbh);
    int rssiX = pillX - (int)rbw - 6;
    if (rssiX > 6) {
      tft.setCursor(rssiX, 12);
      tft.print(rssiTxt);
    }

    strncpy(lastTopDate, topDate, sizeof(lastTopDate));
    lastTopDate[sizeof(lastTopDate) - 1] = '\0';
    lastTopRssi = topRssi;
    lastTopMinute = curMinute;
    lastTopStatus = statusId;
    lastTopDrawMs = nowMs;
  }

  // -------- LANDSCAPE (H < W): two-column layout --------
  if (landscape) {
    static bool init = false;
    static bool lastDelayed = false;
    static int lastW = -1;
    static int lastH = -1;
    static int lastZonesCount = -1;
    static char lastTime[6] = "";
    static char lastDate[20] = "";
    static int lastTemp = -1000;
    static float lastTempF = NAN;
    static char lastTempArrow = '-';
    static int lastMinT = 1000;
    static int lastMaxT = 1000;
    static int lastGustT = 1000;
    static int lastHum = -2;
    static int lastPct = -1;
    static char lastNextZone[12] = "";
    static char lastNextEta[12] = "";
    static char lastNextRem[16] = "";
    static bool lastDelayLine = false;
    static bool lastMasterLine = false;
    static String lastCauseLine;
    static String lastZoneNameShort;
    static int lastWindTenths = 10000;
    static int lastRainTenths = 10000;
    static bool lastZoneState[MAX_ZONES] = {false};
    static uint32_t lastZoneRem[MAX_ZONES] = {0};
    if (g_forceHomeReset) {
      init = false; lastDelayed = false; lastW = lastH = -1; lastZonesCount = -1;
      lastTime[0] = lastDate[0] = '\0';
      lastTemp = -1000; lastHum = -2; lastPct = -1;
      lastTempF = NAN; lastTempArrow = '-';
      lastMinT = 1000; lastMaxT = 1000; lastGustT = 1000;
      lastNextZone[0] = lastNextEta[0] = lastNextRem[0] = '\0';
      lastDelayLine = false; lastMasterLine = false; lastCauseLine = ""; lastZoneNameShort = "";
      lastWindTenths = 10000; lastRainTenths = 10000;
      for (int i=0;i<MAX_ZONES;i++){ lastZoneState[i]=false; lastZoneRem[i]=0; }
      g_forceHomeReset = false;
    }

    const int contentY = topY;
    const int bottomH = delayed ? 22 : 0;
    const int contentH = H - contentY - bottomH - 4;
    const int colGap = 8;

    const int leftW = 118;
    const int rightW = W - 2 * pad - colGap - leftW;
    const int leftX = pad;
    const int rightX = leftX + leftW + colGap;
    const int statsH = 46; // slightly taller to fit tank bar

    const bool layoutChanged = (!init || lastW != W || lastH != H || lastZonesCount != (int)zonesCount);
    const bool stateChanged = (delayed != lastDelayed);

    if (layoutChanged) {
      init = true;
      lastW = W;
      lastH = H;
      lastZonesCount = zonesCount;
      tft.fillScreen(C_BG);
    }
    lastDelayed = delayed;

    if (layoutChanged) {
      // Left: time card
      drawCard(leftX, contentY, leftW, contentH, C_PANEL, C_EDGE);

      // Right: weather/tank card
      drawCard(rightX, contentY, rightW, statsH, C_PANEL, C_EDGE);

      // Right: zones grid
      const int zonesY = contentY + statsH + gap;
      const int zonesH = contentY + contentH - zonesY;
      drawCard(rightX, zonesY, rightW, zonesH, C_PANEL, C_EDGE);

      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, zonesY + 4);
      tft.print("Zones");
    }

    // Time + date (top bar handles date; keep card for time + status lines)
    char tbuf[6] = "--:--";
    if (tmv) snprintf(tbuf, sizeof(tbuf), "%02d:%02d", tmv->tm_hour, tmv->tm_min);
    if (layoutChanged || strcmp(tbuf, lastTime) != 0) {
      tft.fillRect(leftX + 4, contentY + 4, leftW - 8, 28, C_PANEL);
      tft.setTextColor(C_ACCENT);
      tft.setTextSize(3);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(tbuf, 0, 0, &x1, &y1, &tw, &th);
      int timeY = contentY + 6;
      tft.setCursor(leftX + (leftW - (int)tw) / 2, timeY);
      tft.print(tbuf);
      strncpy(lastTime, tbuf, sizeof(lastTime));
      lastTime[sizeof(lastTime) - 1] = '\0';
    }

    // Next watering + status lines below the clock
    NextWaterInfo nw = computeNextWatering();
    char etaBuf[12] = "--:--";
    char zoneBuf[12] = "None";
    char remBuf[16] = "--";
    if (nw.zone >= 0) {
      snprintf(zoneBuf, sizeof(zoneBuf), "Z%d", nw.zone + 1);
      struct tm tnw;
      localtime_r(&nw.epoch, &tnw);
      strftime(etaBuf, sizeof(etaBuf), "%H:%M", &tnw);
      long diff = (long)difftime(nw.epoch, now);
      if (diff < 0) diff = 0;
      int hrs = (int)(diff / 3600L);
      int mins = (int)((diff % 3600L) / 60L);
      if (hrs > 0) {
        snprintf(remBuf, sizeof(remBuf), "in %dh%02dm", hrs, mins);
      } else {
        snprintf(remBuf, sizeof(remBuf), "in %dm", mins);
      }
    }

    const int nextY = contentY + 40; // push down away from clock
    const int nextH = 44;
    bool nextChanged =
      layoutChanged ||
      (strcmp(zoneBuf, lastNextZone) != 0) ||
      (strcmp(etaBuf, lastNextEta) != 0) ||
      (strcmp(remBuf, lastNextRem) != 0);
    if (nextChanged) {
      tft.fillRect(leftX + 4, nextY, leftW - 8, nextH, C_PANEL);
      tft.drawRect(leftX + 4, nextY, leftW - 8, nextH, C_EDGE);
      tft.setTextSize(1);
      tft.setTextColor(C_TEXT);
      tft.setCursor(leftX + 8, nextY + 2);
      tft.print("Next ");
      tft.print(zoneBuf);
      tft.setCursor(leftX + 8, nextY + 16);
      tft.print("Runs at ");
      tft.print(etaBuf);
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(leftX + 8, nextY + 30);
      tft.print(remBuf);
      strncpy(lastNextZone, zoneBuf, sizeof(lastNextZone));
      lastNextZone[sizeof(lastNextZone) - 1] = '\0';
      strncpy(lastNextEta, etaBuf, sizeof(lastNextEta));
      lastNextEta[sizeof(lastNextEta) - 1] = '\0';
      strncpy(lastNextRem, remBuf, sizeof(lastNextRem));
      lastNextRem[sizeof(lastNextRem) - 1] = '\0';
    }

    const bool showExtra = (contentH >= 112);
    int infoY = nextY + nextH + 2;
    if (showExtra) {
      String zn = (nw.zone >= 0) ? zoneNames[nw.zone] : "None";
      if (zn.length() > 12) zn = zn.substring(0, 12);
      int windT = isfinite(windNow) ? (int)lroundf(windNow * 10.0f) : 10000;
      int rainT = (int)lroundf(rain1hNow * 10.0f);
      bool extraChanged = layoutChanged ||
                          (zn != lastZoneNameShort) ||
                          (windT != lastWindTenths) ||
                          (rainT != lastRainTenths);
      if (extraChanged) {
        tft.fillRect(leftX + 4, infoY, leftW - 8, 22, C_PANEL);
        tft.setTextSize(1);
        tft.setTextColor(C_MUTED);
        tft.setCursor(leftX + 8, infoY + 2);
        tft.print("Zone ");
        tft.setTextColor(C_ACCENT);
        tft.print(zn);

        tft.setTextColor(C_MUTED);
        tft.setCursor(leftX + 8, infoY + 12);
        if (isfinite(windNow)) {
          tft.print("W ");
          tft.setTextColor(C_WARN);
          tft.print(windNow, 1);
          tft.setTextColor(C_MUTED);
          tft.print("m/s ");
        } else {
          tft.print("W -- ");
        }
        tft.print("R ");
        tft.setTextColor(C_GOOD);
        tft.print(lastRainAmount, 1);
        tft.setTextColor(C_MUTED);
        tft.print("mm");

        lastZoneNameShort = zn;
        lastWindTenths = windT;
        lastRainTenths = rainT;
      }
    }

    tft.setTextColor(delayed ? C_WARN : C_MUTED);
    const int statY = infoY + (showExtra ? 22 : 0) + 2;
    String causeLine = delayed ? rainDelayCauseText() : "";
    bool lineChanged = layoutChanged ||
                       (delayed != lastDelayLine) ||
                       (systemMasterEnabled != lastMasterLine) ||
                       (causeLine != lastCauseLine);
    if (lineChanged) {
      tft.fillRect(leftX + 4, statY, leftW - 8, 16, C_PANEL);
      tft.setTextSize(1);
      tft.setCursor(leftX + 8, statY + 2);
      if (delayed) {
        tft.print("Delay: ");
        tft.print(causeLine.c_str());
      } else {
        tft.print("Master ");
        tft.print(systemMasterEnabled ? "ON" : "OFF");
      }
      lastDelayLine = delayed;
      lastMasterLine = systemMasterEnabled;
      lastCauseLine = causeLine;
    }

    // Extra space for 240x320: show min/max + gust card under status
    const bool tallLayout = (contentH >= 170);
    if (tallLayout) {
      int extraY = statY + 18;
      int extraH = contentY + contentH - extraY - 4;
      if (extraH >= 24) {
        int minT = isnan(todayMin_C) ? 1000 : (int)lroundf(todayMin_C);
        int maxT = isnan(todayMax_C) ? 1000 : (int)lroundf(todayMax_C);
        int gustT = isnan(maxGust24h_ms) ? 1000 : (int)lroundf(maxGust24h_ms * 10.0f);

        bool extraChanged = layoutChanged || (minT != lastMinT) || (maxT != lastMaxT) || (gustT != lastGustT);
        if (extraChanged) {
          drawCard(leftX + 4, extraY, leftW - 8, extraH, C_PANEL, C_EDGE);
          tft.setTextSize(1);
          tft.setTextColor(C_MUTED);
          tft.setCursor(leftX + 8, extraY + 4);
          tft.print("Today");

          tft.setTextColor(C_TEXT);
          tft.setCursor(leftX + 8, extraY + 14);
          if (minT == 1000 || maxT == 1000) {
            tft.print("Min -- Max --");
          } else {
            tft.print("Min ");
            tft.print(minT);
            tft.print(" Max ");
            tft.print(maxT);
          }

          tft.setTextColor(C_MUTED);
          tft.setCursor(leftX + 8, extraY + 24);
          tft.print("Gust ");
          if (gustT == 1000) {
            tft.print("--");
          } else {
            tft.setTextColor(C_WARN);
            tft.print(maxGust24h_ms, 1);
            tft.setTextColor(C_MUTED);
          }
          tft.print(" m/s");

          lastMinT = minT;
          lastMaxT = maxT;
          lastGustT = gustT;
        }
      }
    }

    // Weather/tank
    int t0 = isnan(temp) ? -1000 : (int)lroundf(temp);
    char newArrow = lastTempArrow;
    if (isfinite(temp)) {
      if (isfinite(lastTempF)) {
        float d = temp - lastTempF;
        if (d > 0.1f) newArrow = '^';
        else if (d < -0.1f) newArrow = 'v';
      }
    } else {
      newArrow = '-';
    }
    bool arrowChanged = (newArrow != lastTempArrow);

    if (layoutChanged || t0 != lastTemp || hum != lastHum || pct != lastPct || arrowChanged) {
      tft.fillRect(rightX + 4, contentY + 4, rightW - 8, statsH - 8, C_PANEL);

      tft.setTextSize(2);
      tft.setTextColor(C_TEXT);
      tft.setCursor(rightX + 8, contentY + 11); // push down for better centering
      tft.print("T:");
      if (isnan(temp)) tft.print("--");
      else { tft.print(temp, 0); tft.print("C"); tft.print(newArrow); }
      tft.print("  H:");
      if (hum < 0) tft.print("--");
      else { tft.print(hum); tft.print("%"); }

      int pctClamped = constrain(pct, 0, 100);
      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, contentY + 27);
      tft.print("Tank ");
      tft.print(pctClamped);
      tft.print("%");

      // Tank bar
      int barX = rightX + 8;
      int barY = contentY + statsH - 10;
      int barW = rightW - 16;
      int barH = 6;
      tft.drawRect(barX, barY, barW, barH, C_EDGE);
      int fillW = (barW - 2) * pctClamped / 100;
      if (fillW < 0) fillW = 0;
      uint16_t barColor = (pctClamped <= (int)tankLowThresholdPct) ? C_BAD : C_GOOD;
      if (fillW > 0) tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, barColor);
      
      lastTemp = t0;
      lastTempF = isfinite(temp) ? temp : NAN;
      lastTempArrow = newArrow;
      lastHum = hum;
      lastPct = pct;
    }

    // Zones grid
    const int zonesY = contentY + statsH + gap;
    const int zonesH = contentY + contentH - zonesY;
    const int gridY = zonesY + 14;
    const int gridH = zonesY + zonesH - gridY - 6;
    const int cols = 2;
    int colW = (rightW - gap) / cols;
    int rows = (zonesCount + 1) / 2;          // two columns
    if (rows < 1) rows = 1;
    int rowH = gridH / rows;
    if (rowH < 8) rowH = 8;                   // allow more rows when >6 zones

    for (int i = 0; i < (int)zonesCount; i++) {
      int r = i / 2;
      int c = i % 2;
      int x = rightX + 6 + c * colW;
      int y = gridY + r * rowH;
      if (y + rowH > zonesY + zonesH) break;

      bool on = zoneActive[i];
      uint32_t rem = 0;
      if (on) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000UL;
        unsigned long total = zoneRunTotalSec[i];
        if (total == 0) total = durationForSlot(i,1);
        rem = (elapsed < total ? total - elapsed : 0UL);
      }

      bool stateChanged = (on != lastZoneState[i]);
      bool remChanged = (rem != lastZoneRem[i]);

      if (layoutChanged || stateChanged || remChanged) {
        // Clear full cell to prevent overlap artifacts
        tft.fillRect(x - 2, y - 1, colW - 4, rowH - 1, C_PANEL);

        // Single-line layout: Z#, pill, timer on the right
        tft.setTextSize(1);
        tft.setTextColor(C_TEXT);
        tft.setCursor(x, y + 2);
        tft.print("Z"); tft.print(i + 1);

        int pillX = x + 16;
        int pillY = y + 1;
        int pillW = 22;
        int pillH = 12;
        tft.fillRect(pillX, pillY, pillW, pillH, on ? C_GOOD : C_EDGE);
        tft.drawRect(pillX, pillY, pillW, pillH, C_EDGE);
        tft.setTextColor(on ? ST77XX_BLACK : C_TEXT);
        tft.setCursor(pillX + 4, pillY + 2);
        tft.print(on ? "ON" : "OFF");

        // Timer or placeholder at right
        char rbuf[10] = "--";
        if (on) { fmtMMSS(rbuf, sizeof(rbuf), rem); }
        tft.setTextColor(on ? C_TEXT : C_MUTED);
        int16_t bx, by; uint16_t bw, bh;
        tft.getTextBounds(rbuf, 0, 0, &bx, &by, &bw, &bh);
        // Position timer just to the right of the pill with a small gap
        int tx = pillX + pillW + 4;
        if (tx + (int)bw > x + colW - 4) tx = x + colW - 4 - (int)bw; // clamp inside cell
        tft.setCursor(tx, y + 2);
        tft.print(rbuf);

        lastZoneState[i] = on;
        lastZoneRem[i] = rem;
      }
    }

    // Bottom banner update
    if (layoutChanged || stateChanged) {
      if (delayed) {
        String cS = rainDelayCauseText();
        const char* c = cS.c_str();
        const int bh = 22;
        tft.fillRect(0, H - bh, W, bh, RGB(24, 18, 8));
        tft.drawFastHLine(0, H - bh, W, C_EDGE);
        tft.setTextSize(1);
        tft.setTextColor(C_WARN);
        tft.setCursor(6, H - bh + 6);
        tft.print("Delay:");
        tft.setTextColor(C_TEXT);
        tft.setCursor(52, H - bh + 6);
        tft.print((c && c[0]) ? c : "--");
      } else if (!layoutChanged) {
        tft.fillRect(0, H - 22, W, 22, C_BG);
      }
    }

    return;
  }
}


bool shouldStartZone(int zone) {
  if (zone < 0 || zone >= (int)MAX_ZONES) return false;

  time_t now = time(nullptr);
  struct tm* tt = localtime(&now);
  if (!tt) return false;

  const int wd = tt->tm_wday; // Sun=0..Sat=6
  const int hr = tt->tm_hour;
  const int mn = tt->tm_min;

  if (lastCheckedMinute[zone] == mn) return false;      // avoid dup triggers
  if (!days[zone][wd]) return false;                    // day not enabled

  const bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  const bool match2 = (enableStartTime2[zone] && hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) {
    lastCheckedMinute[zone] = mn;
    lastStartSlot[zone] = match2 ? 2 : 1;
    if (durationForSlot(zone, lastStartSlot[zone]) > 0) return true;
  }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed=(millis()-zoneStartMs[zone])/1000;
  unsigned long total = zoneRunTotalSec[zone];
  if (total == 0) total = durationForSlot(zone, 1);
  return (elapsed >= total);
}

void turnOnZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;

  checkWindRain();
  Serial.printf("[VALVE] Request ON Z%d rain=%d wind=%d blocked=%d\n",
                z+1, rainActive?1:0, windActive?1:0, isBlockedNow()?1:0);

  if (isBlockedNow())  { cancelStart(z, "BLOCKED", false); return; }
  if (rainActive)      { cancelStart(z, "RAIN",    true ); return; }
  if (windActive)      { pendingStart[z] = true; logEvent(z, "QUEUED", "WIND", false); return; }

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (pin < 0 || pin > 39) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping\n", z+1);
      cancelStart(z, "NO_PIN", false);
      return;
    }
  }

  bool anyOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyOn = true; break; }
  }
  if (!runZonesConcurrent && anyOn) {
    pendingStart[z] = true;
    logEvent(z, "QUEUED", "ACTIVE RUN", false);
    return;
  }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  zoneStartedManual[z] = false;
  unsigned long total = durationForSlot(z, lastStartSlot[z]);
  if (total == 0) total = durationForSlot(z, 1);
  zoneRunTotalSec[z] = total;
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  if (!usePcf) {
  // Use config for active level
  gpioZoneWrite(z, true);   // ON

  setWaterSourceRelays(mainsOn, tankOn);
} else {
    // PCF8574 path unchanged (still active-LOW via expander)
    pcfOut.digitalWrite(PCH[z], LOW); // active-LOW
    setWaterSourceRelays(mainsOn, tankOn);
  }

  logEvent(z, "START", src, false);

  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(2, 0);
    display.print(zoneNames[z]);
    display.print(" ON");
    display.display();
    delay(350);
  }

  // Force a clean redraw immediately for TFT (no ON splash delay).
  if (displayUseTft) {
    g_forceHomeReset = true;
    g_forceRunReset = true;
    tft.fillScreen(C_BG);
  }
  HomeScreen();
}

void turnOffZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  Serial.printf("[VALVE] Request OFF Z%d\n", z+1);
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  bool wasDelayed = rainActive || windActive || isPausedNow() ||
                    !systemMasterEnabled ||
                    (rainCooldownUntilEpoch > time(nullptr));
  if (!zoneStartedManual[z]) {
    logEvent(z, "STOPPED", src, wasDelayed);
  }

  const bool usePcf = useExpanderForZone(z);

  if (!usePcf) {
  // OFF everything in fallback according to config
  gpioZoneWrite(z, false);       // OFF
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;
  zoneStartedManual[z] = false;
  zoneRunTotalSec[z] = 0;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyStillOn = true; break; }
  }
  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);
  }

  if (!displayUseTft) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print(zoneNames[z]);
    display.print(" OFF");
    display.display();
    delay(350);
  }
  if (displayUseTft) {
    g_forceHomeReset = true;
    g_forceRunReset = true;
  }

  // Ensure the display returns to Home after showing the OFF banner
  HomeScreen();
}


void turnOnValveManual(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  if (zoneActive[z])   return;

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (pin < 0 || pin > 39) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping manual start\n", z+1);
      cancelStart(z, "NO_PIN", false);
      showManualSelection();
      return;
    }
  }

  // Respect run mode overlap rules
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      if (!runZonesConcurrent) {
        manualSelectedZone = i;
        Serial.printf("[VALVE] Manual start blocked: Z%d is already running\n", i + 1);
        showManualSelection();
        return;
      }
      break; // concurrent: allow additional zone
    }
  }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  zoneStartedManual[z] = true;
  lastStartSlot[z] = 1;
  zoneRunTotalSec[z] = durationForSlot(z,1);
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  if (!usePcf) {
  gpioZoneWrite(z, true);   // ON

  setWaterSourceRelays(mainsOn, tankOn);
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], LOW);
    setWaterSourceRelays(mainsOn, tankOn);
  }

  // Manual starts are not logged
  manualScreenUntilMs = 0;  // leave zone-select overlay immediately
  g_forceManualReset = true;
  g_forceRunReset = true;
  lastScreenRefresh = 0;
  if (displayUseTft) {
    tft.fillScreen(C_BG);
    updateLCDForZone(z);
  } else {
    HomeScreen();
  }
}

void turnOffValveManual(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  if (!zoneActive[z]) return;

  const bool usePcf = useExpanderForZone(z);
  const char* src="None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  // Turn this zone OFF
  if (!usePcf) {
    // Use configured polarity for GPIO fallback
    gpioZoneWrite(z, false);          // OFF
  } else {
    // PCF8574 path: keep active-LOW semantics (HIGH = OFF)
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;
  zoneStartedManual[z] = false;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      anyStillOn = true;
      break;
    }
  }

  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);   // mains OFF, tank OFF
  }

  // Manual OFF path should also force a clean visual transition.
  const bool manualOverlayActive =
    (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - millis()) > 0);

  if (displayUseTft) {
    tft.fillScreen(C_BG);
    g_forceRunReset = true;
    if (!manualOverlayActive) g_forceHomeReset = true;
  } else {
    display.clearDisplay();
    display.display();
  }

  lastScreenRefresh = 0; // repaint immediately on next loop cycle
  if (manualOverlayActive) drawManualSelection();
  else HomeScreen();
}

// ---------- Next Water (queue-first) ----------
static NextWaterInfo computeNextWatering() {
  NextWaterInfo best{0, -1, 0};

  for (int z=0; z<(int)zonesCount; ++z) {
    if (pendingStart[z]) {
      best.epoch = time(nullptr);
      best.zone  = z;
      best.durSec = (uint32_t)durationForSlot(z, lastStartSlot[z]);
      return best;
    }
  }

  time_t now = time(nullptr);
  struct tm base; localtime_r(&now, &base);

  auto consider = [&](int z, int hr, int mn, bool enabled, int slot) {
    if (!enabled) return;
    unsigned long tot = durationForSlot(z, slot);
    if (tot == 0) return;

    struct tm cand = base;
    cand.tm_sec  = 0;
    cand.tm_hour = hr;
    cand.tm_min  = mn;

    time_t t = mktime(&cand);
    if (t <= now) t += 24*60*60;

    for (int k = 0; k < 8; ++k) {
      struct tm tmp; localtime_r(&t, &tmp);
      int wd = tmp.tm_wday;
      if (days[z][wd]) {
        if (best.zone < 0 || t < best.epoch) {
          best.epoch = t;
          best.zone  = z;
          best.durSec = (uint32_t)tot;
        }
        return;
      }
      t += 24*60*60;
    }
  };

  for (int z=0; z<(int)zonesCount; ++z) {
    consider(z, startHour[z],  startMin[z],  true, 1);
    consider(z, startHour2[z], startMin2[z], enableStartTime2[z], 2);
  }
  return best;
}

// Main Page
void handleRoot() {
  HttpScope _scope;  // NEW: mark that we're in an HTTP handler so no blocking fetches

  // --- Precompute state / snapshots ---
  checkWindRain();

  time_t now = time(nullptr);
  struct tm* ti = localtime(&now);
  char timeStr[9], dateStr[11];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", ti);
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", ti);

  // Keep this - it respects the g_inHttp guard
  updateCachedWeather();

  // Safe reads from decoded snapshot
  float temp = curTempC;
  float hum = (curHumidityPct >= 0) ? (float)curHumidityPct : NAN;
  float ws = curWindMs;
  float feels = curFeelsC;
  String windDir = formatWindDirection(curWindDirDeg);
  int wcode = curWeatherCode;
  String cond = (wcode >= 0) ? String(meteoCodeToDesc(wcode)) : String("-");
  if (cond == "") cond = "-";
  String cityName = meteoLocationLabel();
  if (cityName == "") cityName = "-";

  const int    tankPct   = tankPercent();
  const String causeText = rainDelayCauseText();
  const bool   pausedNow = isPausedNow();
  int activeZoneCount = 0;
  for (int i = 0; i < (int)zonesCount; ++i) {
    if (zoneActive[i]) activeZoneCount++;
  }
  NextWaterInfo nextWater = computeNextWatering();
  char nextWaterTime[6] = "--:--";
  char nextWaterDay[20] = "--";
  String nextWaterLabel = String("--");
  String nextWaterSub = rainActive ? String("Waiting for rain delay to clear") : String("No queued run");
  if (nextWater.epoch > 0 && nextWater.zone >= 0 && nextWater.zone < (int)zonesCount) {
    struct tm nextTm;
    localtime_r(&nextWater.epoch, &nextTm);
    strftime(nextWaterTime, sizeof(nextWaterTime), "%H:%M", &nextTm);
    if (ti) {
      struct tm todayTm = *ti;
      todayTm.tm_hour = 0; todayTm.tm_min = 0; todayTm.tm_sec = 0;
      struct tm nextDayTm = nextTm;
      nextDayTm.tm_hour = 0; nextDayTm.tm_min = 0; nextDayTm.tm_sec = 0;
      time_t todayEpoch = mktime(&todayTm);
      time_t targetEpoch = mktime(&nextDayTm);
      long dayDiff = (todayEpoch > 0 && targetEpoch > 0) ? (long)((targetEpoch - todayEpoch) / 86400L) : 9999L;
      if (dayDiff == 0) {
        strncpy(nextWaterDay, "Today", sizeof(nextWaterDay));
        nextWaterDay[sizeof(nextWaterDay) - 1] = '\0';
      } else if (dayDiff == 1) {
        strncpy(nextWaterDay, "Tomorrow", sizeof(nextWaterDay));
        nextWaterDay[sizeof(nextWaterDay) - 1] = '\0';
      } else {
        strftime(nextWaterDay, sizeof(nextWaterDay), "%a %d %b", &nextTm);
      }
    } else {
      strftime(nextWaterDay, sizeof(nextWaterDay), "%a %d %b", &nextTm);
    }
    nextWaterLabel = String(nextWaterDay) + String(" ") + String(nextWaterTime);
    nextWaterSub = zoneNames[nextWater.zone];
    if (nextWater.durSec > 0) {
      char durBuf[16];
      fmtMMSS(durBuf, sizeof(durBuf), nextWater.durSec);
      nextWaterSub += " - ";
      nextWaterSub += durBuf;
    }
  }
  String heroSystemValue = pausedNow ? String("Paused")
                          : (systemMasterEnabled ? String("Master On") : String("Master Off"));
  String heroSystemSub = pausedNow ? String("Schedules are temporarily suspended")
                        : (systemMasterEnabled
                           ? (activeZoneCount > 0
                              ? String(activeZoneCount) + String(activeZoneCount == 1 ? " zone running" : " zones running")
                              : String("Automation ready"))
                           : String("Outputs are blocked"));
  String heroWeatherValue = isnan(temp) ? String("--") : String(temp, 1) + " C";

  // --- HTML ---
  String html; html.reserve(6000);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  auto flush = [&](){
    if (html.length()) {
      server.sendContent(html);
      html = "";
    }
  };
  html += F("<!doctype html><html lang='en' data-theme='light'><head>");
  html += F("<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#145b63'><meta name='color-scheme' content='light dark'>");
  html += F("<title>ESP32 Irrigation</title>");
  html += F("<style>");
  html += F("@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@500;700&family=Sora:wght@400;600;700;800&display=swap');");
  html += F(".center{max-width:1280px;margin:0 auto}");
  html += F(":root[data-theme='light']{--bg:#edf3ef;--bg2:#f8fbf8;--glass:rgba(255,255,255,.58);--glass-brd:rgba(122,149,140,.32);--panel:#ffffff;--line:#d3dfd9;");
  html += F("--card:#ffffff;--ink:#14232b;--muted:#5f736f;--primary:#1f8a70;--primary-2:#145b63;--ok:#2f9e44;--warn:#c97a1a;--bad:#d9485f;");
  html += F("--chip:#edf7f2;--chip-brd:#c9ddd4;--ring:#dcebe5;--ring2:#b4d4cb;--shadow:0 18px 40px rgba(20,47,45,.14)}");
  html += F(":root[data-theme='dark']{--bg:#081315;--bg2:#0d1d21;--glass:rgba(12,28,31,.62);--glass-brd:rgba(84,123,118,.3);--panel:#0f1e22;--line:#214147;");
  html += F("--card:#0f1e22;--ink:#e6f0ec;--muted:#9ab4ad;--primary:#46c6a3;--primary-2:#1f8a86;--ok:#52c266;--warn:#f0ae4d;--bad:#ff6b7d;");
  html += F("--chip:#10272b;--chip-brd:#24444a;--ring:#123036;--ring2:#1d5152;--shadow:0 18px 40px rgba(0,0,0,.42)}");
  html += F("*{box-sizing:border-box}html{scroll-behavior:smooth}html,body{margin:0;padding:0;");
  html += F("background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),");
  html += F("radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),");
  html += F("radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--ink);font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;line-height:1.35}");
  html += F(":root{--gap:16px;--pad:18px;--pad-lg:22px;--radius:20px;--radius-sm:16px;}");
  html += F("a{text-decoration:none;color:inherit}");

  // Top nav - made a bit tighter on mobiles
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 12px;");
  html += F("background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);");
  html += F("box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1280px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff;flex-wrap:wrap}");
  html += F(".brand{display:flex;align-items:center;gap:8px;font-weight:800;letter-spacing:.2px;font-size:1.12rem}");
  html += F(".brand-copy{display:flex;flex-direction:column;gap:2px}");
  html += F(".brand-title{text-transform:uppercase;letter-spacing:.9px;font-size:1rem;line-height:1}");
  html += F(".brand-sub{font-size:.74rem;font-weight:650;color:rgba(255,255,255,.8);letter-spacing:.08em;text-transform:uppercase}");
  html += F(".dot{width:12px;height:12px;border-radius:999px;background:#84ffb5;box-shadow:0 0 14px #84ffb5}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650;font-size:.88rem}");
  html += F(".pill{background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:7px 12px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;");
  html += F("border-radius:10px;padding:8px 14px;font-weight:700;cursor:pointer;font-size:.92rem}");
  html += F(".hero-shell{position:relative;overflow:hidden;padding:clamp(18px,3vw,30px);margin:18px 0 18px}");
  html += F(".hero-shell::before,.hero-shell::after{content:'';position:absolute;border-radius:999px;pointer-events:none;filter:blur(8px)}");
  html += F(".hero-shell::before{width:220px;height:220px;right:-50px;top:-40px;background:radial-gradient(circle,rgba(233,173,73,.2),transparent 68%)}");
  html += F(".hero-shell::after{width:260px;height:260px;left:-90px;bottom:-100px;background:radial-gradient(circle,rgba(70,198,163,.2),transparent 68%)}");
  html += F(".hero-grid{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(320px,.85fr);gap:18px;align-items:stretch}");
  html += F(".hero-copy,.hero-mini-grid{position:relative;z-index:1}");
  html += F(".hero-copy{display:flex;flex-direction:column;justify-content:center;gap:14px}");
  html += F(".hero-kicker{text-transform:uppercase;letter-spacing:.22em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".hero-title{margin:0;font-size:clamp(1.95rem,4vw,3.2rem);line-height:1.02;max-width:11ch}");
  html += F(".hero-text{margin:0;max-width:60ch;color:var(--muted);font-size:1rem}");
  html += F(".hero-actions{display:flex;gap:10px;flex-wrap:wrap}");
  html += F(".dash-nav{display:flex;gap:10px;flex-wrap:wrap;align-items:center;padding:10px 12px;margin:-2px 0 18px;position:sticky;top:74px;z-index:9}");
  html += F(".dash-nav a{display:inline-flex;align-items:center;justify-content:center;padding:9px 14px;border-radius:999px;border:1px solid var(--chip-brd);background:rgba(255,255,255,.34);font-size:.84rem;font-weight:800;letter-spacing:.04em;text-transform:uppercase;color:var(--ink);transition:transform .08s ease,background .12s ease,border-color .12s ease,box-shadow .12s ease}");
  html += F(".dash-nav a:hover{transform:translateY(-1px);background:rgba(31,138,112,.12);border-color:rgba(31,138,112,.28);box-shadow:0 10px 22px rgba(20,91,99,.12)}");
  html += F(".hero-mini-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}");
  html += F(".hero-mini{min-height:132px;padding:16px;border-radius:18px;border:1px solid var(--glass-brd);");
  html += F("background:linear-gradient(180deg,rgba(255,255,255,.58),rgba(255,255,255,.14));box-shadow:0 14px 30px rgba(19,33,68,.12);");
  html += F("display:flex;flex-direction:column;justify-content:space-between;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}");
  html += F(".hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(31,138,112,.2),rgba(82,194,102,.1))}");
  html += F(".hero-mini-label{text-transform:uppercase;letter-spacing:.16em;font-size:.74rem;font-weight:800;color:var(--muted)}");
  html += F(".hero-mini-value{font-size:1.72rem;font-weight:800;line-height:1.02;font-variant-numeric:tabular-nums}");
  html += F(".hero-mini-value.weather-value{display:inline-flex;align-items:center;gap:8px}");
  html += F(".hero-mini-sub{color:var(--muted);font-size:.92rem}");
  html += F(".section-head{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;max-width:1280px;margin:0 auto 14px;padding:0 2px}");
  html += F(".section-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".section-head h2{margin:4px 0 0;font-size:1.35rem;line-height:1.08}");
  html += F(".section-note{margin:0;max-width:42ch;color:var(--muted);font-size:.92rem;text-align:right}");

  // Cards and grids
  html += F(".wrap{max-width:1280px;margin:20px auto;padding:0 16px}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow)}");
  html += F(".section{padding:var(--pad)}");
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:var(--gap)}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad)}");
  html += F(".card h3{margin:0 0 10px 0;font-size:1.15rem;color:var(--ink);font-weight:850;letter-spacing:.35px;");
  html += F("padding-bottom:6px;border-bottom:2px solid var(--primary);display:flex;align-items:center;gap:8px}");
  html += F(".card h4{margin:6px 0 6px 0;font-size:.95rem;color:var(--muted);font-weight:700;letter-spacing:.2px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:8px 14px;font-weight:650;white-space:nowrap;font-size:.95rem;color:var(--ink)}");
  html += F(".big{font-weight:800;font-size:1.3rem;font-variant-numeric:tabular-nums}.hint{color:var(--muted);font-size:.9rem;margin-top:4px}.sub{color:var(--muted);font-size:.88rem}");
  html += F(".meter{position:relative;height:18px;border-radius:999px;background:linear-gradient(180deg,rgba(0,0,0,.12),transparent);border:1px solid var(--glass-brd);overflow:hidden;margin-top:6px}");
  html += F(".fill{position:absolute;inset:0 0 0 0;width:0%;height:100%;background:linear-gradient(90deg,#56d2c0,#7ecb71,#1f8a70);");
  html += F("box-shadow:0 0 30px rgba(86,210,192,.28) inset;transition:width .4s ease}");
  html += F(".badge{display:inline-flex;align-items:center;gap:6px;padding:8px 13px;border-radius:999px;border:1px solid var(--glass-brd);font-size:.9rem}");
  html += F(".b-ok{background:rgba(34,197,94,.12);border-color:rgba(34,197,94,.3)}");
  html += F(".b-warn{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.35)}");
  html += F(".b-bad{background:rgba(239,68,68,.12);border-color:rgba(239,68,68,.38)}");
  html += F(".summary-grid{align-items:stretch}");
  html += F(".summary-card{display:flex;flex-direction:column;gap:14px;min-height:178px;background:linear-gradient(180deg,var(--card),rgba(255,255,255,.42))}");
  html += F(".summary-card h3{margin-bottom:0}");
  html += F(".summary-link{display:flex;flex-direction:column;justify-content:center;gap:8px;min-height:120px;padding:16px;border-radius:18px;");
  html += F("background:linear-gradient(135deg,var(--chip),rgba(255,255,255,.72));border:1px solid var(--chip-brd)}");
  html += F(".summary-k{font-size:.75rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".summary-value{font-size:clamp(1.45rem,2.5vw,2.15rem);line-height:1.02;font-weight:800;letter-spacing:-.03em;font-variant-numeric:tabular-nums}");
  html += F(".summary-support{color:var(--muted);font-size:.92rem}");
  html += F(".summary-row{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;flex-wrap:wrap}");
  html += F(".summary-meta{display:flex;gap:8px;flex-wrap:wrap}");
  html += F(".mini-chip{display:inline-flex;align-items:center;gap:6px;padding:7px 11px;border-radius:999px;border:1px solid var(--chip-brd);background:var(--chip);font-size:.82rem;font-weight:700;color:var(--ink)}");
  html += F(".summary-metric-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px}");
  html += F(".summary-metric-grid.metric-pair{grid-template-columns:repeat(2,minmax(0,1fr))}");
  html += F(".summary-subhead{font-size:.76rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800;margin-top:2px}");
  html += F(".metric-tile{display:flex;flex-direction:column;justify-content:space-between;gap:8px;min-height:92px;padding:12px 13px;border-radius:16px;");
  html += F("border:1px solid var(--chip-brd);background:linear-gradient(180deg,rgba(255,255,255,.55),rgba(255,255,255,.15))}");
  html += F(".metric-tile.metric-wide{grid-column:span 2}");
  html += F(".metric-split{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}");
  html += F(".metric-split-item{display:flex;align-items:flex-end;justify-content:center;min-height:44px}");
  html += F(".metric-k{font-size:.74rem;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".metric-v{font-size:1.08rem;font-weight:800;color:var(--ink)}");
  html += F(".metric-v.big-metric{font-size:1.45rem;line-height:1.05;font-variant-numeric:tabular-nums}");
  html += F(".metric-v .metric-unit{font-size:.88rem;font-weight:700;color:var(--muted);margin-left:4px}");
  html += F(".summary-note{padding:12px 14px;border-radius:16px;border:1px solid rgba(31,138,112,.2);background:linear-gradient(180deg,rgba(31,138,112,.1),rgba(31,138,112,.03));color:var(--muted);font-size:.9rem}");
  html += F(".toolbar{display:flex;gap:var(--gap);flex-wrap:wrap;margin:14px 0 0}");
  html += F(".section-block,.summary-shell,.sched-shell,.zones-shell,.controls-shell{scroll-margin-top:140px}");
  html += F(".btn{background:linear-gradient(180deg,var(--primary),var(--primary-2));color:#fff;border:1px solid rgba(0,0,0,.08);border-radius:13px;padding:11px 16px;font-weight:800;cursor:pointer;");
  html += F("box-shadow:0 8px 20px rgba(0,0,0,.22);font-size:1.02rem}");
  html += F(".btn-secondary{background:transparent;color:var(--ink);border:1px solid var(--line);box-shadow:none}");
  html += F(".btn:disabled{background:#7f8aa1;cursor:not-allowed;box-shadow:none}.btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border-color:rgba(185,28,28,.6)}");
  html += F(".btn,.btn-ghost,.pill{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn,.btn-ghost,.pill{touch-action:manipulation}");
  html += F(".btn:active:not(:disabled),.btn-ghost:active,.pill:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-ghost,.pill{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".zone-overview{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:var(--gap);margin-bottom:18px}");
  html += F(".zone-summary-card{padding:18px;border:1px solid var(--line);border-radius:18px;background:linear-gradient(160deg,var(--panel),rgba(70,198,163,.08));display:flex;flex-direction:column;gap:8px;min-height:136px}");
  html += F(".zone-summary-k{font-size:.74rem;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:800}");
  html += F(".zone-summary-v{font-size:1.3rem;line-height:1.06;font-weight:850;color:var(--ink)}");
  html += F(".zone-summary-sub{font-size:.95rem;color:var(--ink);font-weight:650}");
  html += F(".zone-summary-meta{margin-top:auto;font-size:.84rem;color:var(--muted)}");
  html += F(".zone-list-head{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;flex-wrap:wrap;margin-bottom:10px}");
  html += F(".zone-list-head h3{margin:0}");
  html += F(".zone-list-note{font-size:.9rem;color:var(--muted)}");
  html += F(".zone-list{display:flex;flex-direction:column;gap:10px}");
  html += F(".zone-row{display:grid;grid-template-columns:minmax(0,1.6fr) auto auto;gap:12px;align-items:center;padding:14px 16px;border:1px solid var(--line);border-radius:18px;background:linear-gradient(180deg,var(--panel),rgba(255,255,255,.04));transition:border-color .14s ease,box-shadow .14s ease,transform .14s ease}");
  html += F(".zone-row.is-active{border-color:rgba(34,197,94,.42);box-shadow:0 14px 30px rgba(22,163,74,.12)}");
  html += F(".zone-row-main{display:flex;align-items:center;gap:10px;min-width:0}");
  html += F(".zone-row-copy{min-width:0}");
  html += F(".zone-row-name{font-size:1rem;font-weight:780;color:var(--ink);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}");
  html += F(".zone-row-sub{font-size:.84rem;color:var(--muted);margin-top:2px}");
  html += F(".zone-index{width:26px;height:26px;border-radius:999px;display:flex;align-items:center;justify-content:center;");
  html += F("font-size:.82rem;background:var(--chip);border:1px solid var(--chip-brd);flex-shrink:0}");
  html += F(".zone-dot{width:13px;height:13px;border-radius:999px;background:var(--line);border:1px solid var(--glass-brd);box-shadow:0 0 0 2px rgba(0,0,0,.06) inset}");
  html += F(".zone-dot.on{background:var(--ok);box-shadow:0 0 10px rgba(34,197,94,.55)}");
  html += F(".zone-row-status{display:flex;align-items:center;justify-content:flex-end;gap:10px;flex-wrap:wrap}");
  html += F(".zone-row-rem{font-size:.96rem;font-weight:760;color:var(--ink);font-variant-numeric:tabular-nums;min-width:86px;text-align:right}");
  html += F(".zone-row-actions{margin:0;display:flex;gap:8px;justify-content:flex-end;flex-wrap:wrap}");
  html += F(".zone-row-actions .btn{padding:9px 14px;font-size:.92rem}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 16px 34px rgba(0,0,0,.18)}");
  html += F(".btn:hover{filter:brightness(1.05)}");
  html += F(".chip:hover,.pill:hover{filter:brightness(1.02)}");
  html += F(".btn:focus-visible,.pill:focus-visible,.chip:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");

  // Schedules styles (collapsible, mobile-friendly)
  html += F(".sched{margin-top:var(--gap)}");
  html += F(".sched-shell{padding:0;overflow:hidden}");
  html += F(".sched-top{display:flex;align-items:flex-end;justify-content:space-between;gap:16px;flex-wrap:wrap;padding:18px 18px 14px;border-bottom:1px solid var(--line);background:linear-gradient(180deg,rgba(255,255,255,.05),transparent)}");
  html += F(".sched-top-copy{display:flex;flex-direction:column;gap:6px;max-width:58ch}");
  html += F(".sched-top-copy h3{margin:0;font-size:1.2rem}");
  html += F(".sched-top-copy p{margin:0;color:var(--muted);font-size:.92rem}");
  html += F(".sched-tools{display:flex;gap:10px;flex-wrap:wrap;align-items:center}");
  html += F(".sched-ctr{--schedW:360px;--gap:var(--gap);max-width:100%;margin:0 auto}");
  html += F(".sched-body{padding:18px}");
  html += F(".sched-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:var(--gap)}");
  html += F(".sched-card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius-sm);padding:var(--pad)}");
  html += F(".sched-card h4{margin:0 0 10px 0;font-size:1.05rem;font-weight:800;color:var(--ink);display:flex;align-items:center;gap:8px}");
  html += F(".sched-badge{background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:4px 9px;font-size:.8rem;font-weight:700;color:var(--muted)}");
  html += F(".rowx{display:grid;grid-template-columns:110px 1fr;gap:10px;align-items:center;margin:10px 0}");
  html += F(".rowx label{min-width:0;font-size:.82rem;color:var(--muted);font-weight:700;letter-spacing:.4px;text-transform:uppercase}");
  html += F(".field{display:flex;align-items:center;gap:8px;flex-wrap:wrap}");
  html += F(".field.inline .in{width:72px}");
  html += F(".field .sep{color:var(--muted);font-weight:700}");
  html += F(".field .unit{color:var(--muted);font-size:.85rem}");
  html += F(".toggle-inline{display:inline-flex;align-items:center;gap:6px;font-size:.85rem;color:var(--muted);padding:7px 11px;border-radius:999px;border:1px solid var(--chip-brd);background:transparent}");
  html += F(".sched-card input[type=checkbox]{appearance:none;-webkit-appearance:none;width:18px;height:18px;margin:0;flex:0 0 18px;display:inline-grid;place-content:center;cursor:pointer;");
  html += F("border:1.6px solid rgba(230,240,236,.78);border-radius:6px;background:transparent;box-shadow:none;transition:border-color .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F(".sched-card input[type=checkbox]::before{content:'';display:block;width:5px;height:9px;border-right:2px solid #fff;border-bottom:2px solid #fff;transform:rotate(45deg) scale(0);margin-top:-1px;transition:transform .12s ease}");
  html += F(".sched-card input[type=checkbox]:checked{border-color:#ffffff;background:transparent;box-shadow:none}");
  html += F(".sched-card input[type=checkbox]:checked::before{transform:rotate(45deg) scale(1)}");
  html += F(".sched-card input[type=checkbox]:focus-visible{outline:none;border-color:#ffffff;box-shadow:0 0 0 3px rgba(31,138,112,.18)}");
  html += F(".sched-card input[type=checkbox]:active{transform:translateY(1px)}");
  html += F(".in{border:1px solid var(--line);border-radius:10px;padding:8px 10px;background:transparent;color:var(--ink);font-size:.9rem}");
  html += F(".days-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;width:100%}");
  html += F(".day{position:relative;display:block;min-width:0}");
  html += F(".day input{position:absolute;inset:0;width:100%;height:100%;margin:0;opacity:0;cursor:pointer;z-index:2}");
  html += F(".day span{display:flex;align-items:center;justify-content:center;min-height:44px;padding:10px 16px;border-radius:999px;border:1px solid var(--chip-brd);");
  html += F("background:linear-gradient(180deg,rgba(255,255,255,.06),rgba(255,255,255,.02));font-size:.78rem;font-weight:800;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);transition:border-color .12s ease,background .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F(".day input:checked + span{border-color:rgba(31,138,112,.46);background:linear-gradient(180deg,rgba(31,138,112,.18),rgba(31,138,112,.05));color:var(--ink);box-shadow:0 10px 20px rgba(20,91,99,.12)}");
  html += F(".day input:focus-visible + span{outline:2px solid var(--primary);outline-offset:2px}");
  html += F(".day input:active + span{transform:translateY(1px)}");
  html += F("@media(max-width:720px){.nav .in{flex-direction:column;align-items:flex-start}.zones{grid-template-columns:1fr}.sched-grid{grid-template-columns:1fr}.rowx{grid-template-columns:1fr}.rowx label{margin-bottom:4px}.days-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.day span{min-height:42px}}");
  html += F("@media(max-width:460px){.days-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}");
  html += F(".collapse{cursor:pointer;user-select:none;display:flex;align-items:center;justify-content:space-between;font-size:1.05rem}");
  html += F(".collapse .arr{font-size:1rem;opacity:.8;margin-left:6px}");
  html += F(".sched-title{display:flex;flex-direction:column;gap:2px}");
  html += F(".sched-sub{font-size:.88rem;color:var(--muted);font-weight:600}");

  // Desktop enhancements
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{margin:20px auto;padding:0 20px;}");
  html += F(".glass.section{padding:var(--pad-lg);}");
  html += F(".summary-grid{grid-template-columns:repeat(4,minmax(0,1fr));}");
  html += F(".zones{grid-template-columns:repeat(3,minmax(300px,1fr));}");
  html += F(".card{padding:var(--pad-lg);}");
  html += F(".card h3{font-size:1.12rem;}");
  html += F("}");
  html += F("html,body{font-family:'Sora','Avenir Next','Trebuchet MS',sans-serif}");
  html += F("body{position:relative;min-height:100vh}");
  html += F("body::before,body::after{content:'';position:fixed;z-index:-1;pointer-events:none;filter:blur(28px);opacity:.42}");
  html += F("body::before{width:340px;height:340px;right:-120px;top:56px;background:radial-gradient(circle,#1f8a86,transparent 68%);animation:floatbg 14s ease-in-out infinite}");
  html += F("body::after{width:290px;height:290px;left:-110px;bottom:36px;background:radial-gradient(circle,#52c266,transparent 66%);animation:floatbg 16s ease-in-out infinite reverse}");
  html += F("@keyframes floatbg{0%,100%{transform:translateY(0)}50%{transform:translateY(-14px)}}");
  html += F(".nav{border-bottom:1px solid rgba(255,255,255,.2);backdrop-filter:blur(7px);-webkit-backdrop-filter:blur(7px)}");
  html += F(".pill,#btn-master{font-weight:700}");
  html += F("#clock{font-family:'JetBrains Mono','Consolas',monospace;font-size:.84rem}");
  html += F(".card{position:relative;overflow:hidden}");
  html += F(".card::before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,var(--primary),#52c266);opacity:.85}");
  html += F(".summary-grid .card{animation:rise .45s ease both}");
  html += F(".summary-grid .card:nth-child(2){animation-delay:.05s}.summary-grid .card:nth-child(3){animation-delay:.1s}.summary-grid .card:nth-child(4){animation-delay:.15s}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}");
  html += F(".btn{font-weight:760;letter-spacing:.15px}");
  html += F(".chip b,.badge b{font-weight:800}");
  html += F("html[data-theme='dark'] .dash-nav a{background:transparent;border-color:var(--chip-brd)}");
  html += F("html[data-theme='dark'] .hero-mini{background:linear-gradient(180deg,rgba(15,30,34,.96),rgba(15,30,34,.9));box-shadow:0 14px 30px rgba(0,0,0,.22)}");
  html += F("html[data-theme='dark'] .hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(31,138,112,.18),rgba(16,39,43,.94))}");
  html += F("html[data-theme='dark'] .summary-card{background:var(--card)}");
  html += F("html[data-theme='dark'] .summary-link{background:var(--chip)}");
  html += F("html[data-theme='dark'] .metric-tile{background:#102327}");
  html += F("html[data-theme='dark'] .sched-top{background:transparent}");
  html += F("html[data-theme='dark'] .action-card{background:var(--card)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]{border-color:#8aa59c;background:rgba(255,255,255,.78);box-shadow:inset 0 1px 0 rgba(255,255,255,.75)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]::before{border-right-color:#17666b;border-bottom-color:#17666b}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]:checked{border-color:#1f8a70;background:#ffffff;box-shadow:0 0 0 3px rgba(31,138,112,.12)}");
  html += F("html[data-theme='light'] .sched-card input[type=checkbox]:focus-visible{border-color:#1f8a70;box-shadow:0 0 0 3px rgba(31,138,112,.14)}");
  html += F(".summary-grid .weather-card,.summary-grid .next-card{grid-column:span 2}");
  html += F(".action-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:var(--gap);margin:12px auto 24px}");
  html += F(".action-card{min-height:180px;display:flex;flex-direction:column;justify-content:space-between;background:linear-gradient(180deg,var(--card),rgba(255,255,255,.32))}");
  html += F(".action-copy p{margin:8px 0 0;color:var(--muted)}");
  html += F("#clock,.hero-mini-value{font-variant-numeric:tabular-nums}");
  html += F("@media(max-width:980px){.hero-grid{grid-template-columns:1fr}.section-head{flex-direction:column;align-items:flex-start}.section-note{text-align:left}.summary-grid .weather-card,.summary-grid .next-card{grid-column:auto}.metric-tile.metric-wide{grid-column:auto}.dash-nav{top:68px}}");
  html += F("@media(max-width:720px){.nav{padding-top:8px}.chip{font-size:.88rem}.hero-shell{padding:18px}.hero-title{max-width:none;font-size:1.95rem}.hero-mini-grid{grid-template-columns:1fr}.zone-row{grid-template-columns:1fr}.zone-row-status,.zone-row-actions{justify-content:flex-start}.zone-row-actions .btn{flex:1 1 140px}.brand-title{letter-spacing:.55px}.summary-card{min-height:auto}.summary-link{min-height:0}.summary-metric-grid{grid-template-columns:1fr 1fr}.summary-metric-grid.metric-pair{grid-template-columns:1fr}.dash-nav{top:120px;overflow:auto;flex-wrap:nowrap;padding-bottom:8px}.dash-nav a{white-space:nowrap}.sched-top{padding:16px}.sched-body{padding:16px}.sched-tools .btn{flex:1 1 140px}}");
  html += F("</style></head><body>");
  flush();

  // --- Nav ---
  html += F("<div class='nav'><div class='in'>");
  html += F("<div class='brand'><span class='dot'></span><div class='brand-copy'><span class='brand-title'>ESP32 Irrigation</span><span class='brand-sub'>Weather-aware watering control</span></div></div>");
  html += F("<div class='meta'>");
  html += F("<span class='pill' id='clock'>"); html += timeStr; html += F("</span>");
  html += F("<span class='pill'>"); html += dateStr; html += F("</span>");
  html += F("<span class='pill'>"); html += ((ti && ti->tm_isdst>0) ? "DST" : "STD"); html += F("</span>");
  html += F("<span class='pill' title='IP ");
  html += WiFi.localIP().toString();
  html += F("'>espirrigation.local</span>");
  html += F("<button id='btn-master' class='pill' style='cursor:pointer;border:none' aria-pressed='");
  html += (systemMasterEnabled ? "true" : "false");
  html += F("' title='Toggle master enable/disable'>Master: <b id='master-state'>");
  html += (systemMasterEnabled ? "On" : "Off");
  html += F("</b></button>");
  html += F("<button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button>");
  html += F("</div></div></div>");

  // --- Hero ---
  html += F("<div class='wrap'><div class='hero-shell glass'><div class='hero-grid'>");
  html += F("<div class='hero-copy'><div class='hero-kicker'>Smart dashboard</div>");
  html += F("<h1 class='hero-title'>ESP32 Irrigation.</h1>");
  html += F("<p class='hero-text'></p>");
  html += F("<div class='hero-actions'><a class='btn' href='/setup'>Setup</a><a class='btn btn-secondary' href='/events'>Events</a><a class='btn btn-secondary' href='/status'>Status JSON</a></div></div>");
  html += F("<div class='hero-mini-grid'>");
  html += F("<div class='hero-mini hero-mini-strong'><span class='hero-mini-label'>System</span><span class='hero-mini-value' id='heroMasterState'>");
  html += heroSystemValue;
  html += F("</span><span class='hero-mini-sub' id='heroMasterSub'>");
  html += heroSystemSub;
  html += F("</span></div>");
  html += F("<div class='hero-mini'><span class='hero-mini-label'>Tank Reserve</span><span class='hero-mini-value' id='heroTankValue'>");
  html += String(tankPct);
  html += F("%</span><span class='hero-mini-sub' id='heroTankSub'>");
  html += sourceModeText();
  html += F("</span></div>");
  html += F("<div class='hero-mini'><span class='hero-mini-label'>Next Start</span><span class='hero-mini-value' id='heroNextValue'>");
  html += nextWaterLabel;
  html += F("</span><span class='hero-mini-sub' id='heroNextSub'>");
  html += nextWaterSub;
  html += F("</span></div>");
  html += F("<div class='hero-mini'><span class='hero-mini-label'>Weather</span><span class='hero-mini-value weather-value'><span id='heroWeatherValue'>");
  html += heroWeatherValue;
  html += F("</span><span id='heroWeatherTrend' style='font-weight:900;'>&rarr;</span></span><span class='hero-mini-sub' id='heroWeatherSub'>");
  html += cond;
  html += F("</span></div>");
  html += F("</div></div></div></div>");
  html += F("<div class='wrap'><div class='dash-nav glass'><a href='#summary-section'>Summary</a><a href='#schedules-section'>Schedules</a><a href='#zones-section'>Zones</a><a href='#controls-section'>Controls</a></div></div>");

  // --- Summary cards ---
  html += F("<div class='wrap section-block' id='summary-section'><div class='section-head'><div><div class='section-kicker'>Overview</div><h2>Controller summary</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='glass section summary-shell'><div class='grid summary-grid'>");

  // Location card with Open-Meteo link
  html += F("<div class='card summary-card'><h3>Location</h3><a class='summary-link' id='meteoLink' href='https://open-meteo.com/en/docs?latitude=-35.1076&longitude=138.5573' target='_blank' rel='noopener'>");
  html += F("<span class='summary-k'>Weather source</span><span class='summary-value' id='cityName'>");
  html += cityName;
  html += F("</span><span class='summary-support'>Click here to goto Open-Meteo site.</span></a></div>");

  html += F("<div class='card summary-card'><h3>Local Time</h3><div class='summary-k'>Device clock</div><div id='upChip' class='summary-value'>--:--:--</div><div class='summary-support'>Controller timezone and DST-aware local time.</div></div>");

  html += F("<div class='card summary-card'><h3>WiFi Signal</h3><div class='summary-k'>Wireless health</div><div id='rssiChip' class='summary-value'>");
  html += String(WiFi.RSSI()); html += F(" dBm</div></div>");

  html += F("<div class='card summary-card'><h3>Tank Level</h3><div class='summary-row'><div><div class='summary-k'>Available reserve</div><div class='summary-value'><span id='tankPctLabel'>");
  html += String(tankPct);
  html += F("%</span></div></div><div id='srcChip' class='mini-chip'>");
  html += sourceModeText();
  html += F("</div></div><div class='meter'><div id='tankFill' class='fill' style='width:");
  html += String(tankPct);
  html += F("%'></div></div></div>");

  html += F("<div class='card summary-card weather-card'><h3>Current Weather</h3><div class='summary-metric-grid'>");
  html += F("<div class='metric-tile'><span class='metric-k'>Temperature</span><div class='metric-v big-metric'><span id='tempChip'>");
  html += (isnan(temp) ? String("--") : String(temp,1)+" C");
  html += F("</span> <span id='tempTrend' style='font-weight:900;'>&rarr;</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Feels Like</span><div class='metric-v' id='feelsChip'>");
  html += (isnan(feels) ? String("--") : String(feels,1)+" C");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Humidity</span><div class='metric-v' id='humChip'>");
  html += (isnan(hum) ? String("--") : String((int)hum)+" %");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Wind</span><div class='metric-v' id='windChip'>");
  html += (isnan(ws) ? String("--") : String(ws,1)+" m/s");
  html += F("</div></div>");
  html += F("</div><div class='summary-metric-grid metric-pair'>");
  html += F("<div class='metric-tile'><span class='metric-k'>Condition</span><div class='metric-v' id='cond'>");
  html += cond.length() ? cond : String("--");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Wind Direction</span><div class='metric-v' id='windDirChip'>");
  html += windDir;
  html += F("</div></div></div>");
  html += F("<div class='summary-subhead'>Today</div><div class='summary-metric-grid'>");
  html += F("<div class='metric-tile'><span class='metric-k'>Low / High</span><div class='metric-split'>");
  html += F("<div class='metric-split-item'><div class='metric-v'><span id='tmin'>---</span><span class='metric-unit'>C</span></div></div>");
  html += F("<div class='metric-split-item'><div class='metric-v'><span id='tmax'>---</span><span class='metric-unit'>C</span></div></div>");
  html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Pressure</span><div class='metric-v'><span id='press'>--</span><span class='metric-unit'>hPa</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Sunrise</span><div class='metric-v' id='sunr'>--:--</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Sunset</span><div class='metric-v' id='suns'>--:--</div></div></div></div>");

  // Delays + Next Water
  html += F("<div class='card summary-card next-card'><h3>Delays & Next Water</h3><div class='summary-meta'>");
  html += F("<div id='rainBadge' class='badge "); html += (rainActive ? "b-bad" : "b-ok"); html += F("'>Rain: <b>");
  html += (rainActive?"Active":"Off"); html += F("</b></div>");
  html += F("<div id='windBadge' class='badge "); html += (windActive ? "b-warn" : "b-ok"); html += F("'>Wind: <b>");
  html += (windActive?"Active":"Off"); html += F("</b></div></div>");
  html += F("<div class='summary-metric-grid'>");
  html += F("<div class='metric-tile metric-wide'><span class='metric-k'>Delay Cause</span><div class='metric-v' id='rainCauseBadge'>"); html += causeText; html += F("</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Rain 1h</span><div class='metric-v'><span id='acc1h'>--</span><span class='metric-unit'>mm</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Rain 24h</span><div class='metric-v'><span id='acc24'>--</span><span class='metric-unit'>mm</span></div></div>");
  html += F("<div class='metric-tile metric-wide'><span class='metric-k'>Zone & Start</span><div class='metric-v' id='nwTime'>");
  html += nextWaterLabel;
  html += F("</div><div class='summary-support'>Zone <span id='nwZone'>--</span></div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>ETA</span><div class='metric-v' id='nwETA'>--</div></div>");
  html += F("<div class='metric-tile'><span class='metric-k'>Duration</span><div class='metric-v' id='nwDur'>--</div></div>");
  html += F("</div><div class='summary-note'>During Rain:Active watering will be stopped, if rain mm threshold is met watering will stop for set hours. If Wind:Active watering will be postponed until wind conditions drop below set threshold.</div></div>");
  html += F("</div></div>"); // end glass / grid
  flush();

  // ---------- Schedules (collapsible) ----------
  static const char* DLBL[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  auto pad2 = [](int v) -> String {
    if (v < 0) v = 0;
    if (v < 10) {
      char b[3];
      snprintf(b, sizeof(b), "0%d", v);
      return String(b);
    }
    return String(v);
  };
  html += F("<div class='wrap section-block' id='schedules-section'><div class='section-head'><div><div class='section-kicker'>Scheduling</div><h2>Zone schedules</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='card sched sched-shell'><div class='sched-top'><div class='sched-top-copy'><div class='section-kicker'>Planner</div><h3>Weekly schedule editor</h3><p></p></div>");
  html += F("<div class='sched-tools'><button class='btn btn-secondary' type='button' id='schedToggle'>Show Schedules</button><button class='btn' id='btn-save-all' title='Save all zone schedules'>Save All</button></div></div>");
  html += F("<div id='schedBody' class='sched-ctr sched-body' style='display:none'>");
  html += F("<div class='sched-grid'>");

  for (int z=0; z<(int)zonesCount; ++z) {
    html += F("<div class='sched-card'><h4><span class='sched-badge'>Z");
    html += String(z + 1);
    html += F("</span> ");
    html += zoneNames[z];
    html += F("</h4><form method='POST' action='/submit'>");
    html += F("<input type='hidden' name='onlyZone' value='"); html += String(z); html += F("'>");

    // Name
    html += F("<div class='rowx'><label>Name</label><div class='field'>");
    html += F("<input class='in' type='text' name='zoneName"); html += String(z);
    html += F("' value='"); html += zoneNames[z]; html += F("' maxlength='32' style='flex:1;min-width:200px'>");
    html += F("</div></div>");

    // Start 1
    html += F("<div class='rowx'><label>Start Time 1 -</label><div class='field inline'>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='startHour"); html += String(z);
    html += F("' value='"); html += pad2(startHour[z]); html += F("'>");
    html += F("<span class='sep'>:</span>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='startMin"); html += String(z);
    html += F("' value='"); html += pad2(startMin[z]); html += F("'>");
    html += F("</div></div>");

    // Start 2 (hide inputs unless enabled)
    html += F("<div class='rowx'><label>Start Time 2 -</label><div class='field inline'>");
    html += F("<span id='start2fields"); html += String(z); html += F("' style='display:");
    html += (enableStartTime2[z] ? "inline-flex" : "none");
    html += F(";align-items:center;gap:8px'>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='startHour2"); html += String(z);
    html += F("' value='"); html += pad2(startHour2[z]); html += F("'>");
    html += F("<span class='sep'>:</span>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='startMin2"); html += String(z);
    html += F("' value='"); html += pad2(startMin2[z]); html += F("'>");
    html += F("</span>");
    html += F("<label class='toggle-inline'><input type='checkbox' name='enableStartTime2");
    html += String(z); html += F("' "); html += (enableStartTime2[z] ? "checked" : "");
    html += F("> Enable</label></div></div>");

    // Duration
    html += F("<div class='rowx'><label>Run Time 1 -</label><div class='field inline'>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,3}' maxlength='3' name='durationMin"); html += String(z);
    html += F("' value='"); html += pad2(durationMin[z]); html += F("'>");
    html += F("<span class='unit'>m</span><span class='sep'>:</span>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='durationSec"); html += String(z);
    html += F("' value='"); html += pad2(durationSec[z]); html += F("'>");
    html += F("<span class='unit'>s</span></div></div>");

    // Duration 2 (used when Start 2 fires)
    html += F("<div class='rowx dur2row' id='dur2row"); html += String(z); html += F("' style='display:");
    html += (enableStartTime2[z] ? "grid" : "none");
    html += F("'><label>Run Time 2 -</label><div class='field inline'>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,3}' maxlength='3' name='duration2Min"); html += String(z);
    html += F("' value='"); html += pad2(duration2Min[z]); html += F("'>");
    html += F("<span class='unit'>m</span><span class='sep'>:</span>");
    html += F("<input class='in' type='text' inputmode='numeric' pattern='[0-9]{1,2}' maxlength='2' name='duration2Sec"); html += String(z);
    html += F("' value='"); html += pad2(duration2Sec[z]); html += F("'>");
    html += F("'</div></div>");

    // Days
    html += F("<div class='rowx'><label>Days</label><div class='days-grid'>");
    for (int d=0; d<7; ++d) {
      html += F("<label class='day'><input type='checkbox' name='day"); html += String(z); html += "_"; html += String(d);
      html += F("' "); html += (days[z][d] ? "checked" : ""); html += F("><span>"); html += DLBL[d]; html += F("</span></label>");
    }
    html += F("</div></div>");

    // Actions
    html += F("<div class='toolbar' style='justify-content:flex-end'><button class='btn' type='submit'>Save Zone</button></div>");
    html += F("</form></div>");
    flush();
  }

  html += F("</div>"); // .sched-grid
  html += F("</div></div></div>"); // #schedBody, .card.sched, .wrap
  flush();

  // --- Live Zones ---
  html += F("<div class='wrap section-block' id='zones-section'><div class='section-head'><div><div class='section-kicker'>Live Control</div><h2>Zone overview</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='card zones-shell'>");

  html += F("<div class='zone-list-head'><h3>Quick Zone Control</h3><div class='zone-list-note'>Compact live rows with manual on/off control.</div></div>");
  html += F("<div class='zone-list'>");

  for (int z = 0; z < (int)zonesCount; z++) {
    unsigned long total = zoneRunTotalSec[z];
    if (total == 0) total = durationForSlot(z, 1);
    unsigned long rem = total;
    if (zoneActive[z]) {
      unsigned long elapsed = (millis() - zoneStartMs[z]) / 1000UL;
      rem = (elapsed < total ? total - elapsed : 0);
    }

    char valueBuf[16];
    fmtMMSS(valueBuf, sizeof(valueBuf), zoneActive[z] ? rem : total);

    html += F("<div class='zone-row ");
    html += (zoneActive[z] ? "is-active" : "");
    html += F("' id='zone-");
    html += String(z);
    html += F("-row'>");

    html += F("<div class='zone-row-main'><span class='zone-index'>Z");
    html += String(z + 1);
    html += F("</span><span id='zone-");
    html += String(z);
    html += F("-dot' class='zone-dot ");
    html += (zoneActive[z] ? "on" : "");
    html += F("'></span><div class='zone-row-copy'><div class='zone-row-name'>");
    html += zoneNames[z];
    html += F("</div><div class='zone-row-sub' id='zone-");
    html += String(z);
    html += F("-meta'>");
    html += (zoneActive[z] ? "Remaining" : "Run time");
    html += F("</div></div></div>");

    html += F("<div class='zone-row-status'><div id='zone-");
    html += String(z);
    html += F("-state' class='badge ");
    html += (zoneActive[z] ? "b-ok" : "");
    html += F("'>");
    html += (zoneActive[z] ? "Running" : "Ready");
    html += F("</div><div class='zone-row-rem' id='zone-");
    html += String(z);
    html += F("-rem'>");
    html += valueBuf;
    html += F("</div></div>");

    html += F("<div class='zone-row-actions'>");
    html += F("<button type='button' class='btn' id='zone-");
    html += String(z);
    html += F("-on' onclick='toggleZone(");
    html += String(z);
    html += F(",1)'");
    if (zoneActive[z]) html += F(" disabled");
    html += F(">On</button>");
    html += F("<button type='button' class='btn btn-danger' id='zone-");
    html += String(z);
    html += F("-off' onclick='toggleZone(");
    html += String(z);
    html += F(",0)'");
    if (!zoneActive[z]) html += F(" disabled");
    html += F(">Off</button>");
    html += F("</div>");

    html += F("</div>");
    flush();
  }

  html += F("</div></div></div>");
  flush();

  // --- System Controls ---
  html += F("<div class='wrap section-block controls-shell' id='controls-section'><div class='section-head'><div><div class='section-kicker'>Actions</div><h2>Controller controls</h2></div>");
  html += F("<p class='section-note'></p></div>");
  html += F("<div class='action-grid'>");
  html += F("<div class='card action-card'><div class='action-copy'><h3>Quick Actions</h3><p>Clear active delays, resume automation, or restart the controller.</p></div><div class='toolbar'><button class='btn btn-secondary' id='btn-clear-delays'>Clear Delays</button><button class='btn btn-secondary' id='btn-resume'>Resume</button><button class='btn btn-danger' id='rebootBtn'>Reboot</button></div></div>");
  html += F("<div class='card action-card'><div class='action-copy'><h3>Deeper Setup</h3><p>Jump into configuration or event history when you need wiring, forecast, timezone.</p></div><div class='toolbar'><a class='btn' href='/setup'>Open Setup</a><a class='btn btn-secondary' href='/events'>View Events</a></div></div>");
  html += F("</div></div>");
  flush();

  // --- JS ---
  html += F("<script>");
  html += F("function pad(n){return n<10?'0'+n:n;}");
  html += F("let _devEpoch=null; let _tickTimer=null; let _lastTemp=null; let _lastTempTrend='\\u2192';");
  html += F("function startDeviceClock(seedSec){_devEpoch=seedSec;if(_tickTimer)clearInterval(_tickTimer);");
  html += F("const draw=()=>{if(_devEpoch==null)return; const d=new Date(_devEpoch*1000);");
  html += F("const h=pad(d.getHours()),m=pad(d.getMinutes()),s=pad(d.getSeconds());");
  html += F("const el=document.getElementById('clock'); if(el) el.textContent=h+':'+m+':'+s; _devEpoch++;};");
  html += F("draw(); _tickTimer=setInterval(draw,1000);} ");
  html += F("function fmtClock12(epoch, offsetMin){");
  html += F(" if(typeof epoch!=='number' || epoch<=0) return '--:--';");
  html += F(" const t=new Date((epoch + (offsetMin||0)*60)*1000);");
  html += F(" let h=t.getUTCHours(); const m=t.getUTCMinutes(); const s=t.getUTCSeconds();");
  html += F(" const am=(h>=12)?'PM':'AM'; h=h%12; if(h===0) h=12;");
  html += F(" return pad(h)+':'+pad(m)+':'+pad(s)+' '+am;");
  html += F("} ");
  html += F("let _busy=false; async function postJson(url,payload){const body=payload?JSON.stringify(payload):\"{}\";");
  html += F("return fetch(url,{method:'POST',headers:{'Content-Type':'application/json','Cache-Control':'no-cache'},body});}");
  html += F("async function postForm(url, body){const opts={method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'}};");
  html += F("if(body)opts.body=body; return fetch(url,opts);} ");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-ghost,.pill').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("async function toggleZone(z,on){if(_busy)return;_busy=true;try{await postJson('/valve/'+(on?'on/':'off/')+z,{t:Date.now()});setTimeout(refreshStatus,200);}catch(e){console.error(e);}finally{_busy=false;}}");

  html += F("const btnReboot=document.getElementById('rebootBtn');");
  html += F("if(btnReboot){btnReboot.addEventListener('click',async()=>{if(confirm('Reboot controller now?')){try{await postJson('/reboot',{t:Date.now()});}catch(e){}}});} ");
  html += F("document.getElementById('btn-clear-delays')?.addEventListener('click',async()=>{await postForm('/clear_delays','a=1');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-24')?.addEventListener('click',async()=>{await postForm('/pause','sec=86400');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-7d')?.addEventListener('click',async()=>{await postForm('/pause','sec='+(7*86400));setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-resume')?.addEventListener('click',async()=>{await postForm('/resume','x=1');setTimeout(refreshStatus,200);});");

  // Master pill
  html += F("const btnMaster=document.getElementById('btn-master');");
  html += F("if(btnMaster){btnMaster.addEventListener('click',async()=>{");
  html += F("  const cur=(btnMaster.getAttribute('aria-pressed')==='true'); const turnOn=!cur;");
  html += F("  try{await fetch('/master',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:turnOn?'on=1':''});");
  html += F("      btnMaster.setAttribute('aria-pressed',turnOn?'true':'false');");
  html += F("      const st=document.getElementById('master-state'); if(st) st.textContent=turnOn?'On':'Off';");
  html += F("  }catch(e){console.error(e);} ");
  html += F("});}");

  // Theme
  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');");
  html += F("if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';");
  html += F("localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("document.getElementById('themeBtn')?.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';");
  html += F("const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});");

  html += F("function toLocalHHMM(epoch){if(!epoch||epoch===0)return'--:--'; const d=new Date(epoch*1000); return pad(d.getHours())+':'+pad(d.getMinutes());}");
  html += F("function nextWaterDayLabel(epoch){if(!epoch||epoch===0)return'--'; const d=new Date(epoch*1000); const now=new Date();");
  html += F("const start=new Date(now.getFullYear(),now.getMonth(),now.getDate()); const target=new Date(d.getFullYear(),d.getMonth(),d.getDate());");
  html += F("const diff=Math.round((target-start)/86400000); if(diff===0) return 'Today'; if(diff===1) return 'Tomorrow';");
  html += F("return d.toLocaleDateString(undefined,{weekday:'short', day:'2-digit', month:'short'});}"); 
  html += F("function nextWaterStartLabel(epoch){if(!epoch||epoch===0)return'--'; return nextWaterDayLabel(epoch)+' '+toLocalHHMM(epoch);}");
  html += F("function fmtDurCompact(s){ if(!(s>0)) return '--'; const m=Math.floor(s/60), sec=s%60; return pad(m)+'m '+pad(sec)+'s'; }");
  html += F("async function refreshStatus(){try{const r=await fetch('/status');const st=await r.json();");
  html += F("if(typeof st.deviceEpoch==='number' && st.deviceEpoch>0 && _devEpoch===null){ startDeviceClock(st.deviceEpoch); }");
  html += F("const rb=document.getElementById('rainBadge');const wb=document.getElementById('windBadge');");
  html += F("if(rb){rb.className='badge '+(st.rainDelayActive?'b-bad':'b-ok');rb.innerHTML='Rain: <b>'+(st.rainDelayActive?'Active':'Off')+'</b>';}");
  html += F("if(wb){wb.className='badge '+(st.windDelayActive?'b-warn':'b-ok');wb.innerHTML='Wind: <b>'+(st.windDelayActive?'Active':'Off')+'</b>';}");
  html += F("const cause=document.getElementById('rainCauseBadge'); if(cause) cause.textContent=st.rainDelayCause||'Off';");
  html += F("const pct=st.tankPct||0; const tf=document.getElementById('tankFill'); const tl=document.getElementById('tankPctLabel');");
  html += F("if(tf) tf.style.width=Math.max(0,Math.min(100,pct))+'%'; if(tl) tl.textContent=pct+'%';");
  html += F("const src=document.getElementById('srcChip'); if(src) src.textContent=st.sourceMode||'';");
  html += F("const heroTank=document.getElementById('heroTankValue'); const heroTankSub=document.getElementById('heroTankSub');");
  html += F("if(heroTank) heroTank.textContent=pct+'%'; if(heroTankSub) heroTankSub.textContent=st.sourceMode||'';");
  html += F("const up=document.getElementById('upChip'); if(up) up.textContent=fmtClock12(st.deviceEpoch, st.utcOffsetMin);");
  html += F("const rssi=document.getElementById('rssiChip'); if(rssi) rssi.textContent=(st.rssi)+' dBm';");
  // Location chip + Open-Meteo link
  html += F("const cityEl=document.getElementById('cityName'); const cityLink=document.getElementById('meteoLink');");
  html += F("if(typeof st.cityName==='string' && st.cityName.length){");
  html += F("  if(cityEl) cityEl.textContent=st.cityName;");
  html += F("}");
  html += F("if(cityLink){");
  html += F("  const lat=Number.isFinite(st.lat)?st.lat:null;");
  html += F("  const lon=Number.isFinite(st.lon)?st.lon:null;");
  html += F("  if(lat!==null && lon!==null){ cityLink.href='https://open-meteo.com/en/docs?latitude='+lat+'&longitude='+lon; }");
  html += F("  else { cityLink.href='https://open-meteo.com/en/docs?latitude=-35.1076&longitude=138.5573'; }");
  html += F("}");




  // 1h & 24h
  html += F("var acc1h=document.getElementById('acc1h');");
  html += F("if(acc1h){var v=(typeof st.rain1hNow==='number')?st.rain1hNow:NaN;acc1h.textContent=isNaN(v)?'--':v.toFixed(1);}");
  html += F("const acc24=document.getElementById('acc24'); if(acc24){ const v=(typeof st.rain24hActual==='number')?st.rain24hActual:(typeof st.rain24h==='number'?st.rain24h:NaN); acc24.textContent=isNaN(v)?'--':v.toFixed(1);} ");

  html += F("let activeCount=0; if(Array.isArray(st.zones)){ st.zones.forEach((z,idx)=>{");
  html += F("if(z.active){ activeCount++; }");
  html += F("const stateEl=document.getElementById('zone-'+idx+'-state'); const remEl=document.getElementById('zone-'+idx+'-rem'); const dotEl=document.getElementById('zone-'+idx+'-dot'); const rowEl=document.getElementById('zone-'+idx+'-row'); const metaEl=document.getElementById('zone-'+idx+'-meta');");
  html += F("if(stateEl){stateEl.className='badge '+(z.active?'b-ok':'');stateEl.textContent=z.active?'Running':'Ready';}");
  html += F("if(dotEl){dotEl.className='zone-dot'+(z.active?' on':'');} if(rowEl){rowEl.classList.toggle('is-active',!!z.active);}");
  html += F("if(metaEl) metaEl.textContent=z.active?'Remaining':'Run time';");
  html += F("if(remEl){ remEl.textContent=z.active?fmtDurCompact(z.remaining||0):fmtDurCompact(z.totalSec||0); }");
  html += F("const onBtn=document.getElementById('zone-'+idx+'-on'); const offBtn=document.getElementById('zone-'+idx+'-off');");
  html += F("if(onBtn) onBtn.disabled=!!z.active; if(offBtn) offBtn.disabled=!z.active;");
  html += F("}); }");


  // Weather stats
  html += F("const tmin=document.getElementById('tmin'); const tmax=document.getElementById('tmax'); const sunr=document.getElementById('sunr'); const suns=document.getElementById('suns'); const press=document.getElementById('press');");
  html += F("if(tmin) tmin.textContent=(st.tmin??0).toFixed(0);");
  html += F("if(tmax) tmax.textContent=(st.tmax??0).toFixed(0);");
  html += F("if(sunr) sunr.textContent = st.sunriseLocal || '--:--';");
  html += F("if(suns) suns.textContent = st.sunsetLocal  || '--:--';");
  html += F("if(press){ const p=st.pressure; press.textContent=(typeof p==='number' && p>0)?p.toFixed(0):'--'; }");
  html += F("const tempEl=document.getElementById('tempChip'); const trendEl=document.getElementById('tempTrend');");
  html += F("const heroWeather=document.getElementById('heroWeatherValue'); const heroWeatherTrend=document.getElementById('heroWeatherTrend'); const heroWeatherSub=document.getElementById('heroWeatherSub');");
  html += F("if(tempEl){ const v=st.temp;");
  html += F("  if(typeof v==='number'){");
  html += F("    tempEl.textContent=v.toFixed(1)+' C';");
  html += F("    if(_lastTemp!==null){ const d=v-_lastTemp; if(d>0.1) _lastTempTrend='\\u2191'; else if(d<-0.1) _lastTempTrend='\\u2193'; }");
  html += F("    _lastTemp=v;");
  html += F("  } else { tempEl.textContent='--'; _lastTemp=null; _lastTempTrend='\\u2192'; }");
  html += F("  if(trendEl){ trendEl.textContent=_lastTempTrend; trendEl.style.color=(_lastTempTrend==='\\u2191')?'#16a34a':(_lastTempTrend==='\\u2193')?'#dc2626':'inherit'; }");
  html += F("  if(heroWeatherTrend){ heroWeatherTrend.textContent=_lastTempTrend; heroWeatherTrend.style.color=(_lastTempTrend==='\\u2191')?'#16a34a':(_lastTempTrend==='\\u2193')?'#dc2626':'inherit'; }");
  html += F("}");
  html += F("const feelsEl=document.getElementById('feelsChip'); if(feelsEl){ const v=st.feels_like; feelsEl.textContent=(typeof v==='number')?v.toFixed(1)+' C':'--'; }");
  html += F("const humEl=document.getElementById('humChip'); if(humEl){ const v=st.humidity; humEl.textContent=(typeof v==='number')?Math.round(v)+' %':'--'; }");
  html += F("const windEl=document.getElementById('windChip'); if(windEl){ const v=st.wind; windEl.textContent=(typeof v==='number')?v.toFixed(1)+' m/s':'--'; }");
  html += F("const windDirEl=document.getElementById('windDirChip'); if(windDirEl){ const v=(typeof st.windDirText==='string'&&st.windDirText.length)?st.windDirText:'--'; windDirEl.textContent=v; }");
  html += F("const condEl=document.getElementById('cond');");
  html += F("const cd=(typeof st.condDesc==='string' && st.condDesc.length)?st.condDesc:'';");
  html += F("const cm=(typeof st.condMain==='string' && st.condMain.length)?st.condMain:'';");
  html += F("const condText=cd||cm||'--'; if(condEl){ condEl.textContent=condText; }");
  html += F("if(heroWeather){ const v=st.temp; heroWeather.textContent=(typeof v==='number')?v.toFixed(1)+' C':'--'; }");
  html += F("if(heroWeatherSub) heroWeatherSub.textContent=(condText!=='--')?condText:'Weather data syncing';");

  // keep master pill synced
  html += F("const bm=document.getElementById('btn-master'); const ms=document.getElementById('master-state');");
  html += F("if(bm && ms && typeof st.masterOn==='boolean'){ bm.setAttribute('aria-pressed', st.masterOn?'true':'false'); ms.textContent = st.masterOn?'On':'Off'; }");
  html += F("const hm=document.getElementById('heroMasterState'); const hms=document.getElementById('heroMasterSub');");
  html += F("if(hm){ if(st.systemPaused){ hm.textContent='Paused'; if(hms) hms.textContent='Schedules are temporarily suspended'; }");
  html += F("else if(st.masterOn){ hm.textContent='Master On'; if(hms) hms.textContent=activeCount?(activeCount+' zone'+(activeCount===1?'':'s')+' running'):'Automation ready'; }");
  html += F("else { hm.textContent='Master Off'; if(hms) hms.textContent='Outputs are blocked'; } }");

  // Next Water
  html += F("(function(){ const zEl=document.getElementById('nwZone'); const tEl=document.getElementById('nwTime'); const eEl=document.getElementById('nwETA'); const dEl=document.getElementById('nwDur');");
  html += F("const hv=document.getElementById('heroNextValue'); const hs=document.getElementById('heroNextSub');");
  html += F("const epoch=st.nextWaterEpoch|0; const zone=st.nextWaterZone; const name=st.nextWaterName||(Number.isInteger(zone)?('Z'+(zone+1)):'--'); const dur=st.nextWaterDurSec|0;");
  html += F("function fmtDur(s){ if(s<=0) return '--'; const m=Math.floor(s/60), sec=s%60; return pad(m)+'m '+pad(sec)+'s'; }");
  html += F("function fmtETA(delta){ if(delta<=0) return 'now'; const h=Math.floor(delta/3600), m=Math.floor((delta%3600)/60); if(h>0) return h+'h '+m+'m'; return m+'m'; }");
  html += F("if(zEl) zEl.textContent=(zone>=0&&zone<255)?name:'--'; if(tEl) tEl.textContent=nextWaterStartLabel(epoch); if(dEl) dEl.textContent=fmtDur(dur);");
  html += F("let nowEpoch=(typeof st.deviceEpoch==='number'&&st.deviceEpoch>0&&_devEpoch!=null)?_devEpoch:Math.floor(Date.now()/1000);");
  html += F("if(eEl) eEl.textContent=epoch?fmtETA(epoch-nowEpoch):'--';");
  html += F("if(hv) hv.textContent=nextWaterStartLabel(epoch);");
  html += F("if(hs) hs.textContent=epoch?(name+(dur>0?(' - '+fmtDur(dur)):'')):(st.rainDelayActive?'Waiting for rain delay to clear':'No queued run');");
  html += F("})();");

  html += F("}catch(e){} } setInterval(refreshStatus,2500); refreshStatus();");

  // expose zonesCount & Save All
  html += F("const ZC="); html += String(zonesCount); html += F(";");
  html += F("async function saveAll(){");
  html += F("  const fd=new URLSearchParams();");
  html += F("  for(let z=0; z<ZC; z++){");
  html += F("    const q=n=>document.querySelector(`[name='${n}']`);");
  html += F("    const add=(k)=>{const el=q(k); if(el){ if((el.type||'').toLowerCase()==='checkbox'){ if(el.checked) fd.append(k,'on'); } else { fd.append(k,el.value); } } };");
  html += F("    add('zoneName'+z); add('startHour'+z); add('startMin'+z); add('startHour2'+z); add('startMin2'+z); add('durationMin'+z); add('durationSec'+z); add('duration2Min'+z); add('duration2Sec'+z);");
  html += F("    add('enableStartTime2'+z);");
  html += F("    for(let d=0; d<7; d++) add('day'+z+'_'+d);");
  html += F("  }");
  html += F("  try{ await fetch('/submit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()}); location.reload(); }catch(e){ console.error(e); }");
  html += F("} ");
  html += F("document.getElementById('btn-save-all')?.addEventListener('click', saveAll);");
  html += F("const schedBody=document.getElementById('schedBody'); const schedToggle=document.getElementById('schedToggle');");
  html += F("if(schedBody && schedToggle){ const syncSched=()=>{ const open=schedBody.style.display!=='none'; schedToggle.textContent=open?'Hide Schedules':'Show Schedules'; };");
  html += F("schedToggle.addEventListener('click',()=>{ const open=schedBody.style.display!=='none'; schedBody.style.display=open?'none':'block'; syncSched(); }); syncSched(); }");
  // Toggle Duration 2 rows when Start 2 is enabled
  html += F("for(let z=0; z<ZC; z++){");
  html += F("  const cb=document.querySelector(`[name='enableStartTime2${z}']`);");
  html += F("  const row=document.getElementById('dur2row'+z);");
  html += F("  const s2=document.getElementById('start2fields'+z);");
  html += F("  if(!cb||!row) continue;");
  html += F("  const sync=()=>{row.style.display=cb.checked?'grid':'none'; if(s2) s2.style.display=cb.checked?'inline-flex':'none';};");
  html += F("  cb.addEventListener('change', sync); sync();");
  html += F("}");

  // Weather model: toggle custom input
  html += F("const modelSel=document.getElementById('meteoModelSelect'); const customRow=document.getElementById('meteoModelCustomRow');");
  html += F("if(modelSel && customRow){ const syncModel=()=>{ customRow.style.display=(modelSel.value==='custom')?'flex':'none'; };");
  html += F("modelSel.addEventListener('change', syncModel); syncModel(); }");

  html += F("</script></body></html>");
  flush();
  server.sendContent("");
}

// Setup Page 
void handleSetupPage() {
  HttpScope _scope;
  loadConfig();
  sanitizePinConfig();
  String html; html.reserve(26000);
  const int uiMaxGpio = 
  #if defined(CONFIG_IDF_TARGET_ESP32)
    39;
  #else
    48;
  #endif
  String latStr = isfinite(meteoLat) ? String(meteoLat, 6) : String("");
  String lonStr = isfinite(meteoLon) ? String(meteoLon, 6) : String("");
  String setupWeatherLabel = meteoLocation.length() ? meteoLocation : String("Open-Meteo ready");
  String setupDisplayLabel = displayUseTft ? String("TFT display") : String("OLED display");
  String setupTankLabel = tankEnabled ? String("Tank Enabled") : String("City Water");
  String setupTzLabel = (tzMode == TZ_FIXED) ? String("Fixed offset") : String("POSIX timezone");

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#145b63'><meta name='color-scheme' content='light dark'>");
  html += F("<title>Setup - ESP32 Irrigation</title>");
  html += F("<style>");
  html += F("@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@500;700&family=Sora:wght@400;600;700;800&display=swap');");
  html += F("html{scroll-behavior:smooth}body{margin:0;font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;background:#0d1718;color:#e7f1ec;font-size:15px;line-height:1.4}");
  html += F(".wrap{max-width:1100px;margin:28px auto;padding:0 16px}");
  html += F("h1{margin:0 0 16px 0;font-size:1.7em;letter-spacing:.3px;font-weight:800}");
  html += F(".page-head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:16px}");
  html += F(".page-head h1{margin:0}");
  html += F(".page-head-copy{display:flex;flex-direction:column;gap:4px}");
  html += F(".page-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:#7fd2bd}");
  html += F(".page-sub{color:#9ab4ad;font-size:.92rem}");
  html += F(".setup-hero{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(280px,.8fr);gap:14px;padding:18px;margin:16px 0 18px;border-radius:18px;");
  html += F("background:linear-gradient(145deg,rgba(15,27,30,.86),rgba(20,69,68,.6));border:1px solid rgba(92,131,125,.24);box-shadow:0 16px 38px rgba(0,0,0,.28)}");
  html += F(".setup-hero-copy{display:flex;flex-direction:column;justify-content:center;gap:10px}");
  html += F(".setup-hero-copy h2{margin:0;font-size:1.55rem;line-height:1.08;max-width:14ch}");
  html += F(".setup-hero-copy p{margin:0;color:#b4c8c2;max-width:60ch}");
  html += F(".setup-badges{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}");
  html += F(".setup-badge{padding:12px 13px;border-radius:15px;border:1px solid rgba(92,131,125,.24);background:linear-gradient(180deg,rgba(255,255,255,.08),rgba(127,210,189,.04))}");
  html += F(".setup-badge-k{font-size:.73rem;letter-spacing:.14em;text-transform:uppercase;color:#9ab4ad;font-weight:800}");
  html += F(".setup-badge-v{margin-top:6px;font-size:1rem;font-weight:760;color:#f0f8f3}");
  html += F(".setup-nav{display:flex;gap:10px;flex-wrap:wrap;margin:0 0 16px;padding:8px;border-radius:16px;position:sticky;top:10px;z-index:8;");
  html += F("background:rgba(12,22,24,.76);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border:1px solid rgba(92,131,125,.2);box-shadow:0 10px 28px rgba(0,0,0,.16)}");
  html += F(".setup-nav a{display:inline-flex;align-items:center;justify-content:center;padding:9px 14px;border-radius:999px;border:1px solid rgba(92,131,125,.24);");
  html += F("background:rgba(10,23,25,.58);color:#e7f1ec;font-size:.86rem;font-weight:760;letter-spacing:.01em;transition:transform .08s ease,background .12s ease,border-color .12s ease,box-shadow .12s ease}");
  html += F(".setup-nav a:hover{transform:translateY(-1px);background:rgba(31,138,112,.18);border-color:rgba(127,210,189,.36);box-shadow:0 10px 24px rgba(20,91,99,.16)}");
  html += F(".setup-actions-top{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:0 0 16px;padding:10px 12px;border-radius:16px;position:sticky;top:74px;z-index:7;background:rgba(13,23,24,.82);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border:1px solid rgba(92,131,125,.2);box-shadow:0 12px 28px rgba(0,0,0,.18)}");
  html += F(".setup-actions-top .btn,.setup-actions-top .btn-alt{margin:0}");
  html += F(".setup-actions-top .btn[type=submit]{min-width:160px}");
  html += F(".card-intro{margin:0 0 8px;color:#9ab4ad;font-size:.9rem;max-width:62ch}");
  html += F(".theme-switch{display:flex;align-items:center;gap:6px;font-weight:700;color:#d5e4de}");
  html += F(".switch{position:relative;display:inline-block;width:42px;height:24px;min-width:unset}");
  html += F(".switch input{opacity:0;width:0;height:0}");
  html += F(".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#16262a;border:1px solid #284449;transition:.2s;border-radius:999px}");
  html += F(".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;top:2px;background:#e7f1ec;transition:.2s;border-radius:50%}");
  html += F("input:checked + .slider{background:#1f8a70;border-color:#1f8a70}");
  html += F("input:checked + .slider:before{transform:translateX(18px);background:#fff}");
  html += F(".card{background:#101d20;border:1px solid #233d42;border-radius:16px;box-shadow:0 8px 34px rgba(0,0,0,.35);padding:18px 16px;margin-bottom:16px}");
  html += F(".card.narrow{max-width:960px;margin-left:auto;margin-right:auto}");
  html += F(".card h3{margin:0 0 12px 0;font-size:1.1em;font-weight:850;letter-spacing:.35px;color:#f0f8f3;");
  html += F("padding-bottom:6px;border-bottom:2px solid #1f8a70;display:flex;align-items:center;gap:8px}");
  html += F("label{display:inline-block;min-width:200px;font-size:.95rem;font-weight:600;color:#d5e4de;text-align:left}");
  // Inputs + select share the same theme
  html += F("input[type=text],input[type=number],select{background:#0c1618;color:#e7f1ec;border:1px solid #29454a;border-radius:12px;padding:9px 12px;font-size:.95rem;text-align:left}");
  html += F("input[type=text],select{width:100%;max-width:520px}");
  html += F("input[type=number]{width:100%;max-width:200px}");
  html += F("input[type=text].in-wide,select.in-wide{max-width:600px}");
  html += F("input[type=text].in-med,select.in-med{max-width:440px}");
  html += F("input[type=number].in-xs{max-width:120px}");
  html += F("input[type=number].in-sm{max-width:160px}");
  html += F("input[type=number].in-md{max-width:240px}");
  html += F(".row{display:flex;align-items:center;gap:12px;margin:10px 0;padding-top:10px;border-top:1px solid rgba(41,69,74,.7);flex-wrap:wrap}.row small{color:#9ab4ad;font-size:.85rem}");
  html += F(".card h3 + .row{border-top:none;padding-top:0}");
  html += F(".btn{background:linear-gradient(180deg,#1f8a70,#145b63);color:#fff;border:1px solid rgba(0,0,0,.18);border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;box-shadow:0 6px 16px rgba(20,91,99,.28);font-size:.95rem}");
  html += F(".btn-alt{background:#17272b;color:#e7f1ec;border:1px solid #29454a;border-radius:12px;padding:10px 14px;font-size:.95rem}");
  html += F(".btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border:1px solid rgba(185,28,28,.6)}");
  html += F(".btn,.btn-alt{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active,.btn-alt:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-alt{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".grid{display:grid;grid-template-columns:1fr;gap:14px}");
  html += F(".cols2{display:grid;grid-template-columns:1fr 1fr;gap:14px}");
  html += F(".panel-split > div{padding:14px;border-radius:14px;border:1px solid rgba(92,131,125,.16);background:linear-gradient(180deg,rgba(255,255,255,.03),rgba(255,255,255,.015))}");
  html += F("@media(max-width:760px){.cols2{grid-template-columns:1fr}label{min-width:150px}input[type=text].in-wide,select.in-wide,input[type=text].in-med,select.in-med{max-width:100%}}");
  html += F(".switchline{display:flex;gap:12px;align-items:center;flex-wrap:wrap}");
  html += F("input[type=checkbox]:not(#themeToggle),input[type=radio]{appearance:none;-webkit-appearance:none;width:18px;height:18px;margin:0;flex:0 0 18px;display:inline-grid;place-content:center;cursor:pointer;");
  html += F("border:1.6px solid rgba(255,255,255,.78);background:transparent;box-shadow:none;transition:border-color .12s ease,background .12s ease,box-shadow .12s ease,transform .06s ease}");
  html += F("input[type=checkbox]:not(#themeToggle){border-radius:6px}");
  html += F("input[type=radio]{border-radius:999px}");
  html += F("input[type=checkbox]:not(#themeToggle)::before,input[type=radio]::before{content:'';display:block;transform:scale(0);transition:transform .12s ease}");
  html += F("input[type=checkbox]:not(#themeToggle)::before{width:5px;height:9px;border-right:2px solid #fff;border-bottom:2px solid #fff;transform:rotate(45deg) scale(0);margin-top:-1px}");
  html += F("input[type=radio]::before{width:8px;height:8px;border-radius:999px;background:#fff}");
  html += F("input[type=checkbox]:not(#themeToggle):checked,input[type=radio]:checked{border-color:#ffffff;background:transparent;box-shadow:none}");
  html += F("input[type=checkbox]:not(#themeToggle):checked::before{transform:rotate(45deg) scale(1)}");
  html += F("input[type=radio]:checked::before{transform:scale(1)}");
  html += F("input[type=checkbox]:not(#themeToggle):focus-visible,input[type=radio]:focus-visible{outline:none;border-color:#ffffff;box-shadow:0 0 0 3px rgba(31,138,112,.2)}");
  html += F("input[type=checkbox]:not(#themeToggle):active,input[type=radio]:active{transform:translateY(1px)}");
  html += F(".subhead{opacity:.85;margin:8px 0 6px 0;font-weight:700;font-size:.98rem}");
  html += F(".hr{height:1px;background:#233d42;margin:10px 0 8px 0;border:none}");

  // Chips + inline options for timezone mode
  html += F(".inline-options{display:flex;flex-wrap:wrap;gap:10px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;background:#0c1618;border:1px solid #29454a;font-size:.85rem;color:#e7f1ec}");
  html += F(".chip input{margin:0}");
  html += F(".chip span{white-space:nowrap}");

  // Collapsible card styling
  html += F(".card,.setup-nav,.setup-actions-top,[id$='-card']{scroll-margin-top:144px}");
  html += F("details.collapse{padding:0;margin:0}");
  html += F("details.collapse summary{cursor:pointer;outline:none;list-style:none;font-weight:800;font-size:1.05rem;color:#e7f1ec;padding:4px 0}");
  html += F("details.collapse summary::-webkit-details-marker{display:none}");
  html += F("details.collapse summary:after{content:'▸';margin-left:8px;transition:transform .18s ease;display:inline-block;width:26px;height:26px;line-height:24px;text-align:center;border-radius:999px;border:1px solid rgba(92,131,125,.22);background:rgba(255,255,255,.04)}");
  html += F("details.collapse[open] summary:after{transform:rotate(90deg)}");
  html += F("details.collapse[open] summary:after{background:rgba(31,138,112,.16);border-color:rgba(127,210,189,.3)}");
  html += F(".collapse-body{margin-top:10px;padding-top:4px}");

  // Help text row: let the text wrap nicely
  html += F(".helptext label{min-width:0}");
  html += F(".helptext small{max-width:520px;display:block}");

  // Make native select look like themed dropdown with a chevron
  html += F("select{appearance:none;-webkit-appearance:none;-moz-appearance:none;padding-right:32px;");
  html += F("background-image:linear-gradient(45deg,transparent 50%,#93aba4 50%),linear-gradient(135deg,#93aba4 50%,transparent 50%);");
  html += F("background-position:calc(100% - 18px) 50%,calc(100% - 13px) 50%;background-size:5px 5px,5px 5px;background-repeat:no-repeat;}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 12px 28px rgba(0,0,0,.22)}");
  html += F(".btn:hover,.btn-alt:hover{filter:brightness(1.06)}");
  html += F("input:focus-visible,select:focus-visible{outline:2px solid #1f8a70;outline-offset:2px}");
  html += F("a{text-decoration:none;color:inherit}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");

  // Light theme overrides
  html += F("html[data-theme='light'] body{background:#f1f5f1;color:#1c2b2c}");
  html += F("html[data-theme='light'] .theme-switch{color:#314847}");
  html += F("html[data-theme='light'] .card{background:#ffffff;border-color:#d4dfd8;box-shadow:0 10px 26px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .card h3{color:#14232b;border-bottom-color:#1f8a70}");
  html += F("html[data-theme='light'] label{color:#213433}");
  html += F("html[data-theme='light'] .row small{color:#607571}");
  html += F("html[data-theme='light'] input[type=text],html[data-theme='light'] input[type=number],html[data-theme='light'] select{background:#f7faf8;color:#14232b;border-color:#c9d9d0}");
  html += F("html[data-theme='light'] .chip{background:#eef6f2;border-color:#c9d9d0;color:#213433}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle),html[data-theme='light'] input[type=radio]{border-color:#8aa59c;background:rgba(255,255,255,.78);box-shadow:inset 0 1px 0 rgba(255,255,255,.75)}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle)::before{border-right-color:#17666b;border-bottom-color:#17666b}");
  html += F("html[data-theme='light'] input[type=radio]::before{background:#17666b}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):checked,html[data-theme='light'] input[type=radio]:checked{border-color:#1f8a70;background:#ffffff;box-shadow:0 0 0 3px rgba(31,138,112,.12)}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):checked::before{border-right-width:2.4px;border-bottom-width:2.4px}");
  html += F("html[data-theme='light'] input[type=checkbox]:not(#themeToggle):focus-visible,html[data-theme='light'] input[type=radio]:focus-visible{border-color:#1f8a70;box-shadow:0 0 0 3px rgba(31,138,112,.14)}");
  html += F("html[data-theme='light'] .btn-alt{background:#e8efeb;color:#213433;border-color:#c9d9d0}");
  html += F("html[data-theme='light'] .btn{background:linear-gradient(180deg,#238b74,#17666b)}");
  html += F("html[data-theme='light'] .btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c)}");
  html += F("html[data-theme='light'] .hr{background:#d4dfd8}");
  html += F("html[data-theme='light'] .setup-hero{background:linear-gradient(145deg,rgba(255,255,255,.88),rgba(235,245,240,.86));border-color:#d4dfd8;box-shadow:0 14px 34px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .page-kicker{color:#1f8a70}");
  html += F("html[data-theme='light'] .page-sub{color:#607571}");
  html += F("html[data-theme='light'] .setup-hero-copy p{color:#607571}");
  html += F("html[data-theme='light'] .setup-badge{background:linear-gradient(180deg,#ffffff,#f1f7f3);border-color:#d4dfd8}");
  html += F("html[data-theme='light'] .setup-badge-k{color:#607571}");
  html += F("html[data-theme='light'] .setup-badge-v{color:#14232b}");
  html += F("html[data-theme='light'] .setup-nav{background:rgba(255,255,255,.8);border-color:#d4dfd8;box-shadow:0 10px 28px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .setup-nav a{background:rgba(255,255,255,.88);border-color:#d4dfd8;color:#213433}");
  html += F("html[data-theme='light'] .setup-nav a:hover{background:#edf6f1;border-color:#b7d5c8;box-shadow:0 10px 24px rgba(31,138,112,.08)}");
  html += F("html[data-theme='light'] .card-intro{color:#607571}");
  html += F("html[data-theme='light'] .row{border-top-color:#e4ece7}");
  html += F("html[data-theme='light'] select{background-image:linear-gradient(45deg,transparent 50%,#607571 50%),linear-gradient(135deg,#607571 50%,transparent 50%)}");
  html += F("html[data-theme='light'] .setup-actions-top{background:rgba(255,255,255,.84);border-color:#d4dfd8;box-shadow:0 12px 28px rgba(16,24,40,.08)}");
  html += F("html[data-theme='light'] .panel-split > div{border-color:#dfe9e3;background:linear-gradient(180deg,#ffffff,#f7fbf8)}");
  html += F("html[data-theme='light'] details.collapse summary:after{border-color:#d4dfd8;background:#f7fbf8}");
  html += F("html[data-theme='light'] details.collapse[open] summary:after{background:#edf6f1;border-color:#b7d5c8}");

  // Desktop tuning
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{max-width:1160px;padding:0 20px;}");
  html += F(".card{padding:20px 18px;}");
  html += F(".row{justify-content:flex-start;}");
  html += F("label{min-width:240px;}");
  html += F("input[type=text],select{max-width:560px;}");
  html += F("input[type=number]{max-width:220px;}");
  html += F("}");
  html += F("html,body{font-family:'Sora','Avenir Next','Trebuchet MS',sans-serif}");
  html += F("body{position:relative;min-height:100vh;background:radial-gradient(900px 480px at 0% -10%,rgba(31,138,112,.2),transparent),radial-gradient(900px 500px at 110% 8%,rgba(82,194,102,.14),transparent),#0d1718}");
  html += F(".page-head{padding:10px 14px;border-radius:15px;background:rgba(16,27,30,.48);backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px);border:1px solid rgba(92,131,125,.24)}");
  html += F(".page-head h1{text-transform:uppercase;font-size:1.35rem;letter-spacing:1px}");
  html += F(".theme-switch{font-size:.88rem}");
  html += F(".card{position:relative;overflow:hidden}");
  html += F(".card::before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,#1f8a70,#52c266)}");
  html += F("input[type=text],input[type=number],select{font-family:'Sora','Trebuchet MS',sans-serif}");
  html += F("input[type=text]:focus-visible,input[type=number]:focus-visible,select:focus-visible{outline:2px solid #1f8a70;outline-offset:1px;box-shadow:0 0 0 3px rgba(31,138,112,.18)}");
  html += F(".btn,.btn-alt{font-weight:760;letter-spacing:.16px}");
  html += F(".chip{font-weight:620}");
  html += F("details.collapse summary{display:flex;align-items:center;justify-content:space-between}");
  html += F(".card{animation:rise .42s ease both}");
  html += F(".card:nth-of-type(2){animation-delay:.04s}.card:nth-of-type(3){animation-delay:.08s}.card:nth-of-type(4){animation-delay:.12s}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:translateY(0)}}");
  html += F("html[data-theme='light'] body{background:radial-gradient(900px 470px at 0% -10%,rgba(31,138,112,.18),transparent),radial-gradient(900px 450px at 110% 6%,rgba(82,194,102,.12),transparent),#f1f5f1}");
  html += F("html[data-theme='light'] .page-head{background:rgba(255,255,255,.72);border-color:#d4dfd8}");
  html += F("@media(max-width:760px){.page-head{padding:10px 12px}.page-head h1{font-size:1.2rem}.setup-hero{grid-template-columns:1fr}.setup-badges{grid-template-columns:1fr 1fr}.setup-nav{top:8px;flex-wrap:nowrap;overflow:auto;padding-bottom:6px}.setup-nav a{white-space:nowrap}.setup-actions-top{top:68px;z-index:9;padding:10px;border-radius:14px;background:rgba(13,23,24,.88);backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);border:1px solid rgba(92,131,125,.2)}.row{padding-top:8px;flex-direction:column;align-items:stretch}.row label{min-width:0;width:100%}.row .btn,.row .btn-alt{width:100%}.switchline{align-items:flex-start}}");
  html += F("</style></head><body>");

  html += F("<div class='wrap'><div class='page-head'><div class='page-head-copy'><div class='page-kicker'>Controller setup</div><h1>Setup</h1><div class='page-sub'>Adjust weather, display, relay and automation settings from one screen.</div></div>");
  html += F("<div class='theme-switch'><span>Light</span><label class='switch'><input type='checkbox' id='themeToggle'><span class='slider'></span></label><span>Dark</span></div>");
  html += F("</div>");
  html += F("<div class='setup-hero'><div class='setup-hero-copy'><div class='page-kicker'>Quick overview</div><h2>Configure the controller without digging through separate pages.</h2><p>Save writes changes immediately, while pin, display and some hardware changes may require a reboot before they take effect.</p></div><div class='setup-badges'>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Zones</div><div class='setup-badge-v'>"); html += String(zonesCount); html += F(" configured</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Display</div><div class='setup-badge-v'>"); html += setupDisplayLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Water Source</div><div class='setup-badge-v'>"); html += setupTankLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Timezone</div><div class='setup-badge-v'>"); html += setupTzLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Weather</div><div class='setup-badge-v'>"); html += setupWeatherLabel; html += F("</div></div>");
  html += F("<div class='setup-badge'><div class='setup-badge-k'>Forecast Model</div><div class='setup-badge-v'>"); html += meteoModel; html += F("</div></div>");
  html += F("</div></div>");
  html += F("<div class='setup-nav'>");
  html += F("<a href='#zones-card'>Zones</a><a href='#tank-card'>Source</a><a href='#delays-card'>Delays</a><a href='#weather-card'>Weather</a><a href='#timezone-card'>Timezone</a><a href='#display-card'>Display</a><a href='#advanced-card'>GPIO</a><a href='#mqtt-card'>Buttons/MQTT</a>");
  html += F("</div><form action='/configure' method='POST'>");
  html += F("<div class='setup-actions-top'><button class='btn' type='submit'>Save Changes</button><button class='btn-alt' formaction='/' formmethod='GET'>Home</button><button class='btn-alt' type='button' onclick=\"fetch('/clear_cooldown',{method:'POST'})\">Clear Cooldown</button><button class='btn btn-danger' type='button' onclick=\"if(confirm('Reboot controller now?'))fetch('/reboot',{method:'POST'})\">Reboot</button></div>");

  // Zones
  html += F("<div class='card narrow' id='zones-card'><h3>Zones</h3><p class='card-intro'>Set how many watering zones are available and whether they run one at a time or together.</p>");
  html += F("<div class='row'><label>Zone Count</label><input class='in-xs' type='number' min='1' max='");
  html += String(MAX_ZONES);
  html += F("' name='zonesMode' value='");
  html += String(zonesCount);
  html += F("'><small>Tank/Mains works with any zone count. Up to ");
  html += String(MAX_ZONES);
  html += F(" zones supported.</small></div>");

  // Run mode
  html += F("<div class='row switchline'><label>Run Mode</label>");
  html += F("<label><input type='checkbox' name='runConcurrent' "); html += (runZonesConcurrent ? "checked" : "");
  html += F("> Run Zones Together</label><small>Unchecked = One at a time. If enabled, ensure your power supply can handle multiple valves running at once.</small></div>");
  html += F("</div>");

  // Tank (available for all modes; water source switching works with any zone count)
  html += F("<div class='card narrow' id='tank-card'><h3>Tank & Water Source</h3><p class='card-intro'>Control how the controller chooses between tank and mains and where the tank sensor is connected.</p>");
  html += F("<div class='row switchline'><label>Enable Tank</label><input type='checkbox' name='tankEnabled' ");
  html += (tankEnabled ? "checked" : "");
  html += F("><small>Unchecked = ignore tank level and force mains</small></div>");
  html += F("<div id='tankCard'>");

  html += F("<div class='row switchline'><label>Water Source</label>");

  // Auto
  html += F("<label><input type='radio' name='waterMode' value='auto' ");
  if (!justUseTank && !justUseMains) html += F("checked");
  html += F("> Auto (Tank + City Water)</label>");

  // Only Tank
  html += F("<label><input type='radio' name='waterMode' value='tank' ");
  if (justUseTank) html += F("checked");
  html += F("> Only Tank</label>");

  // Only Mains
  html += F("<label><input type='radio' name='waterMode' value='mains' ");
  if (justUseMains) html += F("checked");
  html += F("> Only City Water</label>");

  html += F("<small>");
  html += "Tank/City Water switching (USE BACKFLOW PREVENTION!)";
  html += F("</small></div>");

  html += F("<div class='row'><label>Tank Low Threshold (%)</label>"
            "<input class='in-xs' type='number' min='0' max='100' name='tankThresh' value='");
  html += String(tankLowThresholdPct);
  html += F("'><small>Switch to city water if tank drops below this level</small></div>");

  #if defined(CONFIG_IDF_TARGET_ESP32)
    html += F("<div class='row'><label>Tank Level Sensor GPIO</label><input class='in-xs' type='number' min='32' max='39' name='tankLevelPin' value='");
    html += String(tankLevelPin); html += F("'><small>ADC1 pin (ESP32: GPIO32-39)</small></div>");
  #else
    html += F("<div class='row'><label>Tank Level Sensor GPIO</label><input class='in-xs' type='number' min='1' max='20' name='tankLevelPin' value='");
    html += String(tankLevelPin); html += F("'><small>ADC pin (ESP32-S3: GPIO1-20)</small></div>");
  #endif
  html += F("<div class='row'><label></label><a class='btn-alt' href='/tank'>Calibrate Tank</a></div>");
  html += F("</div>"); // end tankCard
  html += F("</div>");

  // Delays & Pause + thresholds
  html += F("<div class='card narrow' id='delays-card'><h3>Delays & Pause</h3><p class='card-intro'>Weather locks and pause controls live here, including cooldown timing and wind thresholds.</p>");
  html += F("<div class='cols2 panel-split'>");

  // Left column: toggles
  html += F("<div>");
  html += F("<div class='subhead'>Delay Toggles</div><hr class='hr'>");
  html += F("<div class='row switchline'><label>Rain Delay</label><input type='checkbox' name='rainDelay' ");
  html += (rainDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>Wind Delay</label><input type='checkbox' name='windCancelEnabled' ");
  html += (windDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>System Pause</label><input type='checkbox' name='pauseEnable' ");
  html += (systemPaused ? "checked" : ""); html += F("><small>Enable System Pause</small></div>");
  html += F("<div class='row' style='gap:8px;flex-wrap:wrap'>");
  html += F("<button class='btn' type='button' id='btn-pause-24'>Pause 24h</button>");
  html += F("<button class='btn' type='button' id='btn-pause-7d'>Pause 7d</button>");
  html += F("<button class='btn' type='button' id='btn-resume'>Unpause</button>");
  html += F("<div class='row'><label>Pause for (hours)</label><input class='in-sm' type='number' min='0' max='720' name='pauseHours' value='");
  time_t nowEp = time(nullptr);
    {
    uint32_t remain = (pauseUntilEpoch > nowEp && systemPaused) ? (pauseUntilEpoch - nowEp) : 0;
    html += String(remain/3600);
  }
  html += F("'></div>");
  html += F("</div>");
  html += F("</div>");

  // Right column: numeric thresholds / timers
  html += F("<div>");
  html += F("<div class='subhead'>Thresholds & Timers</div><hr class='hr'>");
  html += F("<div class='row'><label>Wind Threshold (m/s)</label><input class='in-sm' type='number' step='0.1' min='0' max='50' name='windSpeedThreshold' value='");
  html += String(windSpeedThreshold,1); html += F("'></div>");
  html += F("<div class='row'><label>After Rain Cooldown (hours)</label><input class='in-sm' type='number' min='0' max='720' name='rainCooldownHours' value='");
  html += String(rainCooldownMin / 60);
  html += F("'><small>Delay period after rain stops</small></div>");
  html += F("<div class='row'><label>Rain Threshold 24h (mm)</label><input class='in-sm' type='number' min='0' max='200' name='rainThreshold24h' value='");
  html += String(rainThreshold24h_mm);
  html += F("'><small>Cancel watering if > threshold</small></div>");
  html += F("</div>");

  html += F("</div>"); // end cols2
  html += F("</div>"); // end Delays card

  // Physical rain & forecast
  html += F("<div class='card narrow'><h3>Rain Sources</h3><p class='card-intro'>Choose whether forecast rain, a physical sensor, or both can stop irrigation.</p>");
  html += F("<div class='row switchline'><label>Disable Open-Meteo Rain</label><input type='checkbox' name='rainForecastDisabled' ");
  html += (!rainDelayFromForecastEnabled ? "checked" : ""); html += F("><small>Checked = ignore Open-Meteo rain</small></div>");
  html += F("<div class='row switchline'><label>Enable Rain Sensor</label><input type='checkbox' name='rainSensorEnabled' "); html += (rainSensorEnabled?"checked":""); html += F("></div>");
  html += F("<div class='row'><label>Rain Sensor GPIO</label><input class='in-xs' type='number' min='0' max='39' name='rainSensorPin' value='"); html += String(rainSensorPin); html += F("'><small>e.g. 27</small></div>");
  html += F("<div class='row switchline'><label>Invert Sensor</label><input type='checkbox' name='rainSensorInvert' "); html += (rainSensorInvert?"checked":""); html += F("><small>Use if board is NO</small></div>");
  html += F("</div>");

  // Weather
  html += F("<div class='card narrow' id='weather-card'><h3>Weather (Open-Meteo)</h3><p class='card-intro'>Enter the site coordinates and choose the forecast model used for delay logic and dashboard weather.</p>");
  html += F("<div class='row'><label>Open-Meteo</label><a class='btn-alt' id='setupMeteoLink' href='https://open-meteo.com/en/docs?latitude=");
  html += (isfinite(meteoLat) ? String(meteoLat, 6) : String("-35.107600"));
  html += F("&longitude=");
  html += (isfinite(meteoLon) ? String(meteoLon, 6) : String("138.557300"));
  html += F("' target='_blank' rel='noopener'>Open forecast page</a><small>Click to open Open-Meteo using the current coordinates.</small></div>");
  String modelSel = cleanMeteoModel(meteoModel);
  bool modelIsKnown = isKnownMeteoModel(modelSel);
  html += F("<div class='row'><label>Location Name</label><input class='in-wide' type='text' name='meteoLocation' value='"); html += meteoLocation; html += F("'><small>Optional label for UI/logs</small></div>");
  html += F("<div class='row'><label>Latitude</label><input class='in-med' type='text' name='meteoLat' value='"); html += latStr; html += F("'><small>e.g. -34.9285</small></div>");
  html += F("<div class='row'><label>Longitude</label><input class='in-med' type='text' name='meteoLon' value='"); html += lonStr; html += F("'><small>e.g. 138.6007</small></div>");
  html += F("<div class='row'><label>Model</label><select class='in-med' name='meteoModelSelect' id='meteoModelSelect'>");
  html += F("<option value='gfs'");           html += (modelSel == "gfs" ? " selected" : ""); html += F(">gfs</option>");
  html += F("<option value='icon'");          html += (modelSel == "icon" ? " selected" : ""); html += F(">icon</option>");
  html += F("<option value='ecmwf'");         html += (modelSel == "ecmwf" ? " selected" : ""); html += F(">ecmwf</option>");
  html += F("<option value='meteofrance'");   html += (modelSel == "meteofrance" ? " selected" : ""); html += F(">meteofrance</option>");
  html += F("<option value='jma'");           html += (modelSel == "jma" ? " selected" : ""); html += F(">jma</option>");
  html += F("<option value='cma'");           html += (modelSel == "cma" ? " selected" : ""); html += F(">cma</option>");
  html += F("<option value='gem'");           html += (modelSel == "gem" ? " selected" : ""); html += F(">gem</option>");
  html += F("<option value='icon_seamless'"); html += (modelSel == "icon_seamless" ? " selected" : ""); html += F(">icon_seamless</option>");
  html += F("<option value='icon_global'");   html += (modelSel == "icon_global" ? " selected" : ""); html += F(">icon_global</option>");
  html += F("<option value='icon_eu'");       html += (modelSel == "icon_eu" ? " selected" : ""); html += F(">icon_eu</option>");
  html += F("<option value='bom'");           html += (modelSel == "bom" ? " selected" : ""); html += F(">bom</option>");
  html += F("<option value='bom_access_global'"); html += (modelSel == "bom_access_global" ? " selected" : ""); html += F(">bom_access_global</option>");
  html += F("<option value='ukmo_seamless'"); html += (modelSel == "ukmo_seamless" ? " selected" : ""); html += F(">ukmo_seamless</option>");
  html += F("<option value='custom'");        html += (!modelIsKnown ? " selected" : ""); html += F(">custom</option>");
  html += F("</select><small>Open-Meteo model endpoint</small></div>");
  html += F("<div class='row' id='meteoModelCustomRow' style='display:");
  html += (modelIsKnown ? "none" : "flex");
  html += F("'><label>Custom Model</label><input class='in-med' type='text' name='meteoModelCustom' value='");
  html += (modelIsKnown ? "" : modelSel);
  html += F("'><small>Use an Open-Meteo model slug (e.g., gfs, icon, ecmwf)</small></div>");
  html += F("<div class='row helptext'><label></label><small>No API key required. Enter your coordinates for Open-Meteo.</small></div>");
  html += F("</div>");

  // Timezone
  html += F("<div class='card narrow' id='timezone-card'><h3>Timezone</h3><p class='card-intro'>Use a POSIX timezone string or a fixed UTC offset so schedules and weather line up with local time.</p>");

  // Mode selector - cleaner row, better labels
  html += F("<div class='row switchline'>");
  html += F("<label>Mode</label>");
  html += F("<div class='inline-options'>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='0' ");
  html += (tzMode==TZ_POSIX ? "checked" : "");
  html += F(">");
  html += F("<span>POSIX string</span>");
  html += F("</label>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='2' ");
  html += (tzMode==TZ_FIXED ? "checked" : "");
  html += F(">");
  html += F("<span>Fixed offset</span>");
  html += F("</label>");

  html += F("</div></div>"); // end row + inline-options

  // POSIX string input
  html += F("<div class='row'>");
  html += F("<label>Timezone</label>");
  html += F("<input class='in-wide' type='text' name='tzPosix' value='");
  html += tzPosix;
  html += F("' placeholder='ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3'>");
  html += F("</div>");

  // Help text on its own row so it wraps nicely on mobile
  html += F("<div class='row helptext'>");
  html += F("<label></label>");
  html += F("<small>Example POSIX string with DST: ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3</small>");
  html += F("</div>");

  // IANA input + themed select
  html += F("<div class='row'><label>Select Timezone</label>");
  html += F("<div style='flex:1;display:grid;gap:6px'>");
  html += F("<input type='hidden' name='tzIANA' value='");
  html += tzIANA;
  html += F("'>");
  html += F("<select class='in-med' id='tzIANASelect'><option value=''>Select from list</option></select>");
  html += F("</div>");
  html += F("</div>");

  html += F("<div class='row'><label>Fixed Offset (min)</label><input class='in-sm' type='number' name='tzFixed' value='");
  html += String(tzFixedOffsetMin);
  html += F("'><small>Minutes from UTC</small></div>");
  html += F("</div>"); // end Timezone card

  // Display / Auto backlight
  html += F("<div class='card narrow' id='display-card'><h3>Display</h3><p class='card-intro'>Choose the screen type, rotation, and light-sensor backlight behavior.</p>");
  html += F("<div class='row'><label>Display Type</label><select class='in-med' name='displayType'>");
  html += F("<option value='tft'");
  html += (displayUseTft ? " selected" : "");
  html += F(">TFT (ST7789)</option>");
  html += F("<option value='oled'");
  html += (!displayUseTft ? " selected" : "");
  html += F(">OLED (SSD1306)</option></select><small>Applied after reboot</small></div>");
  html += F("<div class='row'><label>TFT Rotation</label><select class='in-sm' name='tftRotation'>");
  html += F("<option value='0'"); html += (tftRotation == 0 ? " selected" : ""); html += F(">0</option>");
  html += F("<option value='1'"); html += (tftRotation == 1 ? " selected" : ""); html += F(">1</option>");
  html += F("<option value='2'"); html += (tftRotation == 2 ? " selected" : ""); html += F(">2</option>");
  html += F("<option value='3'"); html += (tftRotation == 3 ? " selected" : ""); html += F(">3</option>");
  html += F("</select><small>ST7789 screen orientation (0-3)</small></div>");
  html += F("<div class='row'><label>TFT Size</label><div class='field'><input class='in-xs' type='number' min='120' max='400' name='tftWidth' value='");
  html += String(tftPanelWidth);
  html += F("'><span>x</span><input class='in-xs' type='number' min='120' max='400' name='tftHeight' value='");
  html += String(tftPanelHeight);
  html += F("'></div><small>Saved panel size. Common ST7789 sizes: 170x320, 240x320, 240x240. Applied after reboot.</small></div>");
  html += F("<div class='row switchline'><label>Auto Backlight (LDR)</label><input type='checkbox' name='photoAuto' ");
  html += (photoAutoEnabled ? "checked" : "");
  html += F("><small>Turn off TFT when it is dark</small></div>");
  if (tftBlPin < 0) {
    html += F("<div class='row helptext'><label></label><small>No BL pin set; will use display sleep (backlight stays on).</small></div>");
  }
  int photoRaw = isValidPhotoPin(photoPin) ? analogRead(photoPin) : -1;
  html += F("<div class='row'><label>Photo Raw</label><div class='chip'>");
  if (photoRaw < 0) html += F("--");
  else html += String(photoRaw);
  html += F("</div><small>0-4095 ADC</small></div>");
  #if defined(CONFIG_IDF_TARGET_ESP32)
    html += F("<div class='row'><label>Photo GPIO</label><input class='in-xs' type='number' min='32' max='39' name='photoPin' value='");
    html += String(photoPin);
    html += F("'><small>ADC1 pin (ESP32: GPIO32-39)</small></div>");
  #else
    html += F("<div class='row'><label>Photo GPIO</label><input class='in-xs' type='number' min='1' max='40' name='photoPin' value='");
    html += String(photoPin);
    html += F("'><small>ESP32-S3 photo input range: GPIO1-40</small></div>");
  #endif
  html += F("<div class='row'><label>Dark Threshold</label><input class='in-sm' type='number' min='0' max='4095' name='photoThreshold' value='");
  html += String(photoThreshold);
  html += F("'><small>ADC raw value where screen turns off</small></div>");
  html += F("<div class='row switchline'><label>Invert Sensor</label><input type='checkbox' name='photoInvert' ");
  html += (photoInvert ? "checked" : "");
  html += F("><small>Enable if your LDR reads higher when dark</small></div>");
  html += F("</div>");

  // SPI (TFT) config
  html += F("<div class='card narrow' id='advanced-card'><details class='collapse'><summary>SPI (TFT)</summary><div class='collapse-body'><p class='card-intro'>Advanced screen pin mapping and backlight tools. Change these only if your display wiring differs from the defaults.</p>");
  html += F("<div class='row'><label>TFT Size</label><div class='chip'>");
  html += String(tftPanelWidth); html += "x"; html += String(tftPanelHeight);
  html += F("</div><small>Saved display geometry</small></div>");
  html += F("<datalist id='tftPins'>");
  for (int p = 1; p <= uiMaxGpio; ++p) {
    if (!isUnsafeTftPin(p)) { html += F("<option value='"); html += String(p); html += F("'>"); }
  }
  html += F("</datalist>");
  html += F("<datalist id='tftPinsOrNone'><option value='-1'>");
  for (int p = 1; p <= uiMaxGpio; ++p) {
    if (!isUnsafeTftPin(p)) { html += F("<option value='"); html += String(p); html += F("'>"); }
  }
  html += F("</datalist>");
  html += F("<div class='row'><label>SCK</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftSclk' value='");
  html += String(tftSclkPin); html += F("'></div>");
  html += F("<div class='row'><label>MOSI</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftMosi' value='");
  html += String(tftMosiPin); html += F("'></div>");
  html += F("<div class='row'><label>CS</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftCs' value='");
  html += String(tftCsPin); html += F("'></div>");
  html += F("<div class='row'><label>DC</label><input class='in-xs' type='number' min='1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPins' name='tftDc' value='");
  html += String(tftDcPin); html += F("'></div>");
  html += F("<div class='row'><label>RST</label><input class='in-xs' type='number' min='-1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPinsOrNone' name='tftRst' value='");
  html += String(tftRstPin); html += F("'><small>-1 = not used</small></div>");
  html += F("<div class='row'><label>BL</label><input class='in-xs' type='number' min='-1' max='");
  html += String(uiMaxGpio);
  html += F("' list='tftPinsOrNone' name='tftBl' value='");
  html += String(tftBlPin); html += F("'><small>-1 = not used</small></div>");
  html += F("<div class='row'><label>LCD Brightness (%)</label><input class='in-xs' type='number' id='tftLevel' min='0' max='100' value='100'><button class='btn' type='button' id='btn-tft-bright'>Set</button></div>");
  html += F("<div class='row'><label>Self-Test</label><div class='field'>");
  html += F("<button class='btn-alt' type='button' id='tftSelfTestBtn'>TFT Self-Test</button>");
  html += F("</div></div>");
  html += F("<div class='row helptext'><label></label><small id='tftStatusLine'>Backlight: --</small></div>");
  html += F("<div class='row helptext'><label></label><small>Changing TFT pins requires reboot. ");
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    html += F("ESP32-S3: avoid 0/3/45/46 (strapping) and 26-37 (flash/PSRAM, module-dependent). GPIO19/20 are allowed but will take over the USB pins.");
  #else
    html += F("Use output-capable GPIO 1-");
    html += String(uiMaxGpio);
    html += F(" excluding 19/20 (USB), 0/45/46 (strapping), 9-14 & 35-38 (flash/PSRAM).");
  #endif
  html += F("</small></div>");
  html += F("</div></details></div>");

  // I2C config
  html += F("<div class='card narrow'><details class='collapse'><summary>I2C (PCF8574)</summary><div class='collapse-body'><p class='card-intro'>Expander bus pins for the relay hardware. These normally only need changing during custom wiring.</p>");
  html += F("<div class='row'><label>SDA</label><input class='in-xs' type='number' min='0' max='48' name='i2cSda' value='");
  html += String(i2cSdaPin); html += F("'></div>");
  html += F("<div class='row'><label>SCL</label><input class='in-xs' type='number' min='0' max='48' name='i2cScl' value='");
  html += String(i2cSclPin); html += F("'></div>");
  html += F("<div class='row helptext'><label></label><small>Changing I2C pins requires reboot. Avoid strapping pins and SPI flash/PSRAM pins. 4/15 for KC868 </small></div>");
  html += F("</div></details></div>");

  // GPIO fallback pins
  html += F("<div class='card narrow'><details class='collapse' ");
  html += (useGpioFallback ? "open" : "");
  html += F("><summary>GPIO Fallback (if I2C relays not found)</summary><div class='collapse-body'><div class='grid'>");
  for (uint8_t i=0;i<MAX_ZONES;i++){
    html += F("<div class='row switchline'><label>Zone "); html += String(i+1);
    html += F(" Pin</label><input class='in-xs' type='number' min='-1' max='39' name='zonePin"); html += String(i);
    html += F("' value='"); html += String(zonePins[i]); html += F("'>");
    html += F("<label class='chip'><input type='checkbox' name='zonePinLow"); html += String(i); html += F("' ");
    html += (zoneGpioActiveLow[i] ? "checked" : "");
    html += F("><span>LOW = ON</span></label></div>");
  }
  html += F("<div class='row'><label></label><small>Use -1 to leave a zone unassigned. Zones above the PCF channels use GPIO pins when set.</small></div>");
  html += F("<div class='row switchline'><label>City Water Relay Pin</label><input class='in-xs' type='number' min='0' max='39' name='mainsPin' value='");
  html += String(mainsPin); html += F("'><label class='chip'><input type='checkbox' name='mainsPinLow' ");
  html += (mainsGpioActiveLow ? "checked" : "");
  html += F("><span>LOW = ON</span></label><small>Relay for city water in (use check/backflow prevention device)</small></div>");
  html += F("<div class='row switchline'><label>Tank Relay Pin</label><input class='in-xs' type='number' min='0' max='39' name='tankPin' value='");
  html += String(tankPin); html += F("'><label class='chip'><input type='checkbox' name='tankPinLow' ");
  html += (tankGpioActiveLow ? "checked" : "");
  html += F("><span>LOW = ON</span></label><small>Pump on low pressure (Relay on) and off at a higher pressure (Relay off).</small></div>");
  html += F("<div class='row'><label>Legacy Default</label><div class='chip'>");
  html += (gpioActiveLow ? "LOW = ON" : "HIGH = ON");
  html += F("</div><small>Used only when loading older saved configs that do not have per-pin polarity values yet.</small></div>");

  html += F("</div>");
  html += F("<div class='row' style='justify-content:flex-end;gap:8px'>");
  html += F("<button class='btn' type='submit'>Save</button>");
  html += F("</div>");
  html += F("</div></details></div>");

  // Manual buttons
  html += F("<div class='card narrow'><details class='collapse'><summary>Manual Buttons</summary><div class='collapse-body'><p class='card-intro'>Optional physical buttons for cycling a zone and toggling it on or off without using the web UI.</p>");
  html += F("<div class='row switchline'><label>Select Button Pin</label><input class='in-xs' type='number' min='-1' max='39' name='manualSelectPin' value='");
  html += String(manualSelectPin);
  html += F("'><small>-1 to disable. Uses INPUT_PULLUP; press = LOW.</small></div>");
  html += F("<div class='row switchline'><label>Start/Stop Button Pin</label><input class='in-xs' type='number' min='-1' max='39' name='manualStartPin' value='");
  html += String(manualStartPin);
  html += F("'><small>Toggles the selected zone on/off.</small></div>");
  html += F("<div class='row'><label>Selected Zone</label><div class='sub'>");
  html += zoneNames[manualSelectedZone % zonesCount];
  html += F(" (cycles with Select button)</div></div>");
  html += F("</div></details></div>");

  

  // MQTT
  html += F("<div class='card narrow' id='mqtt-card'><details class='collapse'><summary>MQTT (Home Assistant)</summary><div class='collapse-body'><p class='card-intro'>Publish controller status and accept simple commands from Home Assistant or another MQTT client.</p>");
  html += F("<div class='row switchline'><label>Enable MQTT</label><input type='checkbox' name='mqttEnabled' "); html += (mqttEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row'><label>Broker Host</label><input class='in-wide' type='text' name='mqttBroker' value='"); html += mqttBroker; html += F("'></div>");
  html += F("<div class='row'><label>Port</label><input class='in-xs' type='number' name='mqttPort' value='"); html += String(mqttPort); html += F("'></div>");
  html += F("<div class='row'><label>User</label><input class='in-med' type='text' name='mqttUser' value='"); html += mqttUser; html += F("'></div>");
  html += F("<div class='row'><label>Password</label><input class='in-med' type='text' name='mqttPass' value='"); html += mqttPass; html += F("'></div>");
  html += F("<div class='row'><label>Base Topic</label><input class='in-med' type='text' name='mqttBase' value='"); html += mqttBase; html += F("'><small>e.g. espirrigation</small></div>");
  html += F("</div></details></div>");

  html += F("</form>");

  // quick-actions + timezone JS
  html += F("<script>");
  html += F("async function post(path, body){try{await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});}catch(e){console.error(e)}}");
  html += F("const g=id=>document.getElementById(id);");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-alt').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("g('btn-toggle-backlight')?.addEventListener('click',()=>post('/toggleBacklight','x=1'));");
  html += F("g('btn-pause-24')?.addEventListener('click',()=>post('/pause','sec=86400'));");
  html += F("g('btn-pause-7d')?.addEventListener('click',()=>post('/pause','sec='+(7*86400)));");
  html += F("g('btn-resume')?.addEventListener('click',()=>post('/resume','x=1'));");
  html += F("g('btn-tft-bright')?.addEventListener('click',async()=>{const v=g('tftLevel'); if(!v) return; let n=parseInt(v.value||'100'); if(isNaN(n)) n=100; n=Math.min(100,Math.max(0,n)); v.value=n; try{await post('/tft_brightness','level='+n);}catch(e){console.error(e)}});");
  html += F("g('tftSelfTestBtn')?.addEventListener('click',async()=>{");
  html += F("  if(!confirm('Run TFT self-test now?')) return;");
  html += F("  try{const r=await fetch('/tft_selftest'); const t=await r.text(); alert(t||'TFT self-test done');}catch(e){alert('TFT self-test failed');}");
  html += F("  loadTftStatus();");
  html += F("});");

  html += F("async function loadTftStatus(){");
  html += F("  const el=g('tftStatusLine'); if(!el) return;");
  html += F("  try{const r=await fetch('/status'); const st=await r.json();");
  html += F("    const pin=st.tftBlPin; const pwm=!!st.tftPwm; const on=!!st.tftBlOn; const disp=!!st.tftDisplayOn;");
  html += F("    const pct=(typeof st.tftBrightnessPct==='number')?st.tftBrightnessPct:0;");
  html += F("    if(typeof pin==='number' && pin>=0){");
  html += F("      const mode=pwm?'PWM':'ON/OFF'; el.textContent='Backlight: '+mode+' '+pct+'% (pin '+pin+') '+(on?'on':'off');");
  html += F("    } else {");
  html += F("      el.textContent='Backlight: display '+(disp?'on':'off');");
  html += F("    }");
  html += F("  }catch(e){ el.textContent='Backlight: --'; }");
  html += F("}");

  // === Timezone loading from Nayarsystems posix_tz_db with fallback ===
  html += F("const TZ_DB_URL='https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.json';");
  html += F("const tzInput=document.getElementsByName('tzIANA')[0]||null;");
  html += F("const tzPosixInput=document.getElementsByName('tzPosix')[0]||null;");
  html += F("const tzSel=g('tzIANASelect');");

  // Hard-coded fallback zones used if fetch fails
  html += F("const FALLBACK_ZONES={");
  html += F("'Australia/Adelaide':'ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3',");
  html += F("'Australia/Sydney':'AEST-10AEDT-11,M10.1.0/2,M4.1.0/3',");
  html += F("'UTC':'UTC0'");
  html += F("};");

  // Helper to populate the select + sync inputs from a map of { IANA: POSIX }
  html += F("function buildTzOptions(zones){");
  html += F(" if(!tzSel||!tzInput) return;");
  html += F(" tzSel.innerHTML='<option value=\"\">Select from list</option>';");

  html += F(" const names=Object.keys(zones).sort();");
  html += F(" names.forEach(name=>{");
  html += F("   const opt=document.createElement('option');");
  html += F("   opt.value=name;");
  html += F("   opt.textContent=name;");
  html += F("   if(tzInput.value===name) opt.selected=true;");
  html += F("   tzSel.appendChild(opt);");
  html += F(" });");

  // If tzInput already has a valid value, sync POSIX
  html += F(" if(tzInput.value && zones[tzInput.value] && tzPosixInput){");
  html += F("   tzPosixInput.value=zones[tzInput.value];");
  html += F(" }");

  // Try to guess browser zone if empty and available
  html += F(" if(!tzInput.value && window.Intl && Intl.DateTimeFormat){");
  html += F("   const guess=Intl.DateTimeFormat().resolvedOptions().timeZone;");
  html += F("   if(guess && zones[guess]){");
  html += F("     tzInput.value=guess;");
  html += F("     if(tzPosixInput) tzPosixInput.value=zones[guess];");
  html += F("     for(const opt of tzSel.options){");
  html += F("       if(opt.value===guess){opt.selected=true;break;}");
  html += F("     }");
  html += F("   }");
  html += F(" }");

  // When user picks a zone, update IANA + POSIX fields
  html += F(" tzSel.addEventListener('change',()=>{");
  html += F("   const val=tzSel.value;");
  html += F("   if(!val) return;");
  html += F("   tzInput.value=val;");
  html += F("   if(tzPosixInput && zones[val]) tzPosixInput.value=zones[val];");
  html += F(" });");
  html += F("}"); // end buildTzOptions

  // Tank card always shown now (no toggle)

  html += F("async function loadTimezones(){");
  html += F(" if(!tzSel||!tzInput) return;");

  html += F(" tzSel.innerHTML='<option value=\"\">Loading...</option>';");

  html += F(" try{");
  html += F("   const res=await fetch(TZ_DB_URL);");
  html += F("   if(!res.ok) throw new Error('HTTP '+res.status);");
  html += F("   const zones=await res.json();");
  html += F("   buildTzOptions(zones);");
  html += F(" }catch(e){");
  html += F("   console.error('tz load failed, using fallback',e);");
  html += F("   buildTzOptions(FALLBACK_ZONES);");
  html += F(" }");
  html += F("}");

  html += F("function initThemeToggle(){");
  html += F("  let saved=localStorage.getItem('theme');");
  html += F("  if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';}");
  html += F("  document.documentElement.setAttribute('data-theme', saved);");
  html += F("  const cb=document.getElementById('themeToggle');");
  html += F("  if(cb){cb.checked=(saved==='dark');cb.addEventListener('change',()=>{");
  html += F("    const next=cb.checked?'dark':'light';document.documentElement.setAttribute('data-theme', next);localStorage.setItem('theme', next);");
  html += F("  });}");
  html += F("}");
  html += F("initThemeToggle();");
  html += F("loadTimezones();");
  html += F("loadTftStatus();");
  // === END TZ CODE ===

  html += F("</script>");

  html += F("</div></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Schedule POST (per-zone card or full form) ----------
void handleSubmit() {
  HttpScope _scope;
  auto clampInt = [](int v, int lo, int hi) -> int {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  };

  // If onlyZone is present -> only update that zone from fields in this form
  if (server.hasArg("onlyZone")) {
    int z = server.arg("onlyZone").toInt();
    if (z >= 0 && z < (int)MAX_ZONES) {
      // Zone name
      if (server.hasArg("zoneName"+String(z))) {
        String nm=cleanName(server.arg("zoneName"+String(z)));
        if (nm.length()) zoneNames[z]=nm;
      }
      // Times / duration
      if (server.hasArg("startHour"+String(z)))  startHour[z]  = clampInt(server.arg("startHour"+String(z)).toInt(), 0, 23);
      if (server.hasArg("startMin"+String(z)))   startMin[z]   = clampInt(server.arg("startMin"+String(z)).toInt(), 0, 59);
      if (server.hasArg("startHour2"+String(z))) startHour2[z] = clampInt(server.arg("startHour2"+String(z)).toInt(), 0, 23);
      if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = clampInt(server.arg("startMin2"+String(z)).toInt(), 0, 59);
      if (server.hasArg("durationMin"+String(z))) durationMin[z]=clampInt(server.arg("durationMin"+String(z)).toInt(), 0, 600);
      if (server.hasArg("durationSec"+String(z))) durationSec[z]=clampInt(server.arg("durationSec"+String(z)).toInt(), 0, 59);
      if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=clampInt(server.arg("duration2Min"+String(z)).toInt(), 0, 600);
      if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=clampInt(server.arg("duration2Sec"+String(z)).toInt(), 0, 59);
      enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
      // Days
      for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));

      saveSchedule(); saveConfig();
      server.sendHeader("Location","/",true); server.send(302,"text/plain","");
      return;
    }
  }

  // Otherwise: legacy full update of all zones (expects all fields present)
  for (int z=0; z<(int)MAX_ZONES; z++) {
    for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));
    if (server.hasArg("zoneName"+String(z))) {
      String nm=cleanName(server.arg("zoneName"+String(z)));
      if (nm.length()) zoneNames[z]=nm;
    }
    if (server.hasArg("startHour"+String(z)))  startHour[z]  = clampInt(server.arg("startHour"+String(z)).toInt(), 0, 23);
    if (server.hasArg("startMin"+String(z)))   startMin[z]   = clampInt(server.arg("startMin"+String(z)).toInt(), 0, 59);
    if (server.hasArg("startHour2"+String(z))) startHour2[z] = clampInt(server.arg("startHour2"+String(z)).toInt(), 0, 23);
    if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = clampInt(server.arg("startMin2"+String(z)).toInt(), 0, 59);
    if (server.hasArg("durationMin"+String(z))) durationMin[z]=clampInt(server.arg("durationMin"+String(z)).toInt(), 0, 600);
    if (server.hasArg("durationSec"+String(z))) durationSec[z]=clampInt(server.arg("durationSec"+String(z)).toInt(), 0, 59);
    if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=clampInt(server.arg("duration2Min"+String(z)).toInt(), 0, 600);
    if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=clampInt(server.arg("duration2Sec"+String(z)).toInt(), 0, 59);
    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
  }
  saveSchedule(); saveConfig();
  server.sendHeader("Location","/",true); server.send(302,"text/plain","");
}

// ---------- Event Log Page ----------
void handleLogPage() {
  HttpScope _scope;
  File f = LittleFS.open("/events.csv","r");
  if (!f) {
    server.send(404,"text/plain","No event log");
    return;
  }

  String html; html.reserve(18000);
  html += F("<!doctype html><html lang='en' data-theme='light'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<meta name='theme-color' content='#145b63'><meta name='color-scheme' content='light dark'>");
  html += F("<title>ESP32 Irrigation Events</title>");
  html += F("<style>");
  html += F("@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@500;700&family=Sora:wght@400;600;700;800&display=swap');");
  html += F(":root[data-theme='light']{--bg:#edf3ef;--bg2:#f8fbf8;--glass:rgba(255,255,255,.58);--glass-brd:rgba(122,149,140,.32);--panel:#ffffff;--line:#d3dfd9;");
  html += F("--card:#ffffff;--ink:#14232b;--muted:#5f736f;--primary:#1f8a70;--primary-2:#145b63;--ok:#2f9e44;--warn:#c97a1a;--bad:#d9485f;");
  html += F("--chip:#edf7f2;--chip-brd:#c9ddd4;--ring:#dcebe5;--ring2:#b4d4cb;--shadow:0 18px 40px rgba(20,47,45,.14)}");
  html += F(":root[data-theme='dark']{--bg:#081315;--bg2:#0d1d21;--glass:rgba(12,28,31,.62);--glass-brd:rgba(84,123,118,.3);--panel:#0f1e22;--line:#214147;");
  html += F("--card:#0f1e22;--ink:#e6f0ec;--muted:#9ab4ad;--primary:#46c6a3;--primary-2:#1f8a86;--ok:#52c266;--warn:#f0ae4d;--bad:#ff6b7d;");
  html += F("--chip:#10272b;--chip-brd:#24444a;--ring:#123036;--ring2:#1d5152;--shadow:0 18px 40px rgba(0,0,0,.42)}");
  html += F("*{box-sizing:border-box}html,body{margin:0;padding:0}");
  html += F("html{scroll-behavior:smooth}");
  html += F("body{background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--ink);font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;line-height:1.35;-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F("a{text-decoration:none;color:inherit}");
  html += F(".wrap{max-width:1280px;margin:20px auto;padding:0 16px}");
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 12px;background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1280px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff;flex-wrap:wrap}");
  html += F(".brand{display:flex;align-items:center;gap:8px;font-weight:800;letter-spacing:.2px;font-size:1.12rem}");
  html += F(".brand-copy{display:flex;flex-direction:column;gap:2px}");
  html += F(".brand-title{text-transform:uppercase;letter-spacing:.9px;font-size:1rem;line-height:1}");
  html += F(".brand-sub{font-size:.74rem;font-weight:650;color:rgba(255,255,255,.8);letter-spacing:.08em;text-transform:uppercase}");
  html += F(".dot{width:12px;height:12px;border-radius:999px;background:#84ffb5;box-shadow:0 0 14px #84ffb5}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650;font-size:.88rem}");
  html += F(".pill{display:inline-flex;align-items:center;gap:6px;background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:7px 12px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;border-radius:10px;padding:8px 14px;font-weight:700;cursor:pointer;font-size:.92rem}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:20px;box-shadow:var(--shadow)}");
  html += F(".section{padding:18px}");
  html += F(".hero-shell{position:relative;overflow:hidden;padding:clamp(18px,3vw,30px);margin:18px 0}");
  html += F(".hero-shell::before,.hero-shell::after{content:'';position:absolute;border-radius:999px;pointer-events:none;filter:blur(8px)}");
  html += F(".hero-shell::before{width:220px;height:220px;right:-50px;top:-40px;background:radial-gradient(circle,rgba(233,173,73,.2),transparent 68%)}");
  html += F(".hero-shell::after{width:260px;height:260px;left:-90px;bottom:-100px;background:radial-gradient(circle,rgba(70,198,163,.2),transparent 68%)}");
  html += F(".hero-grid{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(320px,.85fr);gap:18px;align-items:stretch}");
  html += F(".hero-copy,.hero-mini-grid{position:relative;z-index:1}");
  html += F(".hero-copy{display:flex;flex-direction:column;justify-content:center;gap:14px}");
  html += F(".hero-kicker{text-transform:uppercase;letter-spacing:.22em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".hero-title{margin:0;font-size:clamp(1.95rem,4vw,3rem);line-height:1.02;max-width:11ch}");
  html += F(".hero-text{margin:0;max-width:60ch;color:var(--muted);font-size:1rem}");
  html += F(".hero-actions,.toolbar{display:flex;gap:10px;flex-wrap:wrap}");
  html += F(".hero-mini-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}");
  html += F(".hero-mini{min-height:132px;padding:16px;border-radius:18px;border:1px solid var(--glass-brd);background:linear-gradient(180deg,rgba(255,255,255,.58),rgba(255,255,255,.14));box-shadow:0 14px 30px rgba(19,33,68,.12);display:flex;flex-direction:column;justify-content:space-between;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px)}");
  html += F(".hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(31,138,112,.2),rgba(82,194,102,.1))}");
  html += F(".hero-mini-label{text-transform:uppercase;letter-spacing:.16em;font-size:.74rem;font-weight:800;color:var(--muted)}");
  html += F(".hero-mini-value{font-size:1.72rem;font-weight:800;line-height:1.02;font-variant-numeric:tabular-nums}");
  html += F(".hero-mini-sub{color:var(--muted);font-size:.92rem}");
  html += F(".section-head{display:flex;align-items:flex-end;justify-content:space-between;gap:14px;max-width:1280px;margin:0 auto 14px;padding:0 2px;flex-wrap:wrap}");
  html += F(".section-kicker{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;font-weight:800;color:var(--primary)}");
  html += F(".section-head h2{margin:4px 0 0;font-size:1.35rem;line-height:1.08}");
  html += F(".section-note{margin:0;max-width:42ch;color:var(--muted);font-size:.92rem}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:20px;box-shadow:var(--shadow);padding:18px;transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 16px 34px rgba(0,0,0,.18)}");
  html += F(".btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;background:linear-gradient(180deg,var(--primary),var(--primary-2));color:#fff;border:1px solid rgba(0,0,0,.08);border-radius:13px;padding:11px 16px;font-weight:800;cursor:pointer;box-shadow:0 8px 20px rgba(0,0,0,.22);font-size:1rem}");
  html += F(".btn-secondary{background:transparent;color:var(--ink);border:1px solid var(--line);box-shadow:none}");
  html += F(".btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border-color:rgba(185,28,28,.6)}");
  html += F(".btn-warn{background:linear-gradient(180deg,#d97706,#92400e);border-color:rgba(146,64,14,.55)}");
  html += F(".btn,.btn-ghost,.pill{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease;touch-action:manipulation;position:relative;overflow:hidden}");
  html += F(".btn:hover{filter:brightness(1.05)}");
  html += F(".btn:active:not(:disabled),.btn-ghost:active,.pill:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn:focus-visible,.btn-ghost:focus-visible,.pill:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");
  html += F(".toolbar{margin:14px 0 0}");
  html += F(".toolbar form{display:inline-flex;margin:0}");
  html += F(".table-wrap{overflow:auto;border-radius:18px;border:1px solid var(--glass-brd);background:linear-gradient(180deg,rgba(255,255,255,.18),rgba(255,255,255,.05));box-shadow:var(--shadow)}");
  html += F("table{width:100%;border-collapse:separate;border-spacing:0;min-width:820px}");
  html += F("th,td{padding:12px 14px;border-bottom:1px solid var(--line);font-size:.94rem;text-align:left;white-space:nowrap;vertical-align:top}");
  html += F("th{position:sticky;top:0;z-index:1;background:rgba(20,91,99,.92);color:#fff;text-transform:uppercase;letter-spacing:.08em;font-size:.75rem}");
  html += F("tbody tr{background:rgba(255,255,255,.04)}");
  html += F("tbody tr:nth-child(even){background:rgba(255,255,255,.02)}");
  html += F("tbody tr:hover{background:rgba(31,138,112,.09)}");
  html += F("td:first-child{font-family:'JetBrains Mono','Consolas',monospace;font-size:.82rem}");
  html += F("td:last-child{white-space:normal;min-width:260px;color:var(--muted)}");
  html += F(".event-chip{display:inline-flex;align-items:center;justify-content:center;min-width:88px;padding:6px 10px;border-radius:999px;border:1px solid var(--chip-brd);font-size:.78rem;font-weight:800;letter-spacing:.08em;text-transform:uppercase}");
  html += F(".event-chip.start{background:rgba(34,197,94,.12);border-color:rgba(34,197,94,.3);color:var(--ok)}");
  html += F(".event-chip.stopped{background:rgba(217,72,95,.12);border-color:rgba(217,72,95,.32);color:var(--bad)}");
  html += F(".event-chip.cancelled{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.34);color:var(--warn)}");
  html += F(".event-chip.queued{background:rgba(70,198,163,.12);border-color:rgba(70,198,163,.32);color:var(--primary)}");
  html += F(".muted{color:var(--muted)}");
  html += F(".empty-state{padding:28px 22px;text-align:center;color:var(--muted)}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F("html[data-theme='dark'] .hero-mini{background:linear-gradient(180deg,rgba(15,30,34,.96),rgba(15,30,34,.9));box-shadow:0 14px 30px rgba(0,0,0,.22)}");
  html += F("html[data-theme='dark'] .hero-mini.hero-mini-strong{background:linear-gradient(135deg,rgba(31,138,112,.18),rgba(16,39,43,.94))}");
  html += F("html[data-theme='dark'] .table-wrap{background:linear-gradient(180deg,rgba(12,28,31,.9),rgba(12,28,31,.72))}");
  html += F("html[data-theme='dark'] tbody tr{background:rgba(255,255,255,.015)}");
  html += F("html[data-theme='light'] th{background:rgba(20,91,99,.94)}");
  html += F("html[data-theme='light'] tbody tr{background:rgba(255,255,255,.8)}");
  html += F("html[data-theme='light'] tbody tr:nth-child(even){background:rgba(248,251,248,.92)}");
  html += F("@media(max-width:980px){.hero-grid{grid-template-columns:1fr}.hero-mini-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}");
  html += F("@media(max-width:720px){.wrap{padding:0 12px}.nav{padding:10px}.hero-mini-grid{grid-template-columns:1fr}.toolbar,.hero-actions{flex-direction:column}.toolbar form{display:flex}.btn,.btn-ghost{width:100%}.section{padding:16px}th,td{padding:10px 12px}}");
  html += F("@media (prefers-reduced-motion: reduce){*{animation:none!important;transition:none!important}}");
  html += F("</style></head><body>");

  int eventCount = 0;
  int startCount = 0;
  int stopCount = 0;
  int weatherDelayCount = 0;
  String latestTs = "-";

  String eventRows;
  eventRows.reserve(9000);

  while (f.available()) {
    String line=f.readStringUntil('\n');
    if (line.length()<5) continue;

    int i1=line.indexOf(','), i2=line.indexOf(',',i1+1), i3=line.indexOf(',',i2+1), i4=line.indexOf(',',i3+1);
    int i5=line.indexOf(',',i4+1), i6=line.indexOf(',',i5+1), i7=line.indexOf(',',i6+1), i8=line.indexOf(',',i7+1), i9=line.indexOf(',',i8+1);

    String ts   = line.substring(0,i1);
    String zone = line.substring(i1+1,i2);
    String ev   = line.substring(i2+1,i3);
    String src  = line.substring(i3+1,i4);
    const bool showEvent =
      (ev == "START" || ev == "STOPPED" ||
       (ev == "CANCELLED" && src == "RAIN") ||
       (ev == "QUEUED" && src == "WIND"));
    if (!showEvent) continue;
    String rd   = line.substring(i4+1,i5);

    String temp =(i6>i5)?line.substring(i5+1,i6):"";
    String hum  =(i7>i6)?line.substring(i6+1,i7):"";
    String wind =(i8>i7)?line.substring(i7+1,i8):"";
    String cond =(i9>i8)?line.substring(i8+1,i9):"";
    String city =(i9>=0)?line.substring(i9+1):"";

    String details = (temp.length()
                      ? ("T="+temp+"C, H="+hum+"%, W="+wind+"m/s, "+cond+" @ "+city)
                      : "n/a");

    eventCount++;
    latestTs = ts;
    if (ev == "START") startCount++;
    else if (ev == "STOPPED") stopCount++;
    else weatherDelayCount++;

    String row;
    row.reserve(320);
    row += F("<tr><td>"); row += ts;
    row += F("</td><td>"); row += zone;
    row += F("</td><td><span class='event-chip ");
    if (ev == "START") row += F("start'>Start");
    else if (ev == "STOPPED") row += F("stopped'>Stopped");
    else if (ev == "CANCELLED") row += F("cancelled'>Rain Delay");
    else row += F("queued'>Wind Delay");
    row += F("</span>");
    row += F("</td><td>"); row += src;
    row += F("</td><td>"); row += rd;
    row += F("</td><td>"); row += details; row += F("</td></tr>");

    // Keep newest events at the top of the table.
    eventRows = row + eventRows;
  }
  f.close();
  html += F("<nav class='nav'><div class='in'><div class='brand'><span class='dot'></span><div class='brand-copy'><span class='brand-title'>ESP32 Irrigation</span><span class='brand-sub'>Event History</span></div></div><div class='meta'><span class='pill'>");
  html += String(eventCount);
  html += F(" filtered events</span><button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button></div></div></nav>");
  html += F("<div class='wrap'>");
  html += F("<section class='glass hero-shell'><div class='hero-grid'><div class='hero-copy'><div class='hero-kicker'>System History</div><h1 class='hero-title'>Irrigation event log</h1><p class='hero-text'>Review run starts, stops, and scheduled weather delays when rain or wind prevents a zone from starting on time.</p><div class='hero-actions'><a class='btn' href='/'>Home</a><a class='btn btn-secondary' href='/setup'>Setup</a><a class='btn btn-secondary' href='/download/events.csv'>Download CSV</a></div></div>");
  html += F("<div class='hero-mini-grid'><div class='hero-mini hero-mini-strong'><div class='hero-mini-label'>Latest Event</div><div class='hero-mini-value'>");
  html += latestTs;
  html += F("</div><div class='hero-mini-sub'>Most recent matching log timestamp</div></div>");
  html += F("<div class='hero-mini'><div class='hero-mini-label'>Start Events</div><div class='hero-mini-value'>");
  html += String(startCount);
  html += F("</div><div class='hero-mini-sub'>Zone starts recorded in this filtered view</div></div>");
  html += F("<div class='hero-mini'><div class='hero-mini-label'>Stopped Events</div><div class='hero-mini-value'>");
  html += String(stopCount);
  html += F("</div><div class='hero-mini-sub'>Completed or interrupted watering runs</div></div>");
  html += F("<div class='hero-mini'><div class='hero-mini-label'>Weather Delays</div><div class='hero-mini-value'>");
  html += String(weatherDelayCount);
  html += F("</div><div class='hero-mini-sub'>Scheduled starts blocked by rain or queued by wind</div></div></div></div></section>");
  html += F("<div class='section-head'><div><div class='section-kicker'>Audit Trail</div><h2>Recent events</h2></div><p class='section-note'>Newest entries stay at the top, including scheduled rain and wind delay events alongside run starts and stops.</p></div>");
  html += F("<section class='card'><div class='toolbar'><form method='POST' action='/clearevents'><button class='btn btn-danger' type='submit'>Clear Events</button></form><form method='POST' action='/stopall'><button class='btn btn-warn' type='submit'>Stop All</button></form></div><div class='table-wrap'><table><thead><tr>");
  html += F("<th>Time</th><th>Zone</th><th>Event</th><th>Source</th><th>Rain Delay</th><th>Details</th></tr></thead><tbody>");
  if (eventCount == 0) {
    html += F("<tr><td colspan='6' class='empty-state'>No run or weather-delay entries are available in the current event log.</td></tr>");
  } else {
    html += eventRows;
  }
  html += F("</tbody></table></div></section></div>");
  html += F("<script>");
  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');if(saved!=='dark'&&saved!=='light'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("document.getElementById('themeBtn')?.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-ghost').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("</script></body></html>");
  server.send(200,"text/html",html);
}

// ---------- Tank Calibration Page ----------
void handleTankCalibration() {
  HttpScope _scope;
  int raw = isValidAdcPin(tankLevelPin) ? analogRead(tankLevelPin) : -1;
  int pct=tankPercent();

  String html; html.reserve(2000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Tank Calibration</title>");
  html += F("<style>@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@500;700&family=Sora:wght@400;600;700;800&display=swap');");
  html += F("body{font-family:'Sora','Avenir Next','Trebuchet MS',sans-serif;background:radial-gradient(900px 500px at 5% -10%,rgba(43,123,228,.2),transparent),#0f1522;color:#e8eef6;margin:0;font-size:15px}");
  html += F(".wrap{max-width:620px;margin:34px auto;padding:0 16px}.card{background:#0b1220;border:1px solid #22314f;border-radius:16px;padding:20px 16px;box-shadow:0 18px 34px rgba(0,0,0,.28);position:relative;overflow:hidden}");
  html += F(".card:before{content:'';position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,#2b7be4,#22c55e)}");
  html += F("h2{margin:0 0 12px 0;font-size:1.25em;letter-spacing:.8px;text-transform:uppercase}");
  html += F("p b:first-child{font-family:'JetBrains Mono','Consolas',monospace}");
  html += F(".btn{background:linear-gradient(180deg,#2b7be4,#1f62c8);color:#fff;border:1px solid rgba(0,0,0,.18);border-radius:12px;padding:10px 16px;font-weight:700;cursor:pointer;font-size:.95rem}.row{display:flex;gap:12px;justify-content:space-between;margin-top:12px;flex-wrap:wrap}");
  html += F(".btn{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F("@media(max-width:600px){.btn{width:100%}.row form{flex:1 1 100%}}");
  html += F("a{color:#a9cbff}</style></head><body><div class='wrap'><h2>Tank Calibration</h2><div class='card'>");

  html += F("<p>Raw: <b>"); html += String(raw); html += F("</b></p>");
  html += F("<p>Calibrated range: <b>"); html += String(tankEmptyRaw); html += F("</b> to <b>"); html += String(tankFullRaw); html += F("</b></p>");
  if (!isValidAdcPin(tankLevelPin)) {
    #if defined(CONFIG_IDF_TARGET_ESP32)
      html += F("<p style='color:#f59e0b'>Invalid ADC pin. Set tank sensor GPIO to 32-39 (ESP32) in Setup.</p>");
    #else
      html += F("<p style='color:#f59e0b'>Invalid ADC pin. Set tank sensor GPIO to 1-20 (ESP32-S3) in Setup.</p>");
    #endif
  } else {
    html += F("<p>Level: <b>"); html += String(pct); html += F("%</b></p>");
  }
  html += F("<div class='row'><form method='POST' action='/setTankEmpty'><button class='btn' type='submit'>Set Empty</button></form>");
  html += F("<form method='POST' action='/setTankFull'><button class='btn' type='submit'>Set Full</button></form></div>");
  html += F("<p><a href='/'>Home</a> | <a href='/setup'>Setup</a></p></div></div>");
  html += F("<script>");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("setTimeout(()=>location.reload(),2000);");
  html += F("</script></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Config & Schedule ----------

static String _safeReadLine(File& f) {
  if (!f.available()) return String("");
  String s = f.readStringUntil('\n');
  s.trim();
  return s;
}

void loadConfig() {
  if (!LittleFS.exists("/config.txt")) return;
  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  String s;
  meteoLat = NAN;
  meteoLon = NAN;
  meteoLocation = "";

  // Legacy & core (now: Open-Meteo coords)
  if ((s = _safeReadLine(f)).length()) {
    float v = s.toFloat();
    if (v >= -90.0f && v <= 90.0f) meteoLat = v;
  }
  if ((s = _safeReadLine(f)).length()) {
    float v = s.toFloat();
    if (v >= -180.0f && v <= 180.0f) meteoLon = v;
  }
  if ((s = _safeReadLine(f)).length()) tzOffsetHours = s.toFloat(); // legacy

  if ((s = _safeReadLine(f)).length()) rainDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) windSpeedThreshold   = s.toFloat();
  if ((s = _safeReadLine(f)).length()) windDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseTank          = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseMains         = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) tankEmptyRaw         = s.toInt();
  if ((s = _safeReadLine(f)).length()) tankFullRaw          = s.toInt();
  if ((s = _safeReadLine(f)).length()) {
    int z = s.toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }
  if ((s = _safeReadLine(f)).length()) rainSensorEnabled = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (p >= 0 && p <= 39) rainSensorPin = p;
  }
  if ((s = _safeReadLine(f)).length()) rainSensorInvert  = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int th = s.toInt();
    if (th >= 0 && th <= 100) tankLowThresholdPct = th;
  }
  if ((s = _safeReadLine(f)).length()) tankEnabled = (s.toInt() == 1);

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      if (p >= -1 && p <= 39) zonePins[i] = p;
    }
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (isValidOutputPin(p)) mainsPin = p;
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (isValidOutputPin(p)) tankPin  = p;
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualSelectPin = (p >= -1 && p <= 39) ? p : -1;
    }
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualStartPin = (p >= -1 && p <= 39) ? p : -1;
    }
  }

  // Zone names
  for (int i = 0; i < MAX_ZONES && f.available(); i++) {
    String nm = _safeReadLine(f);
    if (nm.length()) zoneNames[i] = nm;
  }

  // trailing (older "new" ones)
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainDelayFromForecastEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemPaused                 = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) pauseUntilEpoch             = (uint32_t)s.toInt(); }

  // Timezone block
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzMode            = (TZMode)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzPosix          = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzIANA           = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzFixedOffsetMin = (int16_t)s.toInt(); }

  // NEW trailing: master / cooldown / threshold / run mode / MQTT
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemMasterEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainCooldownMin     = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainThreshold24h_mm = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) runZonesConcurrent  = (s.toInt() == 1); }

  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBroker  = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPort    = (uint16_t)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttUser    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPass    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBase    = s; }

  // ? NEW: relay polarity (optional trailing line, backwards compatible)
  if (f.available()) { 
    if ((s = _safeReadLine(f)).length()) 
      relayActiveHigh = (s.toInt() == 1);
  }

  // NEW: persisted GPIO polarity for fallback mode
  if (f.available()) {
    if ((s = _safeReadLine(f)).length())
      gpioActiveLow = (s.toInt() == 1);
  }

  // Default per-output polarity to the legacy global value, then override if present.
  for (int i = 0; i < MAX_ZONES; ++i) zoneGpioActiveLow[i] = gpioActiveLow;
  mainsGpioActiveLow = gpioActiveLow;
  tankGpioActiveLow  = gpioActiveLow;

  // Read the rest of the file so older configs can safely skip the new per-pin polarity block.
  String tail[48];
  int tailCount = 0;
  while (f.available() && tailCount < (int)(sizeof(tail) / sizeof(tail[0]))) {
    tail[tailCount++] = _safeReadLine(f);
  }
  int tailIdx = 0;
  const int legacyTailCount = 19;
  const int perOutputPolarityCount = MAX_ZONES + 2;

  auto nextTail = [&](String& out) -> bool {
    if (tailIdx >= tailCount) return false;
    out = tail[tailIdx++];
    return true;
  };

  if (tailCount >= legacyTailCount + perOutputPolarityCount) {
    for (int i = 0; i < MAX_ZONES; ++i) {
      if (nextTail(s) && s.length()) zoneGpioActiveLow[i] = (s.toInt() == 1);
    }
    if (nextTail(s) && s.length()) mainsGpioActiveLow = (s.toInt() == 1);
    if (nextTail(s) && s.length()) tankGpioActiveLow  = (s.toInt() == 1);
  }

  // NEW: tank level sensor pin (ADC)
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    if (isValidAdcPin(p)) tankLevelPin = p;
  }

  // NEW: photoresistor auto-backlight (optional trailing lines)
  if (nextTail(s) && s.length()) photoAutoEnabled = (s.toInt() == 1);
  if (nextTail(s) && s.length()) {
    int p = s.toInt();
    photoPin = isValidPhotoPin(p) ? p : -1;
  }
  if (nextTail(s) && s.length()) photoThreshold = s.toInt();
  if (nextTail(s) && s.length()) photoInvert = (s.toInt() == 1);

  // NEW: TFT SPI pins (optional trailing lines)
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftSclkPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftMosiPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftCsPin   = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidTftSignalPin(p))   tftDcPin   = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidOptionalTftPin(p)) tftRstPin  = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidOptionalTftPin(p)) tftBlPin   = p; }

  // NEW: Open-Meteo location label (optional trailing line)
  if (nextTail(s) && s.length()) meteoLocation = cleanName(s);
  // NEW: Open-Meteo model (optional trailing line)
  if (nextTail(s) && s.length()) meteoModel = cleanMeteoModel(s);
  // NEW: I2C pins (optional trailing lines)
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidGpioPin(p)) i2cSdaPin = p; }
  if (nextTail(s) && s.length()) { int p=s.toInt(); if (isValidGpioPin(p)) i2cSclPin = p; }
  // NEW: display mode (optional trailing line)
  if (nextTail(s) && s.length()) displayUseTft = (s.toInt() == 1);
  // NEW: TFT rotation (optional trailing line)
  if (nextTail(s) && s.length()) {
      int r = s.toInt();
      if (r < 0) r = 0;
      if (r > 3) r = 3;
      tftRotation = (uint8_t)r;
  }
  // NEW: TFT panel width/height (optional trailing lines)
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (isValidTftDimension(v)) tftPanelWidth = (int16_t)v;
  }
  if (nextTail(s) && s.length()) {
    int v = s.toInt();
    if (isValidTftDimension(v)) tftPanelHeight = (int16_t)v;
  }

  f.close();

  // Derive hours from minutes (for UI & cooldown logic)
  if (rainCooldownMin < 0) rainCooldownMin = 0;
  rainCooldownHours = (uint8_t)(rainCooldownMin / 60);
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  if (zonesCount < 1) zonesCount = 1;
  if (zonesCount > MAX_ZONES) zonesCount = MAX_ZONES;

  // Core / legacy order (now: Open-Meteo coords)
  if (isValidLatLon(meteoLat, meteoLon)) {
    f.println(String(meteoLat, 6));
    f.println(String(meteoLon, 6));
  } else {
    f.println("");
    f.println("");
  }
  f.println(String(tzOffsetHours, 3));

  f.println(rainDelayEnabled ? 1 : 0);
  f.println(String(windSpeedThreshold, 3));
  f.println(windDelayEnabled ? 1 : 0);
  f.println(justUseTank ? 1 : 0);
  f.println(justUseMains ? 1 : 0);
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  f.println(zonesCount);
  f.println(rainSensorEnabled ? 1 : 0);
  f.println(rainSensorPin);
  f.println(rainSensorInvert ? 1 : 0);
  f.println(tankLowThresholdPct);
  f.println(tankEnabled ? 1 : 0);

  // ? REMOVE this line from here (it breaks the alignment!)
  // f.println(relayActiveHigh ? 1 : 0);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zonePins[i]);
  f.println(mainsPin);
  f.println(tankPin);
  f.println(manualSelectPin);
  f.println(manualStartPin);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zoneNames[i]);

  // trailing
  f.println(rainDelayFromForecastEnabled ? 1 : 0);
  f.println(systemPaused ? 1 : 0);
  f.println(pauseUntilEpoch);

  // Timezone
  f.println((int)tzMode);
  f.println(tzPosix);
  f.println(tzIANA);
  f.println(tzFixedOffsetMin);

  // Master / cooldown / threshold / run mode / MQTT
  // keep minutes & hours in sync
  rainCooldownMin   = (int)rainCooldownHours * 60;
  if (rainCooldownMin < 0) rainCooldownMin = 0;

  f.println(systemMasterEnabled ? 1 : 0);
  f.println(rainCooldownMin);
  f.println(rainThreshold24h_mm);
  f.println(runZonesConcurrent ? 1 : 0);

  f.println(mqttEnabled ? 1 : 0);
  f.println(mqttBroker);
  f.println(mqttPort);
  f.println(mqttUser);
  f.println(mqttPass);
  f.println(mqttBase);

  // ? NEW: relay polarity written as trailing line to match loadConfig()
  f.println(relayActiveHigh ? 1 : 0);
  // NEW: persist GPIO polarity for fallback mode
  f.println(gpioActiveLow ? 1 : 0);
  // NEW: persist per-output GPIO polarity for fallback mode
  for (int i = 0; i < MAX_ZONES; ++i) f.println(zoneGpioActiveLow[i] ? 1 : 0);
  f.println(mainsGpioActiveLow ? 1 : 0);
  f.println(tankGpioActiveLow ? 1 : 0);
  // NEW: tank level sensor pin (ADC)
  f.println(tankLevelPin);
  // NEW: photoresistor auto-backlight
  f.println(photoAutoEnabled ? 1 : 0);
  f.println(photoPin);
  f.println(photoThreshold);
  f.println(photoInvert ? 1 : 0);
  // NEW: TFT SPI pins
  f.println(tftSclkPin);
  f.println(tftMosiPin);
  f.println(tftCsPin);
  f.println(tftDcPin);
  f.println(tftRstPin);
  f.println(tftBlPin);
  // NEW: Open-Meteo location label
  f.println(cleanName(meteoLocation));
  // NEW: Open-Meteo model
  f.println(cleanMeteoModel(meteoModel));
  // NEW: I2C pins
  f.println(i2cSdaPin);
  f.println(i2cSclPin);
  // NEW: display mode (1=TFT, 0=OLED)
  f.println(displayUseTft ? 1 : 0);
  // NEW: TFT rotation (0..3)
  f.println((int)tftRotation);
  // NEW: TFT panel width/height
  f.println((int)tftPanelWidth);
  f.println((int)tftPanelHeight);

  f.close();
}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt","r");
  if (!f) return;

  for (int i=0; i<MAX_ZONES; i++) {
    String line=f.readStringUntil('\n'); line.trim(); if (!line.length()) continue;

    int idx=0; int tcount=0; int tokens[32];
    while (idx < (int)line.length() && tcount < 32) {
      int nidx=line.indexOf(',', idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim();
      tokens[tcount++] = sv.toInt();
      idx = (nidx<(int)line.length()) ? nidx+1 : nidx;
    }
    auto tok=[&](int k,int def)->int{ return (k < tcount) ? tokens[k] : def; };

    startHour[i]    = tok(0, startHour[i]);
    startMin[i]     = tok(1, startMin[i]);
    startHour2[i]   = tok(2, startHour2[i]);
    startMin2[i]    = tok(3, startMin2[i]);
    durationMin[i]  = tok(4, durationMin[i]);
    durationSec[i]  = tok(5, durationSec[i]);
    duration2Min[i] = tok(6, durationMin[i]);
    duration2Sec[i] = tok(7, durationSec[i]);

    int enIdx = (tcount >= 9) ? 8 : 6; // compatibility when duration2 fields absent
    enableStartTime2[i] = (tok(enIdx, enableStartTime2[i]) == 1);

    int dayStart = enIdx + 1;
    for (int d=0; d<7; d++) {
      days[i][d] = (tok(dayStart + d, days[i][d]) == 1);
    }
  }
  f.close();
}

void saveSchedule() {
  File f = LittleFS.open("/schedule.txt","w");
  if (!f) return;
  for (int i=0; i<MAX_ZONES; i++) {
    f.print(startHour[i]);  f.print(',');
    f.print(startMin[i]);   f.print(',');
    f.print(startHour2[i]); f.print(',');
    f.print(startMin2[i]);  f.print(',');
    f.print(durationMin[i]);f.print(',');
    f.print(durationSec[i]);f.print(',');
    f.print(duration2Min[i]);f.print(',');
    f.print(duration2Sec[i]);f.print(',');
    f.print(enableStartTime2[i] ? '1' : '0');
    for (int d=0; d<7; d++){ f.print(','); f.print(days[i][d] ? '1' : '0'); }
    f.println();
  }
  f.close();
}

void handleConfigure() {
  HttpScope _scope;
  String displayCfgErr;

    // NEW: snapshot current values before applying POST changes
  float  oldLat    = meteoLat;
  float  oldLon    = meteoLon;
  String oldLoc    = meteoLocation;
  String oldModel  = meteoModel;
  int oldTftSclk = tftSclkPin;
  int oldTftMosi = tftMosiPin;
  int oldTftCs   = tftCsPin;
  int oldTftDc   = tftDcPin;
  int oldTftRst  = tftRstPin;
  int oldTftBl   = tftBlPin;
  int oldTftWidth = tftPanelWidth;
  int oldTftHeight = tftPanelHeight;
  int oldTftRotation = tftRotation;
  int oldI2cSda  = i2cSdaPin;
  int oldI2cScl  = i2cSclPin;
  bool oldDisplayUseTft = displayUseTft;

  // Weather (Open-Meteo)
  if (server.hasArg("meteoLocation")) {
    meteoLocation = cleanName(server.arg("meteoLocation"));
  }
  if (server.hasArg("meteoModelSelect")) {
    String sel = cleanMeteoModel(server.arg("meteoModelSelect"));
    if (sel == "custom" && server.hasArg("meteoModelCustom")) {
      meteoModel = cleanMeteoModel(server.arg("meteoModelCustom"));
    } else {
      meteoModel = cleanMeteoModel(sel);
    }
  } else if (server.hasArg("meteoModel")) {
    meteoModel = cleanMeteoModel(server.arg("meteoModel"));
  }
  if (server.hasArg("meteoLat")) {
    String s = server.arg("meteoLat"); s.trim();
    if (!s.length()) {
      meteoLat = NAN;
    } else {
      float v = s.toFloat();
      if (v >= -90.0f && v <= 90.0f) meteoLat = v;
    }
  }
  if (server.hasArg("meteoLon")) {
    String s = server.arg("meteoLon"); s.trim();
    if (!s.length()) {
      meteoLon = NAN;
    } else {
      float v = s.toFloat();
      if (v >= -180.0f && v <= 180.0f) meteoLon = v;
    }
  }
  // Fallback: allow "lat,lon" in the latitude field
  if (!isValidLatLon(meteoLat, meteoLon) && server.hasArg("meteoLat")) {
    float la = NAN, lo = NAN;
    if (parseLatLon(server.arg("meteoLat"), la, lo)) {
      meteoLat = la;
      meteoLon = lo;
    }
  }

  // Zones mode (1..MAX_ZONES, 4 keeps tank/mains behaviour)
  if (server.hasArg("zonesMode")) {
    int z = server.arg("zonesMode").toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }

  // Run mode (sequential vs concurrent)
  runZonesConcurrent = server.hasArg("runConcurrent");

  // Rain forecast gate
  rainDelayFromForecastEnabled = !server.hasArg("rainForecastDisabled");

  // Delay toggles
  rainDelayEnabled = server.hasArg("rainDelay");
  windDelayEnabled = server.hasArg("windCancelEnabled");

  // Physical rain sensor
  rainSensorEnabled = server.hasArg("rainSensorEnabled");
  if (server.hasArg("rainSensorPin")) {
    int pin = server.arg("rainSensorPin").toInt();
    if (pin >= 0 && pin <= 39) rainSensorPin = pin;
  }
  rainSensorInvert = server.hasArg("rainSensorInvert");

  // Wind threshold
  if (server.hasArg("windSpeedThreshold")) {
    windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();
    if (windSpeedThreshold < 0) windSpeedThreshold = 0;
  }

  // Rain cooldown (hours ? minutes + uint8)
  if (server.hasArg("rainCooldownHours")) {
    int h = server.arg("rainCooldownHours").toInt();
    if (h < 0) h = 0;
    rainCooldownHours = (uint8_t)h;
    rainCooldownMin   = h * 60;
  }

  // 24h rain threshold (mm)
  if (server.hasArg("rainThreshold24h")) {
    int mm = server.arg("rainThreshold24h").toInt();
    if (mm < 0) mm = 0;
    rainThreshold24h_mm = mm;
  }

  // Pause handling
  bool resumeNow = server.hasArg("resumeNow");
  int pauseHours = server.hasArg("pauseHours") ? server.arg("pauseHours").toInt() : 0;
  time_t nowEp   = time(nullptr);

  if (resumeNow) {
    systemPaused    = false;
    pauseUntilEpoch = 0;
  } else {
    if (server.hasArg("pauseEnable")) {
      if (pauseHours > 0) {
        systemPaused    = true;
        pauseUntilEpoch = nowEp + (time_t)pauseHours * 3600;
      } else {
        // 0 hours + checkbox = "until manual resume"
        systemPaused    = true;
        pauseUntilEpoch = 0;
      }
    } else {
      // Checkbox not ticked ? clear pause
      systemPaused    = false;
      pauseUntilEpoch = 0;
    }
  }

  // Water mode
  if (server.hasArg("waterMode")) {
    String wm = server.arg("waterMode");
    if (wm == "tank") {
      justUseTank  = true;
      justUseMains = false;
    } else if (wm == "mains") {
      justUseTank  = false;
      justUseMains = true;
    } else { // auto
      justUseTank  = false;
      justUseMains = false;
    }
  }

  // Tank low threshold
  if (server.hasArg("tankThresh")) {
    int th = server.arg("tankThresh").toInt();
    if (th < 0)   th = 0;
    if (th > 100) th = 100;
    tankLowThresholdPct = th;
  }
  tankEnabled = server.hasArg("tankEnabled");

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    String key = "zonePin" + String(i);
    if (server.hasArg(key)) {
      int p = server.arg(key).toInt();
      if (p >= -1 && p <= 39) zonePins[i] = p;
    }
    String polarityKey = "zonePinLow" + String(i);
    zoneGpioActiveLow[i] = server.hasArg(polarityKey);
  }
  if (server.hasArg("mainsPin")) {
    int p = server.arg("mainsPin").toInt();
    if (isValidOutputPin(p)) mainsPin = p;
  }
  mainsGpioActiveLow = server.hasArg("mainsPinLow");
  if (server.hasArg("tankPin")) {
    int p = server.arg("tankPin").toInt();
    if (isValidOutputPin(p)) tankPin = p;
  }
  tankGpioActiveLow = server.hasArg("tankPinLow");
  if (server.hasArg("tankLevelPin")) {
    int p = server.arg("tankLevelPin").toInt();
    if (isValidAdcPin(p)) tankLevelPin = p;
  }
  // Photoresistor auto-backlight
  photoAutoEnabled = server.hasArg("photoAuto");
  if (server.hasArg("photoPin")) {
    int p = server.arg("photoPin").toInt();
    if (isValidPhotoPin(p)) photoPin = p;
  }
  if (server.hasArg("photoThreshold")) {
    int th = server.arg("photoThreshold").toInt();
    if (th < 0) th = 0;
    if (th > 4095) th = 4095;
    photoThreshold = th;
  }
  photoInvert = server.hasArg("photoInvert");
  if (server.hasArg("displayType")) {
    String dm = server.arg("displayType");
    dm.toLowerCase();
    if (dm == "tft") displayUseTft = true;
    else if (dm == "oled") displayUseTft = false;
  }
  if (server.hasArg("tftRotation")) {
    int r = server.arg("tftRotation").toInt();
    if (r < 0) r = 0;
    if (r > 3) r = 3;
    tftRotation = (uint8_t)r;
  }
  if (server.hasArg("tftWidth")) {
    int v = 0;
    if (parseRequiredIntArg(server.arg("tftWidth"), v, displayCfgErr, F("TFT width"))) {
      if (isValidTftDimension(v)) tftPanelWidth = (int16_t)v;
      else displayCfgErr += "TFT width out of range: " + String(v) + " (allowed 120..400)\n";
    }
  }
  if (server.hasArg("tftHeight")) {
    int v = 0;
    if (parseRequiredIntArg(server.arg("tftHeight"), v, displayCfgErr, F("TFT height"))) {
      if (isValidTftDimension(v)) tftPanelHeight = (int16_t)v;
      else displayCfgErr += "TFT height out of range: " + String(v) + " (allowed 120..400)\n";
    }
  }
  // TFT SPI pins (apply on reboot)
  if (server.hasArg("tftSclk")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftSclk"), p, displayCfgErr, F("TFT SCK"))) {
      if (isValidTftSignalPin(p)) tftSclkPin = p;
      else displayCfgErr += "TFT SCK invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftMosi")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftMosi"), p, displayCfgErr, F("TFT MOSI"))) {
      if (isValidTftSignalPin(p)) tftMosiPin = p;
      else displayCfgErr += "TFT MOSI invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftCs")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftCs"), p, displayCfgErr, F("TFT CS"))) {
      if (isValidTftSignalPin(p)) tftCsPin = p;
      else displayCfgErr += "TFT CS invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftDc")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftDc"), p, displayCfgErr, F("TFT DC"))) {
      if (isValidTftSignalPin(p)) tftDcPin = p;
      else displayCfgErr += "TFT DC invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftRst")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftRst"), p, displayCfgErr, F("TFT RST"))) {
      if (isValidOptionalTftPin(p)) tftRstPin = p;
      else displayCfgErr += "TFT RST invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (server.hasArg("tftBl")) {
    int p = 0;
    if (parseRequiredIntArg(server.arg("tftBl"), p, displayCfgErr, F("TFT BL"))) {
      if (isValidOptionalTftPin(p)) tftBlPin = p;
      else displayCfgErr += "TFT BL invalid/unsafe: GPIO" + String(p) + "\n";
    }
  }
  if (displayCfgErr.length()) {
    server.send(400, "text/plain", displayCfgErr);
    return;
  }
  if (server.hasArg("manualSelectPin")) {
    int p = server.arg("manualSelectPin").toInt();
    if ((p >= -1 && p <= 39)) manualSelectPin = p;
  }
  if (server.hasArg("manualStartPin")) {
    int p = server.arg("manualStartPin").toInt();
    if ((p >= -1 && p <= 39)) manualStartPin = p;
  }

  // Keep the legacy global polarity aligned for backward-compatible saves.
  gpioActiveLow = false;
  for (int i = 0; i < MAX_ZONES; ++i) {
    if (zoneGpioActiveLow[i]) { gpioActiveLow = true; break; }
  }
  if (mainsGpioActiveLow || tankGpioActiveLow) gpioActiveLow = true;

  // Timezone mode + values
  if (server.hasArg("tzMode")) {
    int m = server.arg("tzMode").toInt();
    if (m < 0) m = 0;
    if (m > 2) m = 2;
    tzMode = (TZMode)m;
  }
  if (server.hasArg("tzPosix")) {
    String v = server.arg("tzPosix");
    v.trim();
    if (v.length()) tzPosix = v;
  }
  if (server.hasArg("tzIANA")) {
    String v = server.arg("tzIANA");
    v.trim();
    if (v.length()) tzIANA = v;
  }
  if (server.hasArg("tzFixed")) {
    tzFixedOffsetMin = (int16_t)server.arg("tzFixed").toInt();
  }

  // MQTT
  mqttEnabled = server.hasArg("mqttEnabled");
  if (server.hasArg("mqttBroker")) mqttBroker = server.arg("mqttBroker");
  if (server.hasArg("mqttPort"))   mqttPort   = (uint16_t)server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser"))   mqttUser   = server.arg("mqttUser");
  if (server.hasArg("mqttPass"))   mqttPass   = server.arg("mqttPass");
  if (server.hasArg("mqttBase"))   mqttBase   = server.arg("mqttBase");

  bool tftPinsChanged = (oldTftSclk != tftSclkPin ||
                         oldTftMosi != tftMosiPin ||
                         oldTftCs   != tftCsPin   ||
                         oldTftDc   != tftDcPin   ||
                         oldTftRst  != tftRstPin  ||
                         oldTftBl   != tftBlPin);
  bool tftGeometryChanged = (oldTftWidth != tftPanelWidth ||
                             oldTftHeight != tftPanelHeight);

  if (server.hasArg("i2cSda")) {
    int p = server.arg("i2cSda").toInt();
    if (isValidGpioPin(p)) i2cSdaPin = p;
  }
  if (server.hasArg("i2cScl")) {
    int p = server.arg("i2cScl").toInt();
    if (isValidGpioPin(p)) i2cSclPin = p;
  }

  bool i2cPinsChanged = (oldI2cSda != i2cSdaPin || oldI2cScl != i2cSclPin);
  bool displayModeChanged = (oldDisplayUseTft != displayUseTft);
  bool tftRotationChanged = (oldTftRotation != (int)tftRotation);

  // Persist and re-apply runtime things
  validatePinMap();
  saveConfig();
  initManualButtons();
  initGpioPinsForZones();
  if (!displayModeChanged && !tftGeometryChanged && displayUseTft && tftRotationChanged) {
    tft.setRotation(tftRotation);
  }

  // --- NEW: if location/coords changed, force a weather refresh ---
  auto coordChanged = [](float a, float b) -> bool {
    if (!isfinite(a) && !isfinite(b)) return false;
    if (!isfinite(a) || !isfinite(b)) return true;
    return fabsf(a - b) > 0.0001f;
  };
  if (coordChanged(oldLat, meteoLat) || coordChanged(oldLon, meteoLon) || (oldLoc != meteoLocation) || (oldModel != meteoModel)) {
    // Clear current / forecast caches and timers
    cachedWeatherData   = "";
    cachedForecastData  = "";
    lastWeatherUpdate   = 0;
    lastForecastUpdate  = 0;

    // Reset derived metrics so /status & UI show clean values until next fetch
    rain1hNow           = 0.0f;
    rain3hNow           = 0.0f;
    curTempC            = NAN;
    curFeelsC           = NAN;
    curPressureHpa      = NAN;
    curWindMs           = NAN;
    curGustMs           = NAN;
    curWindDirDeg       = NAN;
    curHumidityPct      = -1;
    curWeatherCode      = -1;
    curUtcOffsetSec     = 0;
    curWeatherValid     = false;
    rainNext12h_mm      = 0.0f;
    rainNext24h_mm      = 0.0f;
    popNext12h_pct      = -1;
    nextRainIn_h        = -1;
    maxGust24h_ms       = 0.0f;
    todayMin_C          = NAN;
    todayMax_C          = NAN;
    todaySunrise        = 0;
    todaySunset         = 0;
    for (int i = 0; i < 24; ++i) {
      rainHist[i] = 0.0f;
    }
    rainIdx              = 0;
    lastRainHistHour     = 0;
    lastRainAmount       = 0.0f;
  }


  applyTimezoneAndSNTP();  // re-sync NTP with new TZ
  mqttSetup();             // reconfigure client; loop() will reconnect

  server.sendHeader("Location", "/setup", true);
  server.send(302, "text/plain", "");

  if (tftPinsChanged || tftGeometryChanged || i2cPinsChanged || displayModeChanged) {
    Serial.println("[CFG] Display/pin mapping changed, restarting to apply...");
    delay(200);
    ESP.restart();
  }
}

void handleClearEvents() {
  HttpScope _scope;
  if (LittleFS.exists("/events.csv")) {
    LittleFS.remove("/events.csv");
  }
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}
