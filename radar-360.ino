/*
 * ============================================================================
 *  RADAR 360 - Realtime ADS-B Airplane Radar, Live Weather & Luxury Clock
 * ============================================================================
 *  [하드웨어]
 *   - Board   : ESP32-4848S040 (ESP32-S3, 4.0" 480x480 RGB LCD, ST7701S)
 *   - Memory  : 16MB Flash, 8MB OPI PSRAM (Arduino IDE에서 OPI PSRAM 필수 활성화)
 *
 *  [주요 기능]
 *   1. Flight Radar : 실시간 항공기 추적 (adsb.lol / adsb.fi / airplanes.live)
 *                     Google Maps 배경 지도(위성/로드맵/지형도) + 회전형 비행기 아이콘
 *   2. Weather View : Open-Meteo 실시간 기상 + 3일 예보 + RainViewer 강수 레이더
 *   3. Clock View   : 세계 시간대 설정 + 아날로그 4종(Rolex/Longines/Breitling/Omega)
 *                     + 디지털 3종(Modern/Neon Ring/Retro Flip)
 *   4. Web Console  : Wi-Fi 미연결 시 AP 모드("RADAR-360-Setup", 192.168.4.1) 자동 실행
 *                     Wi-Fi 연결 후 http://radar.local 로 모든 설정 실시간 변경 가능
 *
 *  [Arduino IDE 권장 설정]
 *   - Board            : ESP32S3 Dev Module
 *   - Flash Size       : 16MB (128Mb)
 *   - Flash Mode       : DIO
 *   - PSRAM            : OPI PSRAM
 *   - Partition Scheme : Huge APP (3MB No OTA / 1MB SPIFFS)
 *   - Upload Speed     : 115200 (macOS 안정화 권장)
 * ============================================================================
 */

#include <Wire.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "time.h"

// ==========================================
// 1. 설정 구조체 및 전역 객체
// ==========================================
const char* AP_SSID = "RADAR-360-Setup";

struct RadarConfig {
  // Network
  String ssid       = "";
  String pass       = "";
  String hostname   = "radar";

  // Radar & Map
  String presetName = "INCHEON";
  double lat        = 37.4602;
  double lon        = 126.4407;
  float  radiusKm   = 100.0f;
  int    refreshSec = 10;
  int    minAltFt   = 0;
  int    maxAltFt   = 60000;
  int    maxAc      = 24;
  int    mapStyle   = 0;      // 0: Google Satellite Hybrid, 1: Google Roadmap, 2: Google Terrain

  int    unitDist   = 0;      // 0: km, 1: nm
  int    unitAlt    = 1;      // 0: m,  1: ft
  int    unitSpeed  = 1;      // 0: km/h, 1: kt
  int    viewMode   = 0;      // 0: Flight radar, 1: Weather view, 2: Clock view

  bool   showMapBg     = true;
  bool   showScanLine  = true;
  bool   showCallsign  = true;
  bool   showAltitude  = true;
  bool   showHeading   = false;
  bool   showSpeed     = true;
  bool   showDistance  = false;
  bool   showAcType    = true;
  bool   showVectors   = true;

  // Weather
  bool   wxUseRadarLoc = true;
  double wxLat         = 37.4602;
  double wxLon         = 126.4407;
  int    unitTemp      = 0;   // 0: Celsius, 1: Fahrenheit
  int    unitPrecip    = 0;   // 0: mm, 1: in
  bool   wxShowCards   = true;
  bool   wxFeelsLike   = true;
  bool   wxPrecip      = true;
  bool   wxHumidity    = true;
  bool   wxWind        = true;
  bool   wxPressure    = true;

  // Clock
  int    clockStyle    = 0;   // 0:Rolex, 1:Longines, 2:Breitling, 3:Omega, 4:DigModern, 5:DigNeon, 6:DigFlip
  String tzPosix       = "KST-9";
  String tzLabel       = "SEOUL (UTC+9)";

  // System
  int    brightness    = 85;
};

struct WeatherData {
  bool   valid       = false;
  float  tempC       = 0.0f;
  float  feelsLikeC  = 0.0f;
  int    humidity    = 0;
  int    cloudCover  = 0;
  float  precipMm    = 0.0f;
  float  windKmh     = 0.0f;
  float  gustKmh     = 0.0f;
  int    windDir     = 0;
  float  pressureHpa = 1013.0f;
  int    wmoCode     = 0;
  int    dayCode[3]  = {0, 0, 0};
  float  dayMaxC[3]  = {0, 0, 0};
  float  dayMinC[3]  = {0, 0, 0};
  int    dayPop[3]   = {0, 0, 0};
};

RadarConfig cfg;
WeatherData wxData, pendingWxData;
bool pendingWxReady = false, wxFetchInProgress = false;
unsigned long lastWxFetchTime = 0;
const unsigned long WX_FETCH_INTERVAL = 600000UL;

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
bool isApMode = false;

int    currentMapZoom     = 8;
float  effectiveRangeKm   = 100.0f;
bool   mapLoaded          = false;
bool   mapFetchInProgress = false;

// PSRAM을 사용하는 ArduinoJson 커스텀 할당자 (v6 / v7 자동 호환)
#if ARDUINOJSON_VERSION_MAJOR >= 7
struct SpiRamAllocator : ArduinoJson::Allocator {
  void* allocate(size_t size) override { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
  void deallocate(void* pointer) override { heap_caps_free(pointer); }
  void* reallocate(void* ptr, size_t new_size) override { return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM); }
};
#else
struct SpiRamAllocator {
  void* allocate(size_t size) { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
  void deallocate(void* pointer) { heap_caps_free(pointer); }
  void* reallocate(void* ptr, size_t new_size) { return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM); }
};
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;
#endif

// ==========================================
// 2. ST7701S 3-Wire SPI 직접 초기화
// ==========================================
#define SPI_CS_PIN   39
#define SPI_SCK_PIN  48
#define SPI_SDA_PIN  47

void st7701_send(uint8_t data, bool is_cmd) {
  digitalWrite(SPI_CS_PIN, LOW);
  digitalWrite(SPI_SCK_PIN, LOW);
  digitalWrite(SPI_SDA_PIN, is_cmd ? LOW : HIGH);
  digitalWrite(SPI_SCK_PIN, HIGH);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SPI_SCK_PIN, LOW);
    digitalWrite(SPI_SDA_PIN, (data >> i) & 0x01);
    digitalWrite(SPI_SCK_PIN, HIGH);
  }
  digitalWrite(SPI_CS_PIN, HIGH);
}
void st7701_cmd(uint8_t cmd) { st7701_send(cmd, true); }
void st7701_dat(uint8_t dat) { st7701_send(dat, false); }

void st7701_init_panel() {
  pinMode(SPI_CS_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);
  pinMode(SPI_SDA_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  digitalWrite(SPI_SCK_PIN, HIGH);
  delay(20);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x10);
  st7701_cmd(0xC0); st7701_dat(0x3B); st7701_dat(0x00);
  st7701_cmd(0xC1); st7701_dat(0x0D); st7701_dat(0x02);
  st7701_cmd(0xC2); st7701_dat(0x31); st7701_dat(0x05);
  st7701_cmd(0xCD); st7701_dat(0x00);

  st7701_cmd(0xB0);
  st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x18); st7701_dat(0x0E);
  st7701_dat(0x11); st7701_dat(0x06); st7701_dat(0x07); st7701_dat(0x08);
  st7701_dat(0x07); st7701_dat(0x22); st7701_dat(0x04); st7701_dat(0x12);
  st7701_dat(0x0F); st7701_dat(0xAA); st7701_dat(0x31); st7701_dat(0x18);

  st7701_cmd(0xB1);
  st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x19); st7701_dat(0x0E);
  st7701_dat(0x12); st7701_dat(0x07); st7701_dat(0x08); st7701_dat(0x08);
  st7701_dat(0x08); st7701_dat(0x22); st7701_dat(0x04); st7701_dat(0x11);
  st7701_dat(0x11); st7701_dat(0xA9); st7701_dat(0x32); st7701_dat(0x18);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11);
  st7701_cmd(0xB0); st7701_dat(0x60);
  st7701_cmd(0xB1); st7701_dat(0x32);
  st7701_cmd(0xB2); st7701_dat(0x07);
  st7701_cmd(0xB3); st7701_dat(0x80);
  st7701_cmd(0xB5); st7701_dat(0x49);
  st7701_cmd(0xB7); st7701_dat(0x85);
  st7701_cmd(0xB8); st7701_dat(0x21);
  st7701_cmd(0xC1); st7701_dat(0x78);
  st7701_cmd(0xC2); st7701_dat(0x78);
  delay(20);

  st7701_cmd(0xE0); st7701_dat(0x00); st7701_dat(0x1B); st7701_dat(0x02);
  st7701_cmd(0xE1);
  st7701_dat(0x08); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0x07); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0x00); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE2);
  st7701_dat(0x11); st7701_dat(0x11); st7701_dat(0x44); st7701_dat(0x44);
  st7701_dat(0xED); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);
  st7701_dat(0xEC); st7701_dat(0xA0); st7701_dat(0x00); st7701_dat(0x00);

  st7701_cmd(0xE3); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x11);
  st7701_cmd(0xE4); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE5);
  st7701_dat(0x0A); st7701_dat(0xE9); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0C); st7701_dat(0xEB); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0E); st7701_dat(0xED); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x10); st7701_dat(0xEF); st7701_dat(0xD8); st7701_dat(0xA0);

  st7701_cmd(0xE6); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x11); st7701_dat(0x11);
  st7701_cmd(0xE7); st7701_dat(0x44); st7701_dat(0x44);

  st7701_cmd(0xE8);
  st7701_dat(0x09); st7701_dat(0xE8); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0B); st7701_dat(0xEA); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0D); st7701_dat(0xEC); st7701_dat(0xD8); st7701_dat(0xA0);
  st7701_dat(0x0F); st7701_dat(0xEE); st7701_dat(0xD8); st7701_dat(0xA0);

  st7701_cmd(0xEB);
  st7701_dat(0x02); st7701_dat(0x00); st7701_dat(0xE4); st7701_dat(0xE4);
  st7701_dat(0x88); st7701_dat(0x00); st7701_dat(0x40);

  st7701_cmd(0xEC); st7701_dat(0x3C); st7701_dat(0x00);

  st7701_cmd(0xED);
  st7701_dat(0xAB); st7701_dat(0x89); st7701_dat(0x76); st7701_dat(0x54);
  st7701_dat(0x02); st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0xFF);
  st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0xFF); st7701_dat(0x20);
  st7701_dat(0x45); st7701_dat(0x67); st7701_dat(0x98); st7701_dat(0xBA);

  st7701_cmd(0xFF); st7701_dat(0x77); st7701_dat(0x01); st7701_dat(0x00); st7701_dat(0x00); st7701_dat(0x00);
  st7701_cmd(0x3A); st7701_dat(0x60);
  st7701_cmd(0x36); st7701_dat(0x00);
  st7701_cmd(0x20);

  st7701_cmd(0x11);
  delay(120);
  st7701_cmd(0x29);
  delay(20);
}

// ==========================================
// 3. LovyanGFX Panel_RGB 구성
// ==========================================
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB   _bus;
  lgfx::Panel_RGB _panel;
  lgfx::Light_PWM _light;

  LGFX(void) {
    {
      auto c = _panel.config();
      c.memory_width  = 480; c.memory_height = 480;
      c.panel_width   = 480; c.panel_height  = 480;
      c.offset_x = 0; c.offset_y = 0;
      _panel.config(c);
    }
    {
      auto c = _bus.config();
      c.panel = &_panel;
      c.pin_d0  = GPIO_NUM_4;  c.pin_d1  = GPIO_NUM_5;  c.pin_d2  = GPIO_NUM_6;
      c.pin_d3  = GPIO_NUM_7;  c.pin_d4  = GPIO_NUM_15; c.pin_d5  = GPIO_NUM_8;
      c.pin_d6  = GPIO_NUM_20; c.pin_d7  = GPIO_NUM_3;  c.pin_d8  = GPIO_NUM_46;
      c.pin_d9  = GPIO_NUM_9;  c.pin_d10 = GPIO_NUM_10; c.pin_d11 = GPIO_NUM_11;
      c.pin_d12 = GPIO_NUM_12; c.pin_d13 = GPIO_NUM_13; c.pin_d14 = GPIO_NUM_14;
      c.pin_d15 = GPIO_NUM_0;
      c.pin_henable = GPIO_NUM_18; c.pin_vsync = GPIO_NUM_17;
      c.pin_hsync   = GPIO_NUM_16; c.pin_pclk  = GPIO_NUM_21;
      c.freq_write = 8000000;
      c.hsync_polarity = 1; c.hsync_front_porch = 10; c.hsync_pulse_width = 8; c.hsync_back_porch = 50;
      c.vsync_polarity = 1; c.vsync_front_porch = 10; c.vsync_pulse_width = 8; c.vsync_back_porch = 20;
      c.pclk_active_neg = 0; c.de_idle_high = 0; c.pclk_idle_high = 0;
      _bus.config(c);
      _panel.setBus(&_bus);
    }
    {
      auto c = _light.config();
      c.pin_bl = GPIO_NUM_38; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX lcd;
LGFX_Sprite radarCanvas(&lcd);
LGFX_Sprite mapBgSprite(&lcd);

const int SCREEN_W = 480;
const int SCREEN_H = 480;
const int CX = 240;
const int CY = 240;
const int R  = 222;

float sweepAngle = 0;
unsigned long lastSweepUpdate = 0;
unsigned long lastApiFetchTime = 0;
const int TRAIL_WIDTH = 45;
const int SWEEP_SPEED = 40;

uint16_t greenBright, green, greenDim, greenDark;
uint16_t colSeaBg, colRiver, colAirport, colCardBg, colWxPanel, colWxBorder;

void startAircraftFetchBackground();
void startWeatherFetchBackground();
void startMapFetchBackground();

// ==========================================
// 4. NVS 설정 불러오기 / 저장
// ==========================================
void loadConfig() {
  preferences.begin("radar360", true);
  cfg.ssid          = preferences.getString("ssid", "");
  cfg.pass          = preferences.getString("pass", "");
  cfg.hostname      = preferences.getString("host", "radar");
  cfg.presetName    = preferences.getString("preset", "INCHEON");
  cfg.lat           = preferences.getDouble("lat", 37.4602);
  cfg.lon           = preferences.getDouble("lon", 126.4407);
  cfg.radiusKm      = preferences.getFloat("radKm", 100.0f);
  cfg.refreshSec    = preferences.getInt("refSec", 10);
  cfg.minAltFt      = preferences.getInt("minAlt", 0);
  cfg.maxAltFt      = preferences.getInt("maxAlt", 60000);
  cfg.maxAc         = preferences.getInt("maxAc", 24);
  cfg.mapStyle      = preferences.getInt("mapSty", 0);
  cfg.unitDist      = preferences.getInt("uDist", 0);
  cfg.unitAlt       = preferences.getInt("uAlt", 1);
  cfg.unitSpeed     = preferences.getInt("uSpd", 1);
  cfg.viewMode      = preferences.getInt("viewM", 0);
  cfg.showMapBg     = preferences.getBool("sMap", true);
  cfg.showScanLine  = preferences.getBool("sScan", true);
  cfg.showCallsign  = preferences.getBool("sCall", true);
  cfg.showAltitude  = preferences.getBool("sAlt", true);
  cfg.showHeading   = preferences.getBool("sHdg", false);
  cfg.showSpeed     = preferences.getBool("sSpd", true);
  cfg.showDistance  = preferences.getBool("sDst", false);
  cfg.showAcType    = preferences.getBool("sTyp", true);
  cfg.showVectors   = preferences.getBool("sVec", true);

  cfg.wxUseRadarLoc = preferences.getBool("wxUseR", true);
  cfg.wxLat         = preferences.getDouble("wxLat", 37.4602);
  cfg.wxLon         = preferences.getDouble("wxLon", 126.4407);
  cfg.unitTemp      = preferences.getInt("uTmp", 0);
  cfg.unitPrecip    = preferences.getInt("uPrc", 0);
  cfg.wxShowCards   = preferences.getBool("wxCard", true);
  cfg.wxFeelsLike   = preferences.getBool("wxFeel", true);
  cfg.wxPrecip      = preferences.getBool("wxPrc", true);
  cfg.wxHumidity    = preferences.getBool("wxHum", true);
  cfg.wxWind        = preferences.getBool("wxWnd", true);
  cfg.wxPressure    = preferences.getBool("wxPrs", true);

  cfg.clockStyle    = preferences.getInt("clkSty", 0);
  cfg.tzPosix       = preferences.getString("tzPos", "KST-9");
  cfg.tzLabel       = preferences.getString("tzLbl", "SEOUL (UTC+9)");

  cfg.brightness    = preferences.getInt("brt", 85);
  preferences.end();
}

void saveConfig() {
  preferences.begin("radar360", false);
  preferences.putString("ssid", cfg.ssid);
  preferences.putString("pass", cfg.pass);
  preferences.putString("host", cfg.hostname);
  preferences.putString("preset", cfg.presetName);
  preferences.putDouble("lat", cfg.lat);
  preferences.putDouble("lon", cfg.lon);
  preferences.putFloat("radKm", cfg.radiusKm);
  preferences.putInt("refSec", cfg.refreshSec);
  preferences.putInt("minAlt", cfg.minAltFt);
  preferences.putInt("maxAlt", cfg.maxAltFt);
  preferences.putInt("maxAc", cfg.maxAc);
  preferences.putInt("mapSty", cfg.mapStyle);
  preferences.putInt("uDist", cfg.unitDist);
  preferences.putInt("uAlt", cfg.unitAlt);
  preferences.putInt("uSpd", cfg.unitSpeed);
  preferences.putInt("viewM", cfg.viewMode);
  preferences.putBool("sMap", cfg.showMapBg);
  preferences.putBool("sScan", cfg.showScanLine);
  preferences.putBool("sCall", cfg.showCallsign);
  preferences.putBool("sAlt", cfg.showAltitude);
  preferences.putBool("sHdg", cfg.showHeading);
  preferences.putBool("sSpd", cfg.showSpeed);
  preferences.putBool("sDst", cfg.showDistance);
  preferences.putBool("sTyp", cfg.showAcType);
  preferences.putBool("sVec", cfg.showVectors);

  preferences.putBool("wxUseR", cfg.wxUseRadarLoc);
  preferences.putDouble("wxLat", cfg.wxLat);
  preferences.putDouble("wxLon", cfg.wxLon);
  preferences.putInt("uTmp", cfg.unitTemp);
  preferences.putInt("uPrc", cfg.unitPrecip);
  preferences.putBool("wxCard", cfg.wxShowCards);
  preferences.putBool("wxFeel", cfg.wxFeelsLike);
  preferences.putBool("wxPrc", cfg.wxPrecip);
  preferences.putBool("wxHum", cfg.wxHumidity);
  preferences.putBool("wxWnd", cfg.wxWind);
  preferences.putBool("wxPrs", cfg.wxPressure);

  preferences.putInt("clkSty", cfg.clockStyle);
  preferences.putString("tzPos", cfg.tzPosix);
  preferences.putString("tzLbl", cfg.tzLabel);

  preferences.putInt("brt", cfg.brightness);
  preferences.end();
}

// ==========================================
// 5. Google Maps 타일 + RainViewer 강수 레이더 다운로드
// ==========================================
double deg2radD(double d) { return d * M_PI / 180.0; }
double rad2degD(double r) { return r * 180.0 / M_PI; }

void latLonToGlobalPx(double lat, double lon, int z, double &px, double &py) {
  double worldSize = 256.0 * (double)(1 << z);
  double latRad = deg2radD(constrain(lat, -85.0511, 85.0511));
  px = ((lon + 180.0) / 360.0) * worldSize;
  py = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / M_PI) / 2.0 * worldSize;
}

int calcAutoZoom(double lat, float radiusKm, bool isWeather) {
  double latRad = deg2radD(lat);
  double worldKmAtLat = 40075.016686 * cos(latRad);
  double idealScale = (worldKmAtLat * (double)R) / (256.0 * (double)radiusKm);
  int z = (int)round(log2(idealScale));
  // ★ RainViewer 무료 타일은 최대 z=7까지만 지원 ("zoom level not supported" 방지)
  if (isWeather) return constrain(z, 5, 7);
  return constrain(z, 6, 12);
}

bool downloadAndDrawTile(WiFiClientSecure &client, const String &url, int drawX, int drawY) {
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.setReuse(false);
  http.addHeader("User-Agent", "Mozilla/5.0");

  bool ok = false;
  if (http.begin(client, url)) {
    int code = http.GET();
    if (code == 200) {
      int len = http.getSize();
      if (len > 100 && len < 180000) {
        uint8_t* buf = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
        if (buf) {
          WiFiClient* stream = http.getStreamPtr();
          int got = 0;
          unsigned long tStart = millis();
          while (got < len && millis() - tStart < 5000) {
            int avail = stream->available();
            if (avail > 0) got += stream->readBytes(buf + got, min(avail, len - got));
            else delay(2);
          }
          if (got == len) {
            if (buf[0] == 0xFF && buf[1] == 0xD8) {
              mapBgSprite.drawJpg(buf, len, drawX, drawY);
              ok = true;
            } else if (buf[0] == 0x89 && buf[1] == 0x50) {
              mapBgSprite.drawPng(buf, len, drawX, drawY);
              ok = true;
            }
          }
          heap_caps_free(buf);
        }
      }
    }
    http.end();
  }
  return ok;
}

String getRainViewerPath(WiFiClientSecure &client) {
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(6000);
  String path = "";
  if (http.begin(client, "https://api.rainviewer.com/public/weather-maps.json")) {
    if (http.GET() == 200) {
      String payload = http.getString();
#if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
#else
      DynamicJsonDocument doc(8192);
#endif
      if (!deserializeJson(doc, payload)) {
        JsonArray past = doc["radar"]["past"].as<JsonArray>();
        if (!past.isNull() && past.size() > 0) {
          path = past[past.size() - 1]["path"].as<String>();
        }
      }
    }
    http.end();
  }
  return path;
}

void fetchMapTiles() {
  if (WiFi.status() != WL_CONNECTED) return;

  bool isWx = (cfg.viewMode == 1);
  double targetLat = (isWx && !cfg.wxUseRadarLoc) ? cfg.wxLat : cfg.lat;
  double targetLon = (isWx && !cfg.wxUseRadarLoc) ? cfg.wxLon : cfg.lon;

  int z = calcAutoZoom(targetLat, cfg.radiusKm, isWx);
  double homePx, homePy;
  latLonToGlobalPx(targetLat, targetLon, z, homePx, homePy);

  int centerTx = (int)floor(homePx / 256.0);
  int centerTy = (int)floor(homePy / 256.0);
  int maxTile  = (1 << z);

  double kmPerPx = (40075.016686 * cos(deg2radD(targetLat))) / (256.0 * (double)maxTile);
  currentMapZoom   = z;
  effectiveRangeKm = (float)(kmPerPx * R);

  mapBgSprite.fillScreen(colSeaBg);
  int okCount = 0;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10);

  String rvPath = isWx ? getRainViewerPath(client) : "";

  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int tx = (centerTx + dx + maxTile) % maxTile;
      int ty = centerTy + dy;
      if (ty < 0 || ty >= maxTile) continue;

      int drawX = CX + (int)round((centerTx + dx) * 256.0 - homePx);
      int drawY = CY + (int)round((centerTy + dy) * 256.0 - homePy);
      if (drawX >= SCREEN_W || drawY >= SCREEN_H || drawX + 256 <= 0 || drawY + 256 <= 0) continue;

      // Google Maps 타일 (y: 위성 하이브리드, m: 로드맵, p: 지형도)
      const char* lyrs = (cfg.mapStyle == 1) ? "m" : ((cfg.mapStyle == 2) ? "p" : "y");
      String url = "https://mt1.google.com/vt/lyrs=" + String(lyrs) + "&hl=ko&x=" + String(tx) + "&y=" + String(ty) + "&z=" + String(z);

      if (downloadAndDrawTile(client, url, drawX, drawY)) {
        okCount++;
      }

      if (isWx && rvPath.length() > 0 && z <= 7) {
        String rvUrl = "https://tilecache.rainviewer.com" + rvPath + "/256/" + String(z) + "/" + String(tx) + "/" + String(ty) + "/2/1_1.png";
        downloadAndDrawTile(client, rvUrl, drawX, drawY);
      }
      delay(15);
    }
  }

  if (okCount > 0) {
    mapLoaded = true;
    Serial.printf("[MAP] Google Maps loaded %d tiles (z=%d, range=%.1fkm)\n", okCount, z, effectiveRangeKm);
  }
}

void mapFetchTask(void *parameter) { fetchMapTiles(); mapFetchInProgress = false; vTaskDelete(NULL); }
void startMapFetchBackground() {
  if (mapFetchInProgress) return;
  mapFetchInProgress = true;
  xTaskCreatePinnedToCore(mapFetchTask, "mapFetchTask", 24576, NULL, 1, NULL, 0);
}

// ==========================================
// 6. RADAR 360 웹 콘솔 HTML (Clock 탭 포함)
// ==========================================
const char WEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RADAR 360 Console</title>
<style>
  :root { --bg:#081210; --card:#0d1c19; --border:#1f3d36; --accent:#5effc3; --text:#e2f5ef; --muted:#7ea398; }
  * { box-sizing:border-box; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--text); font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
  .wrap { max-width:780px; margin:0 auto; }
  .header { display:flex; justify-content:space-between; align-items:center; margin-bottom:16px; }
  .brand { display:flex; align-items:center; gap:12px; }
  .logo { width:40px; height:40px; border-radius:50%; border:2px solid var(--accent); display:flex; align-items:center; justify-content:center; color:var(--accent); font-weight:bold; }
  .brand h1 { margin:0; font-size:20px; letter-spacing:1px; }
  .brand p { margin:2px 0 0; font-size:12px; color:var(--muted); }
  .status { text-align:right; font-size:12px; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; background:#ffd54f; margin-right:5px; }
  .tabs { display:flex; background:var(--card); border:1px solid var(--border); border-radius:10px; padding:4px; margin-bottom:16px; gap:4px; flex-wrap:wrap; }
  .tab { flex:1; min-width:70px; text-align:center; padding:9px 4px; border-radius:7px; cursor:pointer; font-size:13px; color:var(--muted); border:none; background:transparent; }
  .tab.active { background:#16332c; color:var(--accent); font-weight:600; }
  .panel { display:none; background:var(--card); border:1px solid var(--border); border-radius:12px; padding:18px; margin-bottom:16px; }
  .panel.active { display:block; }
  h2 { margin:0 0 14px; font-size:15px; color:var(--accent); }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  @media(max-width:560px){ .grid { grid-template-columns:1fr; } }
  .field { margin-bottom:10px; }
  label { display:block; font-size:12px; color:var(--muted); margin-bottom:5px; }
  input[type=text], input[type=password], input[type=number], select {
    width:100%; padding:10px; border-radius:8px; border:1px solid var(--border);
    background:#07100e; color:var(--text); font-size:14px;
  }
  .chk-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:10px; }
  .chk { display:flex; align-items:center; gap:8px; background:#0a1613; border:1px solid var(--border); padding:9px 10px; border-radius:8px; font-size:13px; cursor:pointer; }
  .chk input { accent-color:var(--accent); width:16px; height:16px; }
  .btn { padding:10px 16px; border-radius:8px; border:1px solid var(--border); background:#112621; color:var(--text); cursor:pointer; font-size:13px; }
  .btn-primary { background:var(--accent); color:#052117; border:none; font-weight:700; padding:12px 22px; font-size:14px; }
  .btn-danger { background:#3d1616; border-color:#7a2929; color:#ff9e9e; }
  .scan-list { margin-top:10px; max-height:150px; overflow-y:auto; border:1px solid var(--border); border-radius:8px; display:none; }
  .scan-item { padding:8px 12px; border-bottom:1px solid var(--border); cursor:pointer; font-size:13px; display:flex; justify-content:space-between; }
  .scan-item:hover { background:#16332c; }
  .hint { font-size:11px; color:var(--muted); margin-top:4px; }
  .toast { position:fixed; bottom:20px; right:20px; background:var(--accent); color:#052117; padding:10px 16px; border-radius:8px; font-weight:700; display:none; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <div class="brand">
      <div class="logo">360</div>
      <div>
        <h1>RADAR 360</h1>
        <p>Flight Radar · Live Weather · Luxury Clock</p>
      </div>
    </div>
    <div class="status">
      <div><span class="dot" id="stDot"></span><span id="stMode">Loading...</span></div>
      <div id="stIp" style="color:var(--muted)"></div>
    </div>
  </div>

  <div class="tabs">
    <button class="tab active" onclick="openTab('net',this)">Network</button>
    <button class="tab" onclick="openTab('radar',this)">Radar</button>
    <button class="tab" onclick="openTab('wx',this)">Weather</button>
    <button class="tab" onclick="openTab('clk',this)">Clock</button>
    <button class="tab" onclick="openTab('sys',this)">System</button>
  </div>

  <form id="cfgForm" onsubmit="saveAll(event)">
    <!-- NETWORK -->
    <div id="tab-net" class="panel active">
      <h2>WI-FI Setup</h2>
      <div class="grid">
        <div class="field"><label>Network name (SSID)</label><input type="text" id="ssid" name="ssid"></div>
        <div class="field"><label>Password</label><input type="password" id="pass" name="pass" placeholder="Leave blank to keep current password"></div>
      </div>
      <div class="field">
        <label>Device hostname</label>
        <input type="text" id="hostname" name="hostname" value="radar">
        <div class="hint">접속 주소: http://<span id="hostPreview">radar</span>.local</div>
      </div>
      <button type="button" class="btn" onclick="scanWifi()">Scan networks</button>
      <span class="hint" style="margin-left:8px;">Wi-Fi 또는 호스트명 변경 시 재부팅됩니다.</span>
      <div id="scanList" class="scan-list"></div>
    </div>

    <!-- RADAR -->
    <div id="tab-radar" class="panel">
      <h2>Location & Google Maps</h2>
      <div class="grid">
        <div class="field">
          <label>Airport Presets</label>
          <select id="preset" name="preset" onchange="applyPreset()">
            <optgroup label="대한민국 (Korea)">
              <option value="INCHEON">인천국제공항 (RKSI)</option>
              <option value="BUSAN">부산 김해국제공항 (RKPK)</option>
              <option value="JEJU">제주국제공항 (RKPC)</option>
            </optgroup>
            <optgroup label="아시아 · 오세아니아">
              <option value="NARITA">도쿄 나리타 (RJAA)</option>
              <option value="HANEDA">도쿄 하네다 (RJTT)</option>
              <option value="SINGAPORE">싱가포르 창이 (WSSS)</option>
              <option value="HONGKONG">홍콩 (VHHH)</option>
              <option value="BANGKOK">방콕 수완나품 (VTBS)</option>
              <option value="DUBAI">두바이 (OMDB)</option>
              <option value="SYDNEY">시드니 (YSSY)</option>
            </optgroup>
            <optgroup label="유럽 · 미주">
              <option value="LONDON">런던 히드로 (EGLL)</option>
              <option value="PARIS">파리 샤를드골 (LFPG)</option>
              <option value="FRANKFURT">프랑크푸르트 (EDDF)</option>
              <option value="NEWYORK">뉴욕 JFK (KJFK)</option>
              <option value="LA">로스앤젤레스 (KLAX)</option>
            </optgroup>
            <option value="CUSTOM">직접 입력 (Custom)</option>
          </select>
        </div>
        <div class="field">
          <label>Google Map style</label>
          <select id="mapStyle" name="mapStyle">
            <option value="0">Google Satellite Hybrid (위성 지도)</option>
            <option value="1">Google Roadmap (일반 지도)</option>
            <option value="2">Google Terrain (지형 지도)</option>
          </select>
        </div>
        <div class="field"><label>Latitude (WGS-84)</label><input type="number" id="lat" name="lat" step="0.0001"></div>
        <div class="field"><label>Longitude (WGS-84)</label><input type="number" id="lon" name="lon" step="0.0001"></div>
        <div class="field"><label>Radius (km, 10~200)</label><input type="number" id="radiusKm" name="radiusKm" min="10" max="200"></div>
        <div class="field"><label>Refresh interval (sec)</label><input type="number" id="refreshSec" name="refreshSec" min="5" max="60"></div>
        <div class="field"><label>Minimum altitude (ft)</label><input type="number" id="minAltFt" name="minAltFt" min="0" max="60000" step="500"></div>
        <div class="field"><label>Maximum altitude (ft)</label><input type="number" id="maxAltFt" name="maxAltFt" min="1000" max="60000" step="500"></div>
        <div class="field"><label>Maximum aircraft (1~40)</label><input type="number" id="maxAc" name="maxAc" min="1" max="40"></div>
      </div>

      <h2 style="margin-top:18px;">Display & Units</h2>
      <div class="grid">
        <div class="field"><label>Distance</label><select id="unitDist" name="unitDist"><option value="0">Kilometers (km)</option><option value="1">Nautical miles (nm)</option></select></div>
        <div class="field"><label>Altitude</label><select id="unitAlt" name="unitAlt"><option value="1">Feet (ft)</option><option value="0">Meters (m)</option></select></div>
        <div class="field"><label>Speed</label><select id="unitSpeed" name="unitSpeed"><option value="1">Knots (kt)</option><option value="0">km/h</option></select></div>
        <div class="field">
          <label>Startup / Active View</label>
          <select id="viewMode" name="viewMode" onchange="syncViewMode(this.value)">
            <option value="0">Flight radar (항공기 레이더)</option>
            <option value="1">Weather view (실시간 날씨)</option>
            <option value="2">Clock view (시계 화면)</option>
          </select>
        </div>
      </div>

      <div class="chk-grid">
        <label class="chk"><input type="checkbox" id="showMapBg" name="showMapBg"> Flight map background</label>
        <label class="chk"><input type="checkbox" id="showScanLine" name="showScanLine"> Scan line</label>
        <label class="chk"><input type="checkbox" id="showCallsign" name="showCallsign"> Callsign</label>
        <label class="chk"><input type="checkbox" id="showAltitude" name="showAltitude"> Altitude</label>
        <label class="chk"><input type="checkbox" id="showHeading" name="showHeading"> Heading</label>
        <label class="chk"><input type="checkbox" id="showSpeed" name="showSpeed"> Speed</label>
        <label class="chk"><input type="checkbox" id="showDistance" name="showDistance"> Distance</label>
        <label class="chk"><input type="checkbox" id="showAcType" name="showAcType"> Aircraft type</label>
        <label class="chk"><input type="checkbox" id="showVectors" name="showVectors"> Heading vectors</label>
      </div>
    </div>

    <!-- WEATHER -->
    <div id="tab-wx" class="panel">
      <h2>Display Mode</h2>
      <div class="field">
        <label>화면 표시 모드 선택</label>
        <select id="wxViewMode" onchange="syncViewMode(this.value)">
          <option value="0">Flight radar (항공기 레이더)</option>
          <option value="1">Weather view (실시간 날씨 + 강수 레이더)</option>
          <option value="2">Clock view (시계 화면)</option>
        </select>
      </div>

      <h2 style="margin-top:18px;">Weather Location</h2>
      <label class="chk" style="margin-bottom:12px;"><input type="checkbox" id="wxUseRadarLoc" name="wxUseRadarLoc" onchange="toggleWxLoc()"> Use radar location</label>
      <div class="grid">
        <div class="field"><label>Weather latitude (WGS-84)</label><input type="number" id="wxLat" name="wxLat" step="0.0001"></div>
        <div class="field"><label>Weather longitude (WGS-84)</label><input type="number" id="wxLon" name="wxLon" step="0.0001"></div>
      </div>

      <h2 style="margin-top:18px;">Weather Cards</h2>
      <div class="grid">
        <div class="field"><label>Temperature unit</label><select id="unitTemp" name="unitTemp"><option value="0">Celsius °C</option><option value="1">Fahrenheit °F</option></select></div>
        <div class="field"><label>Precipitation unit</label><select id="unitPrecip" name="unitPrecip"><option value="0">Millimeters mm</option><option value="1">Inches in</option></select></div>
      </div>
      <div class="chk-grid">
        <label class="chk"><input type="checkbox" id="wxShowCards" name="wxShowCards"> Show cards on radar</label>
        <label class="chk"><input type="checkbox" id="wxFeelsLike" name="wxFeelsLike"> Feels like</label>
        <label class="chk"><input type="checkbox" id="wxPrecip" name="wxPrecip"> Current precipitation</label>
        <label class="chk"><input type="checkbox" id="wxHumidity" name="wxHumidity"> Humidity</label>
        <label class="chk"><input type="checkbox" id="wxWind" name="wxWind"> Wind direction & speed</label>
        <label class="chk"><input type="checkbox" id="wxPressure" name="wxPressure"> Pressure</label>
      </div>
    </div>

    <!-- CLOCK -->
    <div id="tab-clk" class="panel">
      <h2>Clock Display Mode</h2>
      <div class="field">
        <label>화면 표시 모드 선택</label>
        <select id="clkViewMode" onchange="syncViewMode(this.value)">
          <option value="0">Flight radar (항공기 레이더)</option>
          <option value="1">Weather view (실시간 날씨)</option>
          <option value="2">Clock view (시계 화면)</option>
        </select>
      </div>

      <h2 style="margin-top:18px;">Watch Face & Timezone</h2>
      <div class="grid">
        <div class="field">
          <label>시계 화면 스타일 선택</label>
          <select id="clockStyle" name="clockStyle">
            <optgroup label="아날로그 명품 시계 (Analog)">
              <option value="0">ROLEX (Submariner Black)</option>
              <option value="1">LONGINES (Master Classic White)</option>
              <option value="2">BREITLING (Navitimer Chrono)</option>
              <option value="3">OMEGA (Seamaster Deep Blue)</option>
            </optgroup>
            <optgroup label="디지털 시계 (Digital)">
              <option value="4">Digital Modern (시간 + 날짜 + 기온)</option>
              <option value="5">Digital Neon Ring (원형 초침 게이지)</option>
              <option value="6">Digital Retro Flip (플립 시계 스타일)</option>
            </optgroup>
          </select>
        </div>
        <div class="field">
          <label>지역별 시간대 (Timezone)</label>
          <select id="tzPosix" name="tzPosix" onchange="updateTzLabel()">
            <option value="KST-9" data-lbl="SEOUL (UTC+9)">대한민국 서울/부산 (UTC+9)</option>
            <option value="JST-9" data-lbl="TOKYO (UTC+9)">일본 도쿄 (UTC+9)</option>
            <option value="SGT-8" data-lbl="SINGAPORE (UTC+8)">싱가포르 / 홍콩 (UTC+8)</option>
            <option value="ICT-7" data-lbl="BANGKOK (UTC+7)">태국 방콕 (UTC+7)</option>
            <option value="GST-4" data-lbl="DUBAI (UTC+4)">아랍에미리트 두바이 (UTC+4)</option>
            <option value="GMT0"  data-lbl="LONDON (UTC+0)">영국 런던 (UTC+0)</option>
            <option value="CET-1" data-lbl="PARIS (UTC+1)">프랑스 파리 / 독일 (UTC+1)</option>
            <option value="EST5"  data-lbl="NEW YORK (UTC-5)">미국 뉴욕 (UTC-5)</option>
            <option value="PST8"  data-lbl="LOS ANGELES (UTC-8)">미국 LA (UTC-8)</option>
            <option value="AEST-10" data-lbl="SYDNEY (UTC+10)">호주 시드니 (UTC+10)</option>
          </select>
          <input type="hidden" id="tzLabel" name="tzLabel" value="SEOUL (UTC+9)">
        </div>
      </div>
    </div>

    <!-- SYSTEM -->
    <div id="tab-sys" class="panel">
      <h2>Display</h2>
      <div class="field">
        <label>Day brightness: <span id="brtVal">85</span>%</label>
        <input type="range" id="brightness" name="brightness" min="10" max="100" style="width:100%;accent-color:var(--accent);"
               oninput="document.getElementById('brtVal').innerText=this.value">
      </div>
      <h2 style="margin-top:20px;">Device</h2>
      <p class="hint" style="margin-bottom:12px;">Factory reset은 저장된 Wi-Fi 및 모든 설정을 초기화하고 설정 모드로 재부팅합니다.</p>
      <button type="button" class="btn" onclick="deviceAction('/api/restart','재부팅 하시겠습니까?')">Restart</button>
      <button type="button" class="btn btn-danger" style="margin-left:8px;" onclick="deviceAction('/api/reset','모든 설정을 초기화하시겠습니까?')">Factory reset</button>
    </div>

    <button type="submit" class="btn btn-primary">Save all settings</button>
  </form>
</div>
<div id="toast" class="toast">Saved!</div>

<script>
const PRESETS = {
  INCHEON:[37.4602,126.4407], BUSAN:[35.1795,128.9382], JEJU:[33.5113,126.4930],
  NARITA:[35.7720,140.3929], HANEDA:[35.5494,139.7798], SINGAPORE:[1.3644,103.9915],
  HONGKONG:[22.3080,113.9185], BANGKOK:[13.6900,100.7501], DUBAI:[25.2532,55.3657],
  SYDNEY:[-33.9399,151.1753], LONDON:[51.4700,-0.4543], PARIS:[49.0097,2.5479],
  FRANKFURT:[50.0379,8.5622], NEWYORK:[40.6413,-73.7781], LA:[33.9416,-118.4085]
};
const CHKS = ['showMapBg','showScanLine','showCallsign','showAltitude','showHeading','showSpeed','showDistance','showAcType','showVectors',
              'wxUseRadarLoc','wxShowCards','wxFeelsLike','wxPrecip','wxHumidity','wxWind','wxPressure'];

function openTab(id, el) {
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById('tab-' + id).classList.add('active');
  el.classList.add('active');
}
function syncViewMode(v) {
  document.getElementById('viewMode').value = v;
  document.getElementById('wxViewMode').value = v;
  document.getElementById('clkViewMode').value = v;
}
function updateTzLabel() {
  const sel = document.getElementById('tzPosix');
  const opt = sel.options[sel.selectedIndex];
  document.getElementById('tzLabel').value = opt.getAttribute('data-lbl') || 'LOCAL';
}
function applyPreset() {
  const k = document.getElementById('preset').value;
  if (PRESETS[k]) {
    document.getElementById('lat').value = PRESETS[k][0];
    document.getElementById('lon').value = PRESETS[k][1];
    if (document.getElementById('wxUseRadarLoc').checked) {
      document.getElementById('wxLat').value = PRESETS[k][0];
      document.getElementById('wxLon').value = PRESETS[k][1];
    }
  }
}
function toggleWxLoc() {
  const u = document.getElementById('wxUseRadarLoc').checked;
  document.getElementById('wxLat').disabled = u;
  document.getElementById('wxLon').disabled = u;
  if (u) {
    document.getElementById('wxLat').value = document.getElementById('lat').value;
    document.getElementById('wxLon').value = document.getElementById('lon').value;
  }
}
document.getElementById('hostname').addEventListener('input', e => {
  document.getElementById('hostPreview').innerText = e.target.value || 'radar';
});

async function loadState() {
  const r = await fetch('/api/config');
  const d = await r.json();
  document.getElementById('stMode').innerText = d.apMode ? 'Wi-Fi setup' : 'Connected';
  document.getElementById('stDot').style.background = d.apMode ? '#ffd54f' : '#5effc3';
  document.getElementById('stIp').innerText = d.ip;

  ['ssid','hostname','preset','mapStyle','lat','lon','radiusKm','refreshSec','minAltFt','maxAltFt','maxAc',
   'unitDist','unitAlt','unitSpeed','viewMode','wxLat','wxLon','unitTemp','unitPrecip','clockStyle','tzPosix','tzLabel','brightness']
    .forEach(k => { if (document.getElementById(k)) document.getElementById(k).value = d[k]; });
  syncViewMode(d.viewMode);
  document.getElementById('hostPreview').innerText = d.hostname;
  document.getElementById('brtVal').innerText = d.brightness;
  CHKS.forEach(k => document.getElementById(k).checked = d[k]);
  toggleWxLoc();
}

async function scanWifi() {
  const list = document.getElementById('scanList');
  list.style.display = 'block';
  list.innerHTML = '<div class="scan-item">스캔 중...</div>';
  const r = await fetch('/api/scan');
  const arr = await r.json();
  if (!arr.length) { list.innerHTML = '<div class="scan-item">검색된 Wi-Fi 없음</div>'; return; }
  list.innerHTML = arr.map(n =>
    `<div class="scan-item" onclick="document.getElementById('ssid').value='${n.ssid}'"><span>${n.ssid}</span><span>${n.rssi} dBm</span></div>`
  ).join('');
}

async function saveAll(e) {
  e.preventDefault();
  document.getElementById('wxLat').disabled = false;
  document.getElementById('wxLon').disabled = false;
  updateTzLabel();
  const params = new URLSearchParams();
  const fields = ['ssid','pass','hostname','preset','mapStyle','lat','lon','radiusKm','refreshSec','minAltFt','maxAltFt','maxAc',
                  'unitDist','unitAlt','unitSpeed','viewMode','wxLat','wxLon','unitTemp','unitPrecip','clockStyle','tzPosix','tzLabel','brightness'];
  fields.forEach(k => {
    const el = document.getElementById(k);
    if (el) params.append(k, el.value);
  });
  CHKS.forEach(k => params.append(k, document.getElementById(k).checked ? '1' : '0'));
  toggleWxLoc();

  const r = await fetch('/api/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: params.toString()
  });
  const res = await r.json();
  const t = document.getElementById('toast');
  t.innerText = res.reboot ? '저장 완료! 재부팅 중...' : '설정이 즉시 적용되었습니다!';
  t.style.display = 'block';
  setTimeout(() => t.style.display = 'none', 3000);
}

async function deviceAction(url, msg) {
  if (!confirm(msg)) return;
  await fetch(url, { method:'POST' });
  alert('요청을 처리했습니다. 기기가 재부팅됩니다.');
}
loadState();
</script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", WEB_HTML); });

  server.on("/api/config", HTTP_GET, []() {
    String j = "{";
    j += "\"apMode\":" + String(isApMode ? "true" : "false") + ",";
    j += "\"ip\":\"" + (isApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    j += "\"ssid\":\"" + cfg.ssid + "\",\"hostname\":\"" + cfg.hostname + "\",\"preset\":\"" + cfg.presetName + "\",";
    j += "\"mapStyle\":" + String(cfg.mapStyle) + ",\"lat\":" + String(cfg.lat, 4) + ",\"lon\":" + String(cfg.lon, 4) + ",\"radiusKm\":" + String(cfg.radiusKm, 0) + ",";
    j += "\"refreshSec\":" + String(cfg.refreshSec) + ",\"minAltFt\":" + String(cfg.minAltFt) + ",\"maxAltFt\":" + String(cfg.maxAltFt) + ",\"maxAc\":" + String(cfg.maxAc) + ",";
    j += "\"unitDist\":" + String(cfg.unitDist) + ",\"unitAlt\":" + String(cfg.unitAlt) + ",\"unitSpeed\":" + String(cfg.unitSpeed) + ",\"viewMode\":" + String(cfg.viewMode) + ",";
    j += "\"showMapBg\":" + String(cfg.showMapBg ? "true" : "false") + ",\"showScanLine\":" + String(cfg.showScanLine ? "true" : "false") + ",";
    j += "\"showCallsign\":" + String(cfg.showCallsign ? "true" : "false") + ",\"showAltitude\":" + String(cfg.showAltitude ? "true" : "false") + ",";
    j += "\"showHeading\":" + String(cfg.showHeading ? "true" : "false") + ",\"showSpeed\":" + String(cfg.showSpeed ? "true" : "false") + ",";
    j += "\"showDistance\":" + String(cfg.showDistance ? "true" : "false") + ",\"showAcType\":" + String(cfg.showAcType ? "true" : "false") + ",";
    j += "\"showVectors\":" + String(cfg.showVectors ? "true" : "false") + ",";
    j += "\"wxUseRadarLoc\":" + String(cfg.wxUseRadarLoc ? "true" : "false") + ",\"wxLat\":" + String(cfg.wxLat, 4) + ",\"wxLon\":" + String(cfg.wxLon, 4) + ",";
    j += "\"unitTemp\":" + String(cfg.unitTemp) + ",\"unitPrecip\":" + String(cfg.unitPrecip) + ",";
    j += "\"wxShowCards\":" + String(cfg.wxShowCards ? "true" : "false") + ",\"wxFeelsLike\":" + String(cfg.wxFeelsLike ? "true" : "false") + ",";
    j += "\"wxPrecip\":" + String(cfg.wxPrecip ? "true" : "false") + ",\"wxHumidity\":" + String(cfg.wxHumidity ? "true" : "false") + ",";
    j += "\"wxWind\":" + String(cfg.wxWind ? "true" : "false") + ",\"wxPressure\":" + String(cfg.wxPressure ? "true" : "false") + ",";
    j += "\"clockStyle\":" + String(cfg.clockStyle) + ",\"tzPosix\":\"" + cfg.tzPosix + "\",\"tzLabel\":\"" + cfg.tzLabel + "\",";
    j += "\"brightness\":" + String(cfg.brightness) + "}";
    server.send(200, "application/json", j);
  });

  server.on("/api/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    String j = "[";
    bool first = true;
    for (int i = 0; i < n; i++) {
      String s = WiFi.SSID(i);
      if (s.length() == 0) continue;
      if (!first) j += ",";
      s.replace("\"", "");
      j += "{\"ssid\":\"" + s + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
      first = false;
    }
    j += "]";
    server.send(200, "application/json", j);
  });

  server.on("/api/save", HTTP_POST, []() {
    bool needReboot = false;
    bool mapChanged = false;
    double oldLat = cfg.lat, oldLon = cfg.lon;
    float  oldRad = cfg.radiusKm;
    int    oldSty = cfg.mapStyle;
    int    oldView = cfg.viewMode;

    String newSsid = server.arg("ssid"); newSsid.trim();
    String newPass = server.arg("pass");
    String newHost = server.arg("hostname"); newHost.trim();
    if (newHost.length() == 0) newHost = "radar";

    if (newSsid != cfg.ssid || newHost != cfg.hostname || newPass.length() > 0) needReboot = true;
    if (newSsid.length() > 0) cfg.ssid = newSsid;
    if (newPass.length() > 0) cfg.pass = newPass;
    cfg.hostname = newHost;

    if (server.hasArg("preset"))     cfg.presetName = server.arg("preset");
    if (server.hasArg("mapStyle"))   cfg.mapStyle   = server.arg("mapStyle").toInt();
    if (server.hasArg("lat"))        cfg.lat        = server.arg("lat").toDouble();
    if (server.hasArg("lon"))        cfg.lon        = server.arg("lon").toDouble();
    if (server.hasArg("radiusKm"))   cfg.radiusKm   = constrain(server.arg("radiusKm").toFloat(), 10.0f, 200.0f);
    if (server.hasArg("refreshSec")) cfg.refreshSec = constrain(server.arg("refreshSec").toInt(), 5, 60);
    if (server.hasArg("minAltFt"))   cfg.minAltFt   = server.arg("minAltFt").toInt();
    if (server.hasArg("maxAltFt"))   cfg.maxAltFt   = server.arg("maxAltFt").toInt();
    if (server.hasArg("maxAc"))      cfg.maxAc      = constrain(server.arg("maxAc").toInt(), 1, 40);
    if (server.hasArg("unitDist"))   cfg.unitDist   = server.arg("unitDist").toInt();
    if (server.hasArg("unitAlt"))    cfg.unitAlt    = server.arg("unitAlt").toInt();
    if (server.hasArg("unitSpeed"))  cfg.unitSpeed  = server.arg("unitSpeed").toInt();
    if (server.hasArg("viewMode"))   cfg.viewMode   = server.arg("viewMode").toInt();

    cfg.showMapBg     = (server.arg("showMapBg") == "1");
    cfg.showScanLine  = (server.arg("showScanLine") == "1");
    cfg.showCallsign  = (server.arg("showCallsign") == "1");
    cfg.showAltitude  = (server.arg("showAltitude") == "1");
    cfg.showHeading   = (server.arg("showHeading") == "1");
    cfg.showSpeed     = (server.arg("showSpeed") == "1");
    cfg.showDistance  = (server.arg("showDistance") == "1");
    cfg.showAcType    = (server.arg("showAcType") == "1");
    cfg.showVectors   = (server.arg("showVectors") == "1");

    cfg.wxUseRadarLoc = (server.arg("wxUseRadarLoc") == "1");
    cfg.wxLat         = cfg.wxUseRadarLoc ? cfg.lat : server.arg("wxLat").toDouble();
    cfg.wxLon         = cfg.wxUseRadarLoc ? cfg.lon : server.arg("wxLon").toDouble();
    if (server.hasArg("unitTemp"))   cfg.unitTemp   = server.arg("unitTemp").toInt();
    if (server.hasArg("unitPrecip")) cfg.unitPrecip = server.arg("unitPrecip").toInt();
    cfg.wxShowCards   = (server.arg("wxShowCards") == "1");
    cfg.wxFeelsLike   = (server.arg("wxFeelsLike") == "1");
    cfg.wxPrecip      = (server.arg("wxPrecip") == "1");
    cfg.wxHumidity    = (server.arg("wxHumidity") == "1");
    cfg.wxWind        = (server.arg("wxWind") == "1");
    cfg.wxPressure    = (server.arg("wxPressure") == "1");

    if (server.hasArg("clockStyle")) cfg.clockStyle = server.arg("clockStyle").toInt();
    if (server.hasArg("tzPosix"))    cfg.tzPosix    = server.arg("tzPosix");
    if (server.hasArg("tzLabel"))    cfg.tzLabel    = server.arg("tzLabel");
    if (server.hasArg("brightness")) cfg.brightness = constrain(server.arg("brightness").toInt(), 10, 100);

    if (fabs(oldLat - cfg.lat) > 0.0001 || fabs(oldLon - cfg.lon) > 0.0001 ||
        fabs(oldRad - cfg.radiusKm) > 1.0f || oldSty != cfg.mapStyle || oldView != cfg.viewMode) {
      mapChanged = true;
    }

    saveConfig();
    lcd.setBrightness(map(cfg.brightness, 10, 100, 25, 255));
    configTzTime(cfg.tzPosix.c_str(), "pool.ntp.org", "time.nist.gov");

    if (!isApMode) {
      if (mapChanged && cfg.viewMode != 2) startMapFetchBackground();
      if (cfg.viewMode == 0) startAircraftFetchBackground();
      startWeatherFetchBackground();
    }

    server.send(200, "application/json", needReboot ? "{\"reboot\":true}" : "{\"reboot\":false}");
    if (needReboot || isApMode) { delay(1200); ESP.restart(); }
  });

  server.on("/api/restart", HTTP_POST, []() { server.send(200, "text/plain", "OK"); delay(500); ESP.restart(); });
  server.on("/api/reset", HTTP_POST, []() {
    preferences.begin("radar360", false); preferences.clear(); preferences.end();
    server.send(200, "text/plain", "OK"); delay(500); ESP.restart();
  });
  server.onNotFound([]() {
    if (isApMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true); server.send(302, "text/plain", ""); }
    else server.send(404, "text/plain", "Not Found");
  });
  server.begin();
}

void runApSetupPortal() {
  isApMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID);
  delay(200);
  IPAddress apIP = WiFi.softAPIP();

  lcd.fillScreen(TFT_BLACK);
  lcd.drawRect(12, 12, 456, 456, greenBright);
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(3); lcd.setTextColor(TFT_YELLOW, TFT_BLACK); lcd.drawString("RADAR 360 SETUP", 240, 80);
  lcd.setTextSize(2); lcd.setTextColor(TFT_WHITE, TFT_BLACK);  lcd.drawString("1. Connect Wi-Fi to:", 240, 155);
  lcd.setTextSize(3); lcd.setTextColor(greenBright, TFT_BLACK);lcd.drawString(AP_SSID, 240, 200);
  lcd.setTextSize(2); lcd.setTextColor(TFT_WHITE, TFT_BLACK);  lcd.drawString("2. Open Browser:", 240, 275);
  lcd.setTextSize(3); lcd.setTextColor(colRiver, TFT_BLACK);   lcd.drawString("http://" + apIP.toString(), 240, 320);
  lcd.setTextSize(2); lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK); lcd.drawString("Waiting for Web Config...", 240, 405);

  dnsServer.start(53, "*", apIP);
  setupWebServer();
  while (true) { dnsServer.processNextRequest(); server.handleClient(); delay(5); }
}

// ==========================================
// 7. 실시간 날씨 + 3일 예보 API (Open-Meteo)
// ==========================================
const char* degToCompass(int deg) {
  const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[((deg + 22) % 360) / 45];
}

void fetchWeatherData() {
  if (WiFi.status() != WL_CONNECTED) return;
  double lat = cfg.wxUseRadarLoc ? cfg.lat : cfg.wxLat;
  double lon = cfg.wxUseRadarLoc ? cfg.lon : cfg.wxLon;

  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(lat, 4) +
               "&longitude=" + String(lon, 4) +
               "&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,cloud_cover,surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m"
               "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&forecast_days=3&timezone=auto";

  WiFiClientSecure client; client.setInsecure(); client.setTimeout(12);
  HTTPClient http; http.setConnectTimeout(8000); http.setTimeout(12000);
  if (!http.begin(client, url)) return;

  if (http.GET() == 200) {
    String payload = http.getString();
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<2560> doc;
#endif
    if (!deserializeJson(doc, payload)) {
      JsonObject cur = doc["current"];
      JsonObject dly = doc["daily"];
      WeatherData tmp;
      tmp.valid       = true;
      tmp.tempC       = cur["temperature_2m"] | 0.0f;
      tmp.feelsLikeC  = cur["apparent_temperature"] | 0.0f;
      tmp.humidity    = cur["relative_humidity_2m"] | 0;
      tmp.cloudCover  = cur["cloud_cover"] | 0;
      tmp.precipMm    = cur["precipitation"] | 0.0f;
      tmp.windKmh     = cur["wind_speed_10m"] | 0.0f;
      tmp.gustKmh     = cur["wind_gusts_10m"] | 0.0f;
      tmp.windDir     = cur["wind_direction_10m"] | 0;
      tmp.pressureHpa = cur["surface_pressure"] | 1013.0f;
      tmp.wmoCode     = cur["weather_code"] | 0;

      for (int i = 0; i < 3; i++) {
        tmp.dayCode[i] = dly["weather_code"][i] | 0;
        tmp.dayMaxC[i] = dly["temperature_2m_max"][i] | 0.0f;
        tmp.dayMinC[i] = dly["temperature_2m_min"][i] | 0.0f;
        tmp.dayPop[i]  = dly["precipitation_probability_max"][i] | 0;
      }
      noInterrupts(); pendingWxData = tmp; pendingWxReady = true; interrupts();
    }
  }
  http.end();
}

void weatherFetchTask(void *parameter) { fetchWeatherData(); wxFetchInProgress = false; vTaskDelete(NULL); }
void startWeatherFetchBackground() {
  if (wxFetchInProgress) return;
  wxFetchInProgress = true;
  xTaskCreatePinnedToCore(weatherFetchTask, "weatherFetchTask", 16384, NULL, 1, NULL, 0);
}
void applyPendingWeatherData() {
  if (!pendingWxReady) return;
  noInterrupts(); wxData = pendingWxData; pendingWxReady = false; interrupts();
}

// ==========================================
// 8. 전 세계 주요 공항 마커 및 ADS-B 수신
// ==========================================
const char* AIRPORT_CODES[] = {
  "RKSI","RKPK","RKPC","RJAA","RJTT","WSSS","VHHH","VTBS","OMDB","YSSY","EGLL","LFPG","EDDF","KJFK","KLAX"
};
const double AIRPORT_COORDS[][2] = {
  {37.4602,126.4407},{35.1795,128.9382},{33.5113,126.4930},{35.7720,140.3929},{35.5494,139.7798},
  {1.3644,103.9915},{22.3080,113.9185},{13.6900,100.7501},{25.2532,55.3657},{-33.9399,151.1753},
  {51.4700,-0.4543},{49.0097,2.5479},{50.0379,8.5622},{40.6413,-73.7781},{33.9416,-118.4085}
};
const int AIRPORTS_LEN = sizeof(AIRPORT_CODES) / sizeof(AIRPORT_CODES[0]);

struct AircraftPoint {
  bool valid; double lat; double lon; float distanceKm; float bearingDeg;
  int altitudeFt; float speedKt; float trackDeg;
  char flight[12]; char typeCode[8];
};

const int MAX_PLANES = 40;
AircraftPoint planes[MAX_PLANES], pendingPlanes[MAX_PLANES];
int planeCount = 0, pendingPlaneCount = 0;
bool apiOk = false, pendingApiOk = false, fetchInProgress = false, pendingDataReady = false;
String apiStatusMsg = "WAIT...";

float angleDiffBehindSweep(float s, float b) {
  float d = s - b; while (d < 0) d += 360; while (d >= 360) d -= 360; return d;
}
float distanceKm(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2radD(lat2 - lat1), dLon = deg2radD(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2radD(lat1)) * cos(deg2radD(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return 6371.0 * (2 * atan2(sqrt(a), sqrt(1 - a)));
}
float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2radD(lon2 - lon1)) * cos(deg2radD(lat2));
  double x = cos(deg2radD(lat1)) * sin(deg2radD(lat2)) - sin(deg2radD(lat1)) * cos(deg2radD(lat2)) * cos(deg2radD(lon2 - lon1));
  double b = rad2degD(atan2(y, x)); return (b < 0) ? b + 360.0 : b;
}

void geoToScreen(double lat, double lon, int &outX, int &outY) {
  if (mapLoaded) {
    double hPx, hPy, tPx, tPy;
    latLonToGlobalPx(cfg.lat, cfg.lon, currentMapZoom, hPx, hPy);
    latLonToGlobalPx(lat, lon, currentMapZoom, tPx, tPy);
    outX = CX + (int)round(tPx - hPx);
    outY = CY + (int)round(tPy - hPy);
  } else {
    float rr = (distanceKm(cfg.lat, cfg.lon, lat, lon) / cfg.radiusKm) * R;
    float rad = (bearingDeg(cfg.lat, cfg.lon, lat, lon) - 90.0) * DEG_TO_RAD;
    outX = CX + cos(rad) * rr;
    outY = CY + sin(rad) * rr;
  }
}

void canvasCorrectedRadialLine(int angleDeg, int r1, int r2, uint16_t col) {
  float rad = (angleDeg - 90) * DEG_TO_RAD;
  radarCanvas.drawLine(CX + cos(rad) * r1, CY + sin(rad) * r1, CX + cos(rad) * r2, CY + sin(rad) * r2, col);
}

bool tryFetchFromUrl(const String &url, const char* serverTag) {
  WiFiClientSecure client; client.setInsecure(); client.setTimeout(15);
  HTTPClient http;
  http.setConnectTimeout(10000); http.setTimeout(15000); http.setReuse(false);
  http.addHeader("User-Agent", "ESP32-S3-Radar360/1.0");
  http.addHeader("Accept", "application/json");
  http.addHeader("Accept-Encoding", "identity");

  if (!http.begin(client, url)) { apiStatusMsg = "BEGIN ERR"; return false; }
  int code = http.GET();
  if (code != 200) { apiStatusMsg = "HTTP " + String(code); http.end(); return false; }
  String payload = http.getString(); http.end();
  if (payload.length() < 20) { apiStatusMsg = "EMPTY"; return false; }

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument filter;
  JsonObject acF = filter["ac"][0].to<JsonObject>();
  acF["lat"]=true; acF["lon"]=true; acF["alt_baro"]=true; acF["alt_geom"]=true; acF["gs"]=true; acF["track"]=true; acF["flight"]=true; acF["t"]=true;
  JsonObject airF = filter["aircraft"][0].to<JsonObject>();
  airF["lat"]=true; airF["lon"]=true; airF["alt_baro"]=true; airF["alt_geom"]=true; airF["gs"]=true; airF["track"]=true; airF["flight"]=true; airF["t"]=true;
  SpiRamAllocator allocator; JsonDocument doc(&allocator);
#else
  StaticJsonDocument<512> filter;
  JsonObject acF = filter["ac"].createNestedObject();
  acF["lat"]=true; acF["lon"]=true; acF["alt_baro"]=true; acF["alt_geom"]=true; acF["gs"]=true; acF["track"]=true; acF["flight"]=true; acF["t"]=true;
  JsonObject airF = filter["aircraft"].createNestedObject();
  airF["lat"]=true; airF["lon"]=true; airF["alt_baro"]=true; airF["alt_geom"]=true; airF["gs"]=true; airF["track"]=true; airF["flight"]=true; airF["t"]=true;
  SpiRamJsonDocument doc(131072);
#endif

  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) { apiStatusMsg = "JSON ERR"; return false; }

  AircraftPoint tempPlanes[MAX_PLANES];
  int tempCount = 0, limitCount = min(cfg.maxAc, MAX_PLANES);
  float rangeLim = mapLoaded ? effectiveRangeKm : cfg.radiusKm;
  JsonArray arr = doc["ac"].as<JsonArray>();
  if (arr.isNull() || arr.size() == 0) arr = doc["aircraft"].as<JsonArray>();

  for (JsonObject ac : arr) {
    if (tempCount >= limitCount) break;
    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;
    double lat = ac["lat"], lon = ac["lon"];
    float d = distanceKm(cfg.lat, cfg.lon, lat, lon);
    if (d > rangeLim) continue;

    int alt = ac["alt_baro"].is<int>() ? (int)ac["alt_baro"] : (ac["alt_geom"].is<int>() ? (int)ac["alt_geom"] : -1);
    if (alt >= 0 && (alt < cfg.minAltFt || alt > cfg.maxAltFt)) continue;

    AircraftPoint &p = tempPlanes[tempCount++];
    p.valid = true; p.lat = lat; p.lon = lon;
    p.distanceKm = d; p.bearingDeg = bearingDeg(cfg.lat, cfg.lon, lat, lon);
    p.altitudeFt = alt; p.speedKt = ac["gs"] | 0.0f; p.trackDeg = ac["track"] | 0.0f;
    strncpy(p.flight, ac["flight"] | "", sizeof(p.flight) - 1); p.flight[sizeof(p.flight) - 1] = '\0';
    strncpy(p.typeCode, ac["t"] | "", sizeof(p.typeCode) - 1); p.typeCode[sizeof(p.typeCode) - 1] = '\0';
  }

  noInterrupts();
  pendingPlaneCount = tempCount;
  for (int i = 0; i < tempCount; i++) pendingPlanes[i] = tempPlanes[i];
  pendingApiOk = true; pendingDataReady = true;
  interrupts();
  return true;
}

void fetchAircraftData() {
  if (WiFi.status() != WL_CONNECTED) { apiStatusMsg = "NO WIFI"; pendingApiOk = false; pendingPlaneCount = 0; pendingDataReady = true; return; }
  float rangeLim = mapLoaded ? effectiveRangeKm : cfg.radiusKm;
  int apiRadiusNm = max(10, (int)roundf(rangeLim / 1.852f));
  String path = String(cfg.lat, 4) + "/" + String(cfg.lon, 4) + "/" + String(apiRadiusNm);
  if (tryFetchFromUrl("https://api.adsb.lol/v2/point/" + path, "adsb.lol")) return;
  delay(250);
  if (tryFetchFromUrl("https://opendata.adsb.fi/api/v2/lat/" + String(cfg.lat, 4) + "/lon/" + String(cfg.lon, 4) + "/dist/" + String(apiRadiusNm), "adsb.fi")) return;
  delay(250);
  if (tryFetchFromUrl("https://api.airplanes.live/v2/point/" + path, "airplanes.live")) return;
  pendingApiOk = false; pendingDataReady = true;
}

void aircraftFetchTask(void *parameter) { fetchAircraftData(); fetchInProgress = false; vTaskDelete(NULL); }
void startAircraftFetchBackground() {
  if (fetchInProgress) return;
  fetchInProgress = true;
  xTaskCreatePinnedToCore(aircraftFetchTask, "aircraftFetchTask", 24576, NULL, 1, NULL, 0);
}
void applyPendingAircraftData() {
  if (!pendingDataReady) return;
  noInterrupts();
  planeCount = pendingPlaneCount;
  for (int i = 0; i < planeCount; i++) planes[i] = pendingPlanes[i];
  apiOk = pendingApiOk; pendingDataReady = false;
  interrupts();
}

// ==========================================
// 9. 시계 화면 렌더링 (명품 아날로그 4종 + 디지털 3종)
// ==========================================
void drawThickHand(float angleDeg, int len, int tailLen, int halfW, uint16_t col) {
  float rad = (angleDeg - 90.0f) * DEG_TO_RAD;
  float perp = rad + M_PI / 2.0f;
  int tx = CX + cos(rad) * len, ty = CY + sin(rad) * len;
  int bx = CX - cos(rad) * tailLen, by = CY - sin(rad) * tailLen;
  int lx = bx + cos(perp) * halfW, ly = by + sin(perp) * halfW;
  int rx = bx - cos(perp) * halfW, ry = by - sin(perp) * halfW;
  radarCanvas.fillTriangle(tx, ty, lx, ly, rx, ry, col);
}

void drawClockView() {
  struct tm ti;
  bool hasTime = getLocalTime(&ti);
  int hr = hasTime ? ti.tm_hour : 10;
  int mn = hasTime ? ti.tm_min  : 10;
  int sc = hasTime ? ti.tm_sec  : 30;
  int day = hasTime ? ti.tm_mday : 23;

  float secAng  = sc * 6.0f;
  float minAng  = mn * 6.0f + sc * 0.1f;
  float hourAng = (hr % 12) * 30.0f + mn * 0.5f;

  radarCanvas.fillScreen(TFT_BLACK);

  // ---- [0] ROLEX Submariner ----
  if (cfg.clockStyle == 0) {
    uint16_t bezelCol = radarCanvas.color565(18, 22, 24);
    uint16_t goldCol  = radarCanvas.color565(220, 190, 90);
    radarCanvas.fillCircle(CX, CY, 232, radarCanvas.color565(90, 96, 102));
    radarCanvas.fillCircle(CX, CY, 224, bezelCol);
    radarCanvas.fillCircle(CX, CY, 190, TFT_BLACK);
    radarCanvas.drawCircle(CX, CY, 190, radarCanvas.color565(150, 155, 160));

    // 베젤 눈금
    for (int i = 0; i < 60; i++) {
      float rad = (i * 6 - 90) * DEG_TO_RAD;
      int r1 = (i % 5 == 0) ? 200 : 210;
      radarCanvas.drawLine(CX + cos(rad)*r1, CY + sin(rad)*r1, CX + cos(rad)*218, CY + sin(rad)*218, TFT_WHITE);
    }
    // 다이얼 인덱스
    for (int i = 0; i < 12; i++) {
      float rad = (i * 30 - 90) * DEG_TO_RAD;
      int ix = CX + cos(rad) * 162, iy = CY + sin(rad) * 162;
      if (i == 0) {
        radarCanvas.fillTriangle(CX, CY - 144, CX - 14, CY - 174, CX + 14, CY - 174, TFT_WHITE);
      } else if (i == 3) {
        radarCanvas.fillRoundRect(CX + 132, CY - 16, 40, 32, 5, TFT_WHITE);
        radarCanvas.setTextDatum(middle_center); radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_BLACK);
        radarCanvas.drawNumber(day, CX + 152, CY);
      } else if (i == 6 || i == 9) {
        radarCanvas.fillCircle(ix, iy, 9, TFT_WHITE);
      } else {
        radarCanvas.fillCircle(ix, iy, 8, TFT_WHITE);
        radarCanvas.drawCircle(ix, iy, 9, goldCol);
      }
    }
    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(3); radarCanvas.setTextColor(goldCol); radarCanvas.drawString("ROLEX", CX, CY - 78);
    radarCanvas.setTextSize(1); radarCanvas.setTextColor(TFT_LIGHTGREY);
    radarCanvas.drawString("OYSTER PERPETUAL DATE", CX, CY - 52);
    radarCanvas.drawString("SUBMARINER", CX, CY + 62);
    radarCanvas.drawString(cfg.tzLabel, CX, CY + 82);

    drawThickHand(hourAng, 108, 22, 7, TFT_WHITE);
    drawThickHand(minAng,  156, 26, 5, TFT_WHITE);
    drawThickHand(secAng,  170, 34, 2, greenBright);
    radarCanvas.fillCircle(CX, CY, 8, TFT_WHITE);
  }
  // ---- [1] LONGINES Master Classic ----
  else if (cfg.clockStyle == 1) {
    uint16_t dialCol = radarCanvas.color565(242, 240, 232);
    uint16_t blueHand = radarCanvas.color565(18, 65, 165);
    radarCanvas.fillCircle(CX, CY, 228, radarCanvas.color565(180, 185, 192));
    radarCanvas.fillCircle(CX, CY, 218, dialCol);
    radarCanvas.drawCircle(CX, CY, 198, radarCanvas.color565(120, 120, 120));

    for (int i = 0; i < 60; i++) {
      float rad = (i * 6 - 90) * DEG_TO_RAD;
      radarCanvas.drawLine(CX + cos(rad)*(i%5==0?188:194), CY + sin(rad)*(i%5==0?188:194), CX + cos(rad)*202, CY + sin(rad)*202, TFT_BLACK);
    }
    radarCanvas.setTextDatum(middle_center); radarCanvas.setTextSize(3); radarCanvas.setTextColor(TFT_BLACK);
    for (int i = 1; i <= 12; i++) {
      float rad = (i * 30 - 90) * DEG_TO_RAD;
      radarCanvas.drawNumber(i, CX + cos(rad) * 164, CY + sin(rad) * 164);
    }
    radarCanvas.setTextSize(2); radarCanvas.drawString("LONGINES", CX, CY - 74);
    radarCanvas.setTextSize(1); radarCanvas.drawString("AUTOMATIC", CX, CY + 68);
    radarCanvas.drawString(cfg.tzLabel, CX, CY + 86);

    drawThickHand(hourAng, 112, 20, 6, blueHand);
    drawThickHand(minAng,  162, 24, 4, blueHand);
    drawThickHand(secAng,  176, 32, 2, blueHand);
    radarCanvas.fillCircle(CX, CY, 7, blueHand);
  }
  // ---- [2] BREITLING Navitimer ----
  else if (cfg.clockStyle == 2) {
    uint16_t navyCol = radarCanvas.color565(12, 22, 42);
    uint16_t goldCol = radarCanvas.color565(240, 200, 50);
    radarCanvas.fillCircle(CX, CY, 232, radarCanvas.color565(140, 145, 152));
    radarCanvas.fillCircle(CX, CY, 224, TFT_WHITE); // 외곽 슬라이드 룰
    radarCanvas.fillCircle(CX, CY, 184, navyCol);

    for (int i = 0; i < 60; i++) {
      float rad = (i * 6 - 90) * DEG_TO_RAD;
      radarCanvas.drawLine(CX + cos(rad)*186, CY + sin(rad)*186, CX + cos(rad)*(i%5==0?218:204), CY + sin(rad)*(i%5==0?218:204), TFT_BLACK);
    }
    // 서브 다이얼 3개 (3, 6, 9시)
    radarCanvas.fillCircle(CX - 76, CY, 40, TFT_WHITE); radarCanvas.drawCircle(CX - 76, CY, 40, goldCol);
    radarCanvas.fillCircle(CX + 76, CY, 40, TFT_WHITE); radarCanvas.drawCircle(CX + 76, CY, 40, goldCol);
    radarCanvas.fillCircle(CX, CY + 76, 40, TFT_WHITE); radarCanvas.drawCircle(CX, CY + 76, 40, goldCol);

    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(2); radarCanvas.setTextColor(goldCol); radarCanvas.drawString("BREITLING", CX, CY - 78);
    radarCanvas.setTextSize(1); radarCanvas.setTextColor(TFT_WHITE); radarCanvas.drawString("1884 · NAVITIMER", CX, CY - 56);
    radarCanvas.setTextColor(TFT_BLACK); radarCanvas.drawNumber(day, CX + 76, CY);
    radarCanvas.drawNumber(sc, CX - 76, CY);

    drawThickHand(hourAng, 108, 20, 6, TFT_WHITE);
    drawThickHand(minAng,  156, 24, 4, TFT_WHITE);
    drawThickHand(secAng,  174, 34, 2, TFT_RED);
    radarCanvas.fillCircle(CX, CY, 8, goldCol);
  }
  // ---- [3] OMEGA Seamaster Deep Blue ----
  else if (cfg.clockStyle == 3) {
    uint16_t deepBlue = radarCanvas.color565(10, 38, 86);
    uint16_t waveCol  = radarCanvas.color565(22, 64, 128);
    radarCanvas.fillCircle(CX, CY, 232, radarCanvas.color565(150, 158, 168));
    radarCanvas.fillCircle(CX, CY, 224, radarCanvas.color565(12, 48, 108));
    radarCanvas.fillCircle(CX, CY, 190, deepBlue);

    // 물결 패턴
    for (int y = CY - 140; y <= CY + 140; y += 26) {
      for (int x = CX - 150; x <= CX + 150; x += 6) {
        int wy = y + (int)(sin(x * 0.08f) * 5.0f);
        if (hypot(x - CX, wy - CY) < 182) radarCanvas.drawPixel(x, wy, waveCol);
      }
    }
    for (int i = 0; i < 12; i++) {
      float rad = (i * 30 - 90) * DEG_TO_RAD;
      radarCanvas.fillCircle(CX + cos(rad)*162, CY + sin(rad)*162, (i%3==0)?9:7, TFT_WHITE);
    }
    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(3); radarCanvas.setTextColor(TFT_WHITE); radarCanvas.drawString("OMEGA", CX, CY - 78);
    radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_RED);   radarCanvas.drawString("Seamaster", CX, CY - 48);
    radarCanvas.setTextSize(1); radarCanvas.setTextColor(TFT_LIGHTGREY); radarCanvas.drawString(cfg.tzLabel, CX, CY + 76);

    drawThickHand(hourAng, 106, 20, 7, TFT_WHITE);
    drawThickHand(minAng,  156, 24, 5, TFT_WHITE);
    drawThickHand(secAng,  172, 34, 2, TFT_RED);
    radarCanvas.fillCircle(CX, CY, 8, TFT_WHITE);
  }
  // ---- [4] Digital Modern ----
  else if (cfg.clockStyle == 4) {
    radarCanvas.fillScreen(colCardBg);
    radarCanvas.drawCircle(CX, CY, 228, colRiver);
    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(2); radarCanvas.setTextColor(colRiver);
    radarCanvas.drawString(cfg.tzLabel, CX, 100);

    char hm[8], ss[4], dt[24];
    sprintf(hm, "%02d:%02d", hr, mn);
    sprintf(ss, ":%02d", sc);
    if (hasTime) sprintf(dt, "%04d-%02d-%02d", ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday);
    else strcpy(dt, "---- -- --");

    radarCanvas.setTextSize(7); radarCanvas.setTextColor(TFT_WHITE);
    radarCanvas.drawString(hm, CX - 26, CY - 10);
    radarCanvas.setTextSize(3); radarCanvas.setTextColor(greenBright);
    radarCanvas.drawString(ss, CX + 132, CY + 4);

    radarCanvas.setTextSize(3); radarCanvas.setTextColor(TFT_YELLOW);
    radarCanvas.drawString(dt, CX, CY + 75);

    if (wxData.valid) {
      float tVal = (cfg.unitTemp == 1) ? (wxData.tempC * 9.0f / 5.0f + 32.0f) : wxData.tempC;
      radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_LIGHTGREY);
      radarCanvas.drawString(cfg.presetName + "  " + String(tVal, 1) + (cfg.unitTemp==1?"F":"C") + "  H:" + String(wxData.humidity) + "%", CX, CY + 130);
    }
  }
  // ---- [5] Digital Neon Ring ----
  else if (cfg.clockStyle == 5) {
    radarCanvas.fillScreen(TFT_BLACK);
    for (int i = 0; i < 60; i++) {
      float rad = (i * 6 - 90) * DEG_TO_RAD;
      uint16_t c = (i <= sc) ? greenBright : radarCanvas.color565(20, 45, 35);
      radarCanvas.drawLine(CX + cos(rad)*200, CY + sin(rad)*200, CX + cos(rad)*222, CY + sin(rad)*222, c);
    }
    char hms[12]; sprintf(hms, "%02d:%02d:%02d", hr, mn, sc);
    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(5); radarCanvas.setTextColor(greenBright);
    radarCanvas.drawString(hms, CX, CY - 10);
    radarCanvas.setTextSize(2); radarCanvas.setTextColor(colRiver);
    radarCanvas.drawString(cfg.tzLabel, CX, CY + 56);
  }
  // ---- [6] Digital Retro Flip ----
  else {
    radarCanvas.fillScreen(radarCanvas.color565(18, 18, 20));
    radarCanvas.fillRoundRect(42, 130, 184, 190, 16, radarCanvas.color565(34, 36, 42));
    radarCanvas.fillRoundRect(254, 130, 184, 190, 16, radarCanvas.color565(34, 36, 42));
    radarCanvas.drawRoundRect(42, 130, 184, 190, 16, radarCanvas.color565(90, 95, 105));
    radarCanvas.drawRoundRect(254, 130, 184, 190, 16, radarCanvas.color565(90, 95, 105));

    char hStr[4], mStr[4], sStr[16];
    sprintf(hStr, "%02d", hr); sprintf(mStr, "%02d", mn); sprintf(sStr, "SEC %02d  ·  %s", sc, cfg.tzLabel.c_str());

    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(8); radarCanvas.setTextColor(TFT_WHITE);
    radarCanvas.drawString(hStr, 134, 225);
    radarCanvas.drawString(mStr, 346, 225);
    // 플립 중앙 분할선
    radarCanvas.drawFastHLine(42, 225, 184, TFT_BLACK);
    radarCanvas.drawFastHLine(254, 225, 184, TFT_BLACK);

    radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_YELLOW);
    radarCanvas.drawString(sStr, CX, 370);
  }
}

// ==========================================
// 10. 레이더 & 날씨 화면 렌더링
// ==========================================
void drawMapBackground() {
  if (mapLoaded) mapBgSprite.pushSprite(&radarCanvas, 0, 0);
  else radarCanvas.fillRect(2, 2, SCREEN_W - 4, SCREEN_H - 4, colSeaBg);

  radarCanvas.setTextDatum(top_left); radarCanvas.setTextSize(1); radarCanvas.setTextColor(colAirport);
  for (int i = 0; i < AIRPORTS_LEN; i++) {
    int ax, ay;
    geoToScreen(AIRPORT_COORDS[i][0], AIRPORT_COORDS[i][1], ax, ay);
    if (hypot(ax - CX, ay - CY) <= R - 10) {
      radarCanvas.drawRect(ax - 4, ay - 4, 9, 9, colAirport);
      radarCanvas.drawString(AIRPORT_CODES[i], ax + 7, ay - 4);
    }
  }
}

void drawWeatherIcon(int cx, int cy, int wmoCode, float s) {
  if (wmoCode == 0) {
    radarCanvas.fillCircle(cx, cy, (int)(11 * s), TFT_YELLOW);
    for (int a = 0; a < 360; a += 45) {
      float r = a * DEG_TO_RAD;
      radarCanvas.drawLine(cx + cos(r)*14*s, cy + sin(r)*14*s, cx + cos(r)*18*s, cy + sin(r)*18*s, TFT_YELLOW);
    }
  } else {
    uint16_t cCol = (wmoCode >= 51) ? radarCanvas.color565(200, 220, 235) : TFT_WHITE;
    radarCanvas.fillCircle(cx - (int)(8*s), cy + (int)(2*s), (int)(8*s), cCol);
    radarCanvas.fillCircle(cx + (int)(8*s), cy + (int)(2*s), (int)(7*s), cCol);
    radarCanvas.fillCircle(cx, cy - (int)(4*s), (int)(10*s), cCol);
    radarCanvas.fillRect(cx - (int)(8*s), cy + (int)(2*s), (int)(16*s), (int)(8*s), cCol);
    if (wmoCode >= 51) {
      uint16_t rainCol = radarCanvas.color565(90, 210, 255);
      radarCanvas.fillCircle(cx - (int)(7*s), cy + (int)(14*s), (int)(2*s), rainCol);
      radarCanvas.fillCircle(cx, cy + (int)(17*s), (int)(2*s), rainCol);
      radarCanvas.fillCircle(cx + (int)(7*s), cy + (int)(14*s), (int)(2*s), rainCol);
    }
  }
}

void drawFullWeatherView() {
  drawMapBackground();
  radarCanvas.fillCircle(CX, CY, 4, TFT_WHITE);

  radarCanvas.setTextDatum(middle_center);
  radarCanvas.setTextSize(2);
  radarCanvas.setTextColor(TFT_WHITE, colWxPanel);
  struct tm ti;
  char tStr[10] = "--:--";
  if (getLocalTime(&ti)) sprintf(tStr, "%02d:%02d", ti.tm_hour, ti.tm_min);
  radarCanvas.drawString(" WEATHER   " + String(tStr) + " ", CX, 32);

  radarCanvas.fillRoundRect(92, 54, 96, 26, 13, radarCanvas.color565(18, 72, 42));
  radarCanvas.drawRoundRect(92, 54, 96, 26, 13, greenBright);
  radarCanvas.setTextColor(greenBright); radarCanvas.drawString("Live", 140, 67);

  radarCanvas.fillRoundRect(292, 54, 96, 26, 13, radarCanvas.color565(20, 58, 108));
  radarCanvas.drawRoundRect(292, 54, 96, 26, 13, colRiver);
  radarCanvas.setTextColor(colRiver); radarCanvas.drawString("Latest", 340, 67);

  if (!wxData.valid) {
    radarCanvas.fillRoundRect(90, 200, 300, 60, 12, colWxPanel);
    radarCanvas.drawRoundRect(90, 200, 300, 60, 12, colRiver);
    radarCanvas.setTextColor(TFT_WHITE); radarCanvas.drawString("Loading Weather...", CX, 230);
    return;
  }

  float tVal = (cfg.unitTemp == 1) ? (wxData.tempC * 9.0f / 5.0f + 32.0f) : wxData.tempC;
  float fVal = (cfg.unitTemp == 1) ? (wxData.feelsLikeC * 9.0f / 5.0f + 32.0f) : wxData.feelsLikeC;
  String tUnit = (cfg.unitTemp == 1) ? "F" : "C";
  float pVal = (cfg.unitPrecip == 1) ? (wxData.precipMm / 25.4f) : wxData.precipMm;

  radarCanvas.fillRoundRect(46, 92, 186, 108, 12, colWxPanel);
  radarCanvas.drawRoundRect(46, 92, 186, 108, 12, colWxBorder);
  drawWeatherIcon(82, 126, wxData.wmoCode, 1.15f);

  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(3); radarCanvas.setTextColor(TFT_WHITE);
  radarCanvas.drawString(String((int)roundf(tVal)) + tUnit, 120, 104);
  radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_LIGHTGREY);
  radarCanvas.drawString("~ " + String((int)roundf(fVal)) + tUnit, 120, 140);
  radarCanvas.setTextColor(colRiver);
  radarCanvas.drawString(String(pVal, 1) + (cfg.unitPrecip == 1 ? " in" : " mm"), 66, 172);

  radarCanvas.fillRoundRect(248, 92, 186, 108, 12, colWxPanel);
  radarCanvas.drawRoundRect(248, 92, 186, 108, 12, colWxBorder);

  float wSpd = (cfg.unitSpeed == 1) ? (wxData.windKmh / 1.852f) : wxData.windKmh;
  float gSpd = (cfg.unitSpeed == 1) ? (wxData.gustKmh / 1.852f) : wxData.gustKmh;
  String sUnit = (cfg.unitSpeed == 1) ? "kt" : "k";
  String visStr = (wxData.wmoCode >= 45 && wxData.wmoCode <= 48) ? "1nm" : ((wxData.wmoCode >= 51) ? "5nm" : "10nm");

  radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_WHITE);
  radarCanvas.drawString("H:" + String(wxData.humidity) + "% C:" + String(wxData.cloudCover) + "%", 258, 104);
  radarCanvas.setTextColor(greenBright);
  radarCanvas.drawString(String(degToCompass(wxData.windDir)) + " " + String((int)roundf(wSpd)) + sUnit + " G" + String((int)roundf(gSpd)), 258, 138);
  radarCanvas.setTextColor(TFT_YELLOW);
  radarCanvas.drawString(String((int)wxData.pressureHpa) + " " + visStr, 258, 172);

  radarCanvas.fillRoundRect(56, 312, 368, 98, 12, colWxPanel);
  radarCanvas.drawRoundRect(56, 312, 368, 98, 12, colWxBorder);
  radarCanvas.drawFastVLine(178, 322, 78, colWxBorder);
  radarCanvas.drawFastVLine(302, 322, 78, colWxBorder);

  const char* dayLabels[3] = {"Today", "Tmrw", "Day 3"};
  int colCenters[3] = {117, 240, 363};

  for (int i = 0; i < 3; i++) {
    int cx = colCenters[i];
    float dMax = (cfg.unitTemp == 1) ? (wxData.dayMaxC[i] * 9.0f / 5.0f + 32.0f) : wxData.dayMaxC[i];
    float dMin = (cfg.unitTemp == 1) ? (wxData.dayMinC[i] * 9.0f / 5.0f + 32.0f) : wxData.dayMinC[i];

    radarCanvas.setTextDatum(middle_center);
    radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_LIGHTGREY); radarCanvas.drawString(dayLabels[i], cx, 328);
    drawWeatherIcon(cx - 32, 358, wxData.dayCode[i], 0.72f);
    radarCanvas.setTextColor(TFT_WHITE); radarCanvas.drawString(String((int)roundf(dMax)) + "/" + String((int)roundf(dMin)), cx + 18, 358);
    radarCanvas.setTextColor(colRiver);  radarCanvas.drawString(String(wxData.dayPop[i]) + "%", cx, 392);
  }

  radarCanvas.setTextDatum(middle_right); radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_WHITE);
  radarCanvas.drawString("Light", 162, 428);
  for (int i = 0; i < 140; i++) {
    uint16_t c;
    if (i < 35)       c = radarCanvas.color565(30, 100 + i * 4, 255);
    else if (i < 70)  c = radarCanvas.color565(30, 240, 255 - (i - 35) * 6);
    else if (i < 105) c = radarCanvas.color565(30 + (i - 70) * 6, 240, 30);
    else              c = radarCanvas.color565(255, 240 - (i - 105) * 6, 30);
    radarCanvas.drawFastVLine(170 + i, 424, 9, c);
  }
  radarCanvas.setTextDatum(middle_left); radarCanvas.setTextColor(TFT_WHITE);
  radarCanvas.drawString("Heavy", 318, 428);

  radarCanvas.setTextDatum(middle_center); radarCanvas.setTextSize(2); radarCanvas.setTextColor(greenBright, colWxPanel);
  radarCanvas.drawString(" " + cfg.presetName + " ", CX, 456);
}

void drawAirplaneIcon(int cx, int cy, float headingDeg, float scale, uint16_t color) {
  float rad = headingDeg * DEG_TO_RAD, sinA = sin(rad), cosA = cos(rad);
  auto drawJet = [&](float s, int ox, int oy, uint16_t c) {
    auto rx = [&](float lx, float ly) -> int { return cx + ox + (int)roundf((lx * cosA - ly * sinA) * s); };
    auto ry = [&](float lx, float ly) -> int { return cy + oy + (int)roundf((lx * sinA + ly * cosA) * s); };
    int noseX=rx(0,-11.5f), noseY=ry(0,-11.5f), fLFx=rx(-1.8f,-5.5f), fLFy=ry(-1.8f,-5.5f), fRFx=rx(1.8f,-5.5f), fRFy=ry(1.8f,-5.5f);
    int fLBx=rx(-1.4f,7.5f), fLBy=ry(-1.4f,7.5f), fRBx=rx(1.4f,7.5f), fRBy=ry(1.4f,7.5f), tailX=rx(0,10.0f), tailY=ry(0,10.0f);
    radarCanvas.fillTriangle(noseX,noseY,fLFx,fLFy,fRFx,fRFy,c);
    radarCanvas.fillTriangle(fLFx,fLFy,fRFx,fRFy,fLBx,fLBy,c);
    radarCanvas.fillTriangle(fRFx,fRFy,fLBx,fLBy,fRBx,fRBy,c);
    radarCanvas.fillTriangle(fLBx,fLBy,fRBx,fRBy,tailX,tailY,c);
    radarCanvas.fillTriangle(rx(1.4f,-3.5f),ry(1.4f,-3.5f),rx(1.4f,1.5f),ry(1.4f,1.5f),rx(11.0f,2.8f),ry(11.0f,2.8f),c);
    radarCanvas.fillTriangle(rx(1.4f,1.5f),ry(1.4f,1.5f),rx(11.0f,2.8f),ry(11.0f,2.8f),rx(10.5f,4.5f),ry(10.5f,4.5f),c);
    radarCanvas.fillTriangle(rx(-1.4f,-3.5f),ry(-1.4f,-3.5f),rx(-1.4f,1.5f),ry(-1.4f,1.5f),rx(-11.0f,2.8f),ry(-11.0f,2.8f),c);
    radarCanvas.fillTriangle(rx(-1.4f,1.5f),ry(-1.4f,1.5f),rx(-11.0f,2.8f),ry(-11.0f,2.8f),rx(-10.5f,4.5f),ry(-10.5f,4.5f),c);
    radarCanvas.fillTriangle(rx(1.0f,6.2f),ry(1.0f,6.2f),rx(0.8f,8.8f),ry(0.8f,8.8f),rx(5.0f,9.5f),ry(5.0f,9.5f),c);
    radarCanvas.fillTriangle(rx(-1.0f,6.2f),ry(-1.0f,6.2f),rx(-0.8f,8.8f),ry(-0.8f,8.8f),rx(-5.0f,9.5f),ry(-5.0f,9.5f),c);
  };
  drawJet(scale * 1.12f, 0, 1, TFT_BLACK);
  drawJet(scale, 0, 0, color);
}

void drawWeatherOverlayCard() {
  if (!cfg.wxShowCards || !wxData.valid) return;
  float tVal = (cfg.unitTemp == 1) ? (wxData.tempC * 9.0f / 5.0f + 32.0f) : wxData.tempC;
  String tUnit = (cfg.unitTemp == 1) ? "F" : "C";

  radarCanvas.fillRect(6, SCREEN_H - 64, 152, 58, colCardBg);
  radarCanvas.drawRect(6, SCREEN_H - 64, 152, 58, colRiver);
  radarCanvas.setTextDatum(top_left);
  radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_WHITE);
  radarCanvas.drawString(String(tVal, 1) + tUnit, 12, SCREEN_H - 59);

  String row2 = "";
  if (cfg.wxHumidity) row2 += "H:" + String(wxData.humidity) + "% ";
  if (cfg.wxWind) {
    float wSpd = (cfg.unitSpeed == 1) ? (wxData.windKmh / 1.852f) : wxData.windKmh;
    row2 += String(degToCompass(wxData.windDir)) + " " + String(wSpd, 0) + (cfg.unitSpeed == 1 ? "kt" : "k/h");
  }
  radarCanvas.setTextSize(1); radarCanvas.setTextColor(colRiver);
  radarCanvas.drawString(row2, 12, SCREEN_H - 34);

  String row3 = "";
  if (cfg.wxPrecip) {
    float pVal = (cfg.unitPrecip == 1) ? (wxData.precipMm / 25.4f) : wxData.precipMm;
    row3 += "P:" + String(pVal, 1) + (cfg.unitPrecip == 1 ? "in " : "mm ");
  }
  if (cfg.wxPressure) row3 += String((int)wxData.pressureHpa) + "hPa";
  radarCanvas.setTextColor(greenBright); radarCanvas.drawString(row3, 12, SCREEN_H - 20);
}

void drawRadarFrameToCanvas() {
  if (cfg.viewMode == 1) { drawFullWeatherView(); return; }
  if (cfg.viewMode == 2) { drawClockView(); return; }

  if (!cfg.showMapBg) radarCanvas.fillRect(2, 2, SCREEN_W - 4, SCREEN_H - 4, TFT_BLACK);
  else drawMapBackground();

  if (cfg.showScanLine) {
    for (float i = TRAIL_WIDTH; i >= 0; i -= 3.0) {
      float a = sweepAngle - i; if (a < 0) a += 360;
      uint8_t g = 40 + ((TRAIL_WIDTH - i) / TRAIL_WIDTH) * 180;
      float rad = (a - 90) * DEG_TO_RAD;
      radarCanvas.drawLine(CX, CY, CX + cos(rad) * R, CY + sin(rad) * R, radarCanvas.color565(0, g, 40));
    }
    float rad = (sweepAngle - 90) * DEG_TO_RAD;
    radarCanvas.drawLine(CX, CY, CX + cos(rad) * R, CY + sin(rad) * R, greenBright);
  }

  int step = R / 5;
  for (int i = 1; i <= 4; i++) radarCanvas.drawCircle(CX, CY, step * i, greenDim);
  radarCanvas.drawCircle(CX, CY, R, greenBright);
  radarCanvas.drawFastHLine(CX - R, CY, R * 2, greenDim);
  radarCanvas.drawFastVLine(CX, CY - R, R * 2, greenDim);

  float dispRangeKm = mapLoaded ? effectiveRangeKm : cfg.radiusKm;
  radarCanvas.setTextDatum(middle_center); radarCanvas.setTextSize(1); radarCanvas.setTextColor(greenBright);
  for (int i = 1; i <= 4; i++) {
    float val = (dispRangeKm / 5.0f) * i;
    radarCanvas.drawString((cfg.unitDist == 1) ? String((int)roundf(val / 1.852f)) + "nm" : String((int)roundf(val)) + "km", CX + 22, CY - step * i - 7);
  }
  for (int a = 0; a < 360; a += 5) canvasCorrectedRadialLine(a, R - (a % 10 == 0 ? 10 : 5), R, greenBright);

  for (int i = 0; i < planeCount; i++) {
    if (!planes[i].valid) continue;
    float ageAngle = cfg.showScanLine ? angleDiffBehindSweep(sweepAngle, planes[i].bearingDeg) : 0;
    if (cfg.showScanLine && ageAngle > 320) continue;
    float fade = (cfg.showScanLine && ageAngle > 240) ? max(0.40f, 1.0f - ((ageAngle - 240.0f) / 80.0f)) : 1.0f;

    int x, y;
    geoToScreen(planes[i].lat, planes[i].lon, x, y);
    if (hypot(x - CX, y - CY) > R) continue;

    uint8_t r = 0, g = 255 * fade, b = 50 * fade;
    if (planes[i].altitudeFt >= 30000)      { r = 0; g = 230 * fade; b = 255 * fade; }
    else if (planes[i].altitudeFt >= 10000) { r = 50 * fade; g = 255 * fade; b = 50 * fade; }
    else if (planes[i].altitudeFt >= 0)     { r = 255 * fade; g = 235 * fade; b = 0; }
    else                                    { r = 255 * fade; g = 60 * fade; b = 60 * fade; }

    uint16_t col = radarCanvas.color565(r, g, b);
    float heading = (planes[i].trackDeg >= 0) ? planes[i].trackDeg : 0.0f;

    if (cfg.showVectors && planes[i].trackDeg >= 0) {
      float tr = (heading - 90) * DEG_TO_RAD;
      radarCanvas.drawLine(x, y, x + cos(tr) * 20, y + sin(tr) * 20, col);
    }
    drawAirplaneIcon(x, y, heading, 1.0f, col);

    if (!cfg.showScanLine || ageAngle < 150) {
      radarCanvas.setTextDatum(top_left); radarCanvas.setTextSize(1);
      int ly = y - 10;
      String l1 = "";
      if (cfg.showCallsign) { String fl = String(planes[i].flight); fl.trim(); l1 += (fl.length() > 0 ? fl : "UNK"); }
      if (cfg.showAcType && strlen(planes[i].typeCode) > 0) { if (l1.length() > 0) l1 += " "; l1 += "[" + String(planes[i].typeCode) + "]"; }
      if (l1.length() > 0) { radarCanvas.setTextColor(TFT_WHITE, TFT_BLACK); radarCanvas.drawString(l1, x + 13, ly); ly += 10; }

      String l2 = "";
      if (cfg.showAltitude && planes[i].altitudeFt >= 0) l2 += (cfg.unitAlt == 1) ? String(planes[i].altitudeFt) + "ft" : String((int)(planes[i].altitudeFt * 0.3048f)) + "m";
      if (cfg.showSpeed && planes[i].speedKt > 0) { if (l2.length() > 0) l2 += " "; l2 += (cfg.unitSpeed == 1) ? String((int)planes[i].speedKt) + "kt" : String((int)(planes[i].speedKt * 1.852f)) + "k/h"; }
      if (l2.length() > 0) { radarCanvas.setTextColor(TFT_YELLOW, TFT_BLACK); radarCanvas.drawString(l2, x + 13, ly); ly += 10; }

      String l3 = "";
      if (cfg.showHeading) l3 += "H:" + String((int)heading) + " ";
      if (cfg.showDistance) l3 += (cfg.unitDist == 1) ? String(planes[i].distanceKm / 1.852f, 1) + "nm" : String(planes[i].distanceKm, 0) + "km";
      if (l3.length() > 0) { radarCanvas.setTextColor(colRiver, TFT_BLACK); radarCanvas.drawString(l3, x + 13, ly); }
    }
  }

  radarCanvas.fillRect(6, 6, 112, 42, TFT_BLACK);
  radarCanvas.drawRect(6, 6, 112, 42, greenBright);
  radarCanvas.setTextDatum(top_left); radarCanvas.setTextSize(2);
  if (apiOk) {
    radarCanvas.setTextColor(green); radarCanvas.drawString("ADS-B", 10, 8);
    radarCanvas.setTextColor(greenBright); radarCanvas.drawString("AC:" + String(planeCount), 10, 28);
  } else {
    radarCanvas.setTextColor(radarCanvas.color565(255, 80, 80)); radarCanvas.drawString(apiStatusMsg, 10, 16);
  }

  radarCanvas.fillRect(SCREEN_W - 110, 6, 104, 34, TFT_BLACK);
  radarCanvas.drawRect(SCREEN_W - 110, 6, 104, 34, greenBright);
  radarCanvas.setTextDatum(top_right);
  struct tm ti; radarCanvas.setTextSize(2); radarCanvas.setTextColor(TFT_YELLOW);
  if (getLocalTime(&ti)) { char tStr[12]; sprintf(tStr, "%02d:%02d:%02d", ti.tm_hour, ti.tm_min, ti.tm_sec); radarCanvas.drawString(tStr, SCREEN_W - 10, 8); }
  else radarCanvas.drawString("--:--:--", SCREEN_W - 10, 8);
  radarCanvas.setTextSize(1); radarCanvas.setTextColor(green); radarCanvas.drawString(cfg.tzLabel, SCREEN_W - 10, 28);

  drawWeatherOverlayCard();

  radarCanvas.fillRect(SCREEN_W - 126, SCREEN_H - 42, 120, 36, TFT_BLACK);
  radarCanvas.drawRect(SCREEN_W - 126, SCREEN_H - 42, 120, 36, greenBright);
  radarCanvas.setTextDatum(bottom_right);
  radarCanvas.setTextSize(2); radarCanvas.setTextColor(greenBright); radarCanvas.drawString(cfg.presetName, SCREEN_W - 10, SCREEN_H - 22);
  radarCanvas.setTextSize(1); radarCanvas.setTextColor(TFT_YELLOW);  radarCanvas.drawString(cfg.hostname + ".local", SCREEN_W - 10, SCREEN_H - 8);

  radarCanvas.drawRect(1, 1, SCREEN_W - 2, SCREEN_H - 2, greenBright);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(400);

  loadConfig();
  st7701_init_panel();

  lcd.init();
  lcd.setBrightness(map(cfg.brightness, 10, 100, 25, 255));

  greenBright = lcd.color565(0, 255, 60);
  green       = lcd.color565(0, 200, 50);
  greenDim    = lcd.color565(20, 120, 55);
  greenDark   = lcd.color565(15, 60, 40);
  colSeaBg    = lcd.color565(12, 32, 54);
  colRiver    = lcd.color565(60, 190, 255);
  colAirport  = lcd.color565(255, 210, 0);
  colCardBg   = lcd.color565(10, 22, 26);
  colWxPanel  = lcd.color565(16, 26, 36);
  colWxBorder = lcd.color565(58, 92, 116);

  radarCanvas.setPsram(true); radarCanvas.setColorDepth(16); radarCanvas.setSwapBytes(false);
  radarCanvas.createSprite(480, 480);

  mapBgSprite.setPsram(true); mapBgSprite.setColorDepth(16); mapBgSprite.setSwapBytes(false);
  mapBgSprite.createSprite(480, 480);
  mapBgSprite.fillScreen(colSeaBg);

  if (cfg.ssid.length() == 0) runApSetupPortal();

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK); lcd.setTextSize(2); lcd.setTextDatum(middle_center);
  lcd.drawString("Connecting WiFi...", 240, 220);
  lcd.setTextColor(greenBright, TFT_BLACK); lcd.drawString(cfg.ssid, 240, 255);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(300);

  if (WiFi.status() != WL_CONNECTED) runApSetupPortal();

  MDNS.begin(cfg.hostname.c_str());
  setupWebServer();
  configTzTime(cfg.tzPosix.c_str(), "pool.ntp.org", "time.nist.gov");

  if (cfg.viewMode != 2) {
    lcd.drawString("Downloading Google Map...", 240, 295);
    fetchMapTiles();
  }

  fetchAircraftData();
  applyPendingAircraftData();
  fetchWeatherData();
  applyPendingWeatherData();
  lastApiFetchTime = millis();
  lastWxFetchTime = millis();

  drawRadarFrameToCanvas();
  radarCanvas.pushSprite(0, 0);
}

// ---------- LOOP ----------
void loop() {
  server.handleClient();

  unsigned long interval = (cfg.viewMode == 0) ? SWEEP_SPEED : 250;
  if (millis() - lastSweepUpdate > interval) {
    lastSweepUpdate = millis();

    applyPendingWeatherData();
    drawRadarFrameToCanvas();
    radarCanvas.pushSprite(0, 0);

    if (cfg.viewMode == 0) {
      sweepAngle += 2;
      if (sweepAngle >= 360) {
        sweepAngle -= 360;
        applyPendingAircraftData();

        if (millis() - lastApiFetchTime > (unsigned long)cfg.refreshSec * 1000UL) {
          lastApiFetchTime = millis();
          startAircraftFetchBackground();
        }
      }
    }

    if (millis() - lastWxFetchTime > WX_FETCH_INTERVAL) {
      lastWxFetchTime = millis();
      startWeatherFetchBackground();
    }
  }
}