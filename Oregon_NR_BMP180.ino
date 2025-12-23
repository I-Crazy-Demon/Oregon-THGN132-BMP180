/*  CrazyDemon Weather
    JSON + narodmon.ru + Oregon Scientific + BMP180
    --------------------------------------------------------------------
    Production sketch for ESP8266 (Wemos D1, NodeMCU) with:
      - Oregon Scientific THGN132N / THN132N wireless sensors
      - BMP180 (temperature, pressure, altitude)
      - 433 MHz OOK receiver on D7 (GPIO13)
      - WiFi (ESP8266) + HTTP server
      - JSON export for narodmon.ru and local /json
      - Web UI (material-like, dark/light themes)
      - NTP time sync + timezone stored in EEPROM
      - Runtime logging with ring buffer, web log and Serial

    Скетч для:
      - приёма данных с беспроводных датчиков Oregon Scientific,
      - вывода информации на встроенную веб‑страницу,
      - отправки показаний на сервис narodmon.ru (HTTP JSON),
      - логирования работы устройства с отметкой времени.

    Исходная библиотека Oregon_NR и оригинальный скетч:
    https://github.com/invandy

    Hardware:
      - ESP8266 (Wemos D1 / NodeMCU)
      - OOK 433 MHz receiver (3V, data → D7 / GPIO13)
      - BMP180 (I2C)
*/

#include "Oregon_NR.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <EEPROM.h>
#include <time.h>

Adafruit_BMP085 bmp;

// HTTP server on port 80
// Веб‑сервер на порту 80
ESP8266WebServer server(80);

#define TESTMODE 0  // 0 = normal (send to narodmon), 1 = test mode (no send)
// 0 = рабочий режим (отправка на narodmon), 1 = тест (без отправки)

// Number of supported Oregon THGN132/THN132 channels (1..3)
// Количество поддерживаемых каналов THGN132/THN132 (1..3)
#define NOF_132 3

// Send interval to narodmon (ms)
// Интервал отправки данных на narodmon (мс)
#define SENDINTERVAL 300000UL    // 5 minutes / 5 минут
#define CONNECTTIMEOUT 10000     // WiFi connect timeout (ms) / тайм‑аут подключения к WiFi (мс)
#define DISCONNECTTIMEOUT 10000  // HTTP timeout (ms) / тайм‑аут HTTP‑соединения (мс)

// Device MAC (narodmon ID)
// MAC‑адрес устройства на narodmon.ru (используется как ID), можно комбинировать с именем для удобства
#define mac "DeviceName001122334455"

// WiFi credentials / Параметры WiFi - обязательно сменить на свои
#define ssid "wifi"
#define password "12345678"

// Status LEDs (GPIO numbers; 255 = not used)
// Светодиоды статуса (GPIO; 255 = не используется)
#define BLUE_LED 2    // WiFi activity / активность WiFi
#define GREEN_LED 14  // send OK to narodmon / успешная отправка на narodmon
#define RED_LED 255   // send FAIL to narodmon / ошибка отправки

// Logging configuration
// Дефолтные Настройки логирования. После можно изменить в настройках на вебстранице
#define DEBUG_LEVEL_DEFAULT 1  // default debug level (0..3) / уровень по умолчанию (0..3)
#define FORCE_SERIAL_DEBUG 0   // 1 = always log to Serial, ignore UI / 1 = всегда логировать в Serial

// HTTP Basic Auth for /config and /log
// Авторизация на страницах /config и /log (замените на свои)
#define CONFIG_USER "admin"
#define CONFIG_PASS "admin123"

// EEPROM layout for persistent config
// Layout EEPROM для сохранения настроек
#define EEPROM_SIZE 64
#define EEPROM_MAGIC 0x42
#define EEPROM_VERSION 3  // версия структуры LogConfig

// Persistent configuration structure stored in EEPROM
// Структура конфигурации, хранимая в EEPROM
struct LogConfig {
  uint8_t magic;           // magic marker / сигнатура
  uint8_t version;         // struct version / версия структуры
  uint8_t debugLevel;      // 0..3 debug level / уровень логирования
  uint8_t logToSerial;     // 0 = web log, 1 = Serial / выбор вывода
  int8_t tzOffsetHours;    // timezone offset in hours (−12..+14) / часовой пояс
  uint8_t refreshMainSec;  // auto refresh main page (seconds) / автообновление главной страницы
  uint8_t refreshLogSec;   // auto refresh log page (seconds) / автообновление страницы логов
};

// Exponential smoothing for sensor values (affects only values, not reception quality)
// Параметры сглаживания (влияют только на плавность показаний, не на приём)
const float SMOOTH_ALPHA = 0.3f;  // 30% new, 70% old / 30% новое, 70% предыдущее значение

// NTP server / NTP сервер
const char *ntpServer = "pool.ntp.org";

// narodmon send interval
// Интервал отправки данных на narodmon
const unsigned long postingInterval = SENDINTERVAL;
unsigned long lastConnectionTime = 0;

// Oregon receiver on D7 (GPIO13), LED on BLUE_LED
// Приёмник 433 МГц на D7 (GPIO13), светодиод BLUE_LED
Oregon_NR oregon(13, 13, BLUE_LED, true);

// THGN132/THN132 sensor state
// Структура для хранения данных по каналам THGN132/THN132
struct BTHGNsensor {
  bool isreceived = false;     // at least one valid packet seen / был ли хоть один валидный пакет
  byte numberofreceiving = 0;  // number of received frames (for averaging) / число принятых кадров
  unsigned long rcvtime = 0;   // millis() of last receive / время последнего приёма
  byte chnl = 0;               // sensor channel (1..3) / канал датчика (1..3)
  word type = 0;               // sensor type / тип датчика
  float temperature = 0;       // temperature (°C) / температура (°C)
  float humidity = 0;          // humidity (%) / влажность (%)
  bool battery = true;         // battery ok flag / состояние батареи
};

BTHGNsensor tsensor[NOF_132];

// TCP client for narodmon HTTP
// Клиент TCP для отправки данных на narodmon
WiFiClient narodClient;

// Runtime logging settings (can be changed via /config)
// Настройки логирования (меняются через /config)
uint8_t debugLevel = DEBUG_LEVEL_DEFAULT;  // current debug level / текущий уровень логирования
bool logToSerial = true;                   // true = Serial, false = web log / куда писать лог
int8_t tzOffsetHours = 9;                  // default timezone GMT+9 / часовой пояс по умолчанию
uint8_t refreshMainSec = 30;               // main page auto refresh / автообновление главной страницы
uint8_t refreshLogSec = 5;                 // log page auto refresh / автообновление страницы логов

// -------- Ring buffer for logs / Кольцевой буфер логов --------
const size_t LOG_BUFFER_MAX = 4000;
char logBuf[LOG_BUFFER_MAX];
size_t logHead = 0;
bool logWrapped = false;

// Append message to ring buffer (web log)
// Добавить строку в кольцевой буфер (для веб‑лога)
void logToRingBuffer(const String &msg) {
  for (size_t i = 0; i < msg.length(); i++) {
    logBuf[logHead] = msg[i];
    logHead = (logHead + 1) % LOG_BUFFER_MAX;
    if (logHead == 0) logWrapped = true;
  }
  logBuf[logHead] = '\n';
  logHead = (logHead + 1) % LOG_BUFFER_MAX;
  if (logHead == 0) logWrapped = true;
}

// Dump ring buffer as String
// Получить весь лог из кольцевого буфера
String getLogBufferString() {
  String out;
  if (!logWrapped) {
    for (size_t i = 0; i < logHead; i++) out += logBuf[i];
  } else {
    size_t i = logHead;
    do {
      out += logBuf[i];
      i = (i + 1) % LOG_BUFFER_MAX;
    } while (i != logHead);
  }
  return out;
}

// Convert RSSI (dBm) to 0..100%
// Перевод RSSI (dBm) в проценты 0..100
int wifiRssiToPercent(int rssi) {
  if (rssi <= -90) return 0;
  if (rssi >= -30) return 100;
  return (rssi + 90) * 100 / 60;
}

// Format current local time with seconds and trailing space for log prefix
// Форматировать текущее локальное время (для префикса в логе)
String getTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) return String("[no time] ");
  struct tm *tm_info = localtime(&now);
  if (!tm_info) return String("[no time] ");
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S ", tm_info);
  return String(buf);
}

// Format arbitrary UTC time to local string (for JSON/HTML)
// Форматировать произвольное время UTC в локальную строку (JSON/HTML)
String getTimeStringShort(time_t t) {
  if (t < 100000) return String("");
  struct tm *tm_info = localtime(&t);
  if (!tm_info) return String("");
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
  return String(buf);
}

// Log macro with time prefix and level filtering
// Макрос логирования с префиксом времени и фильтром по уровню
#define LOG(level, msg) \
  do { \
    if ((level) <= debugLevel) { \
      String __m = getTimeString() + msg; \
      if (FORCE_SERIAL_DEBUG || logToSerial) { \
        Serial.println(__m); \
      } else { \
        logToRingBuffer(__m); \
      } \
    } \
  } while (0)

// Prototypes
// Прототипы функций
void wifi_connect();
void syncTime();
bool senddata();
String buildNarodmonJson();

void handleRoot();
void handleReboot();
void handleJson();
void handleConfig();
void handleLog();

void saveLogConfig();
void loadLogConfig();

// ---------- HTTP Auth / Авторизация ----------
bool isAuthorized() {
  if (!server.authenticate(CONFIG_USER, CONFIG_PASS)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// Common app header with menu
// Общий заголовок с навигацией
void addMenuLinks(String &web) {
  web += "<header class=\"app-bar\">";
  web += "<div class=\"app-title\">Home Weather</div>";
  web += "<nav class=\"app-nav\">";
  web += "<a href=\"/\" class=\"nav-link\">Main</a>";
  web += "<a href=\"/json\" class=\"nav-link\">JSON</a>";
  web += "<a href=\"/config\" class=\"nav-link\">Config</a>";
  web += "<a href=\"/log\" class=\"nav-link\">Log</a>";
  web += "</nav>";
  web += "<button id=\"themeToggle\" class=\"icon-button\" aria-label=\"Toggle theme\">🌓</button>";
  web += "</header>";
}

// ---------- Main page (material-like UI) / Главная страница ----------
void handleRoot() {
  String web;

  web += "<!DOCTYPE html><html><head><meta charset=\"utf-8\">";
  web += "<title>Home Weather Station</title>";
  web += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  if (refreshMainSec > 0) {
    // Auto refresh (seconds), configurable in /config
    // Автообновление страницы (секунды), настраивается в /config
    web += "<meta http-equiv=\"refresh\" content=\"";
    web += String(refreshMainSec);
    web += "\">";
  }

  // CSS: dark/light theme + cards layout
  // CSS: тёмная/светлая тема + карточный layout
  web += "<style>";
  web += ":root{--bg-color:#121212;--card-bg:#1E1E1E;--text-color:#E0E0E0;";
  web += "--accent:#03DAC6;--accent-2:#BB86FC;--error:#CF6679;";
  web += "--muted:#888;--border-radius:12px;--shadow:0 4px 12px rgba(0,0,0,0.4);}";
  web += ":root.light{--bg-color:#F5F5F5;--card-bg:#FFFFFF;--text-color:#212121;";
  web += "--accent:#6200EE;--accent-2:#03DAC6;--error:#B00020;";
  web += "--muted:#777;--shadow:0 2px 8px rgba(0,0,0,0.15);}";
  web += "body{margin:0;font-family:-apple-system,BlinkMacSystemFont,'Roboto','Segoe UI',sans-serif;";
  web += "background:var(--bg-color);color:var(--text-color);}";

  web += ".app-bar{position:sticky;top:0;z-index:10;display:flex;align-items:center;";
  web += "justify-content:space-between;padding:8px 16px;background:rgba(18,18,18,0.9);";
  web += "backdrop-filter:blur(8px);border-bottom:1px solid rgba(255,255,255,0.06);}";
  web += ".light .app-bar{background:rgba(245,245,245,0.9);}";

  web += ".app-title{font-weight:600;font-size:18px;}";
  web += ".app-nav{display:flex;gap:8px;}";
  web += ".nav-link{color:var(--text-color);text-decoration:none;padding:6px 10px;";
  web += "border-radius:999px;font-size:14px;}";
  web += ".nav-link:hover{background:rgba(255,255,255,0.06);}";

  web += ".icon-button{border:none;background:transparent;color:var(--text-color);";
  web += "cursor:pointer;font-size:18px;padding:4px;border-radius:50%;}";
  web += ".icon-button:hover{background:rgba(255,255,255,0.1);}";

  web += ".container{max-width:1000px;margin:12px auto 24px;padding:0 12px;}";
  web += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;}";
  web += ".card{background:var(--card-bg);border-radius:var(--border-radius);padding:14px;";
  web += "box-shadow:var(--shadow);display:flex;flex-direction:column;gap:4px;}";
  web += ".card-header{display:flex;justify-content:space-between;align-items:center;}";
  web += ".card-title{font-size:15px;font-weight:600;}";
  web += ".chip{font-size:11px;border-radius:999px;padding:2px 8px;border:1px solid var(--muted);color:var(--muted);}";
  web += ".chip-ok{border-color:var(--accent);color:var(--accent);}";
  web += ".chip-stale{border-color:var(--error);color:var(--error);}";
  web += ".value{font-size:26px;font-weight:500;}";
  web += ".value span{font-size:13px;margin-left:4px;color:var(--muted);}";
  web += ".meta{font-size:11px;color:var(--muted);}";
  web += ".section-title{font-size:14px;font-weight:600;margin:16px 4px 8px;opacity:0.9;}";
  web += ".row{display:flex;justify-content:space-between;font-size:13px;}";
  web += ".row span:last-child{font-weight:500;}";
  web += "@media (max-width:600px){.app-title{font-size:16px;} .card{padding:12px;}}";
  web += "</style>";

  // Theme toggle (dark/light) via localStorage
  // Переключение темы (тёмная/светлая) через localStorage
  web += "<script>";
  web += "document.addEventListener('DOMContentLoaded',function(){";
  web += " const root=document.documentElement;";
  web += " const saved=localStorage.getItem('theme');";
  web += " if(saved==='light') root.classList.add('light');";
  web += " const btn=document.getElementById('themeToggle');";
  web += " if(btn){btn.addEventListener('click',()=>{";
  web += "  root.classList.toggle('light');";
  web += "  localStorage.setItem('theme',root.classList.contains('light')?'light':'dark');";
  web += " });}";
  web += "});";
  web += "</script>";

  web += "</head><body>";

  addMenuLinks(web);

  web += "<main class=\"container\">";

  unsigned long nowMs = millis();
  time_t nowUtc = time(nullptr);

  // --- Oregon sensors section first / Секция Oregon сенсоров первой ---
  web += "<div class=\"section-title\">Oregon sensors</div>";
  web += "<div class=\"grid\">";

  for (byte i = 0; i < NOF_132; i++) {
    if (!tsensor[i].isreceived) continue;

    // In this version we do NOT clear isreceived on timeout.
    // Здесь мы НЕ сбрасываем isreceived по тайм‑ауту — всегда показываем последнее значение.
    bool isStale = false;
    if (tsensor[i].rcvtime > 0) {
      unsigned long ageMs = nowMs - tsensor[i].rcvtime;
      // считаем stale, если больше 30 минут, для отображения
      isStale = (ageMs > 30UL * 60UL * 1000UL);
    }

    // Reconstruct approximate last receive time in UTC using millis() age
    time_t lastUtc = 0;
    if (nowUtc > 100000 && tsensor[i].rcvtime > 0) {
      unsigned long ageMs = nowMs - tsensor[i].rcvtime;
      lastUtc = nowUtc - (ageMs / 1000UL);
    }

    // Human‑readable channel names, as in original sketch:
    // 1 → Bedroom, 2 → Outdoor, 3 → Channel3
    String chName = "Channel " + String(tsensor[i].chnl);
    if (tsensor[i].chnl == 1) chName = "Bedroom";
    if (tsensor[i].chnl == 2) chName = "Outdoor";
    if (tsensor[i].chnl == 3) chName = "Channel3";

    web += "<article class=\"card\">";
    web += "<div class=\"card-header\">";
    web += "<div class=\"card-title\">";
    web += chName;
    web += "</div>";
    if (isStale) web += "<span class=\"chip chip-stale\">Stale</span>";
    else web += "<span class=\"chip chip-ok\">Live</span>";
    web += "</div>";

    web += "<div class=\"value\">";
    web += String(tsensor[i].temperature, 2);
    web += "<span>°C</span></div>";

    if (tsensor[i].humidity > 0 && tsensor[i].humidity <= 100) {
      web += "<div class=\"row\"><span>Humidity</span><span>";
      web += String(tsensor[i].humidity, 1);
      web += " %</span></div>";
    }

    web += "<div class=\"row\"><span>Battery</span><span>";
    web += (tsensor[i].battery ? "LOW" : "OK");
    web += "</span></div>";

    web += "<div class=\"meta\">Last update: ";
    if (lastUtc > 0) web += getTimeStringShort(lastUtc);
    else web += "unknown";
    web += "</div>";

    web += "</article>";
  }

  web += "</div>";  // grid Oregon

  // --- Station (BMP180 + WiFi) ---
  web += "<div class=\"section-title\">Station</div>";
  web += "<div class=\"grid\">";

  float press_mm = bmp.readPressure() / 133.3 - 0.1;
  web += "<article class=\"card\">";
  web += "<div class=\"card-header\">";
  web += "<div class=\"card-title\">Pressure (BMP180)</div>";
  web += "<span class=\"chip chip-ok\">BMP180</span>";
  web += "</div>";
  web += "<div class=\"value\">";
  web += String(press_mm, 2);
  web += "<span>mmHg</span></div>";
  web += "</article>";

  web += "<article class=\"card\">";
  web += "<div class=\"card-header\">";
  web += "<div class=\"card-title\">Station temp</div>";
  web += "<span class=\"chip chip-ok\">BMP180</span>";
  web += "</div>";
  web += "<div class=\"value\">";
  web += String(bmp.readTemperature(), 2);
  web += "<span>°C</span></div>";
  web += "</article>";

  web += "<article class=\"card\">";
  web += "<div class=\"card-header\">";
  web += "<div class=\"card-title\">Altitude</div>";
  web += "<span class=\"chip chip-ok\">BMP180</span>";
  web += "</div>";
  web += "<div class=\"value\">";
  web += String(bmp.readAltitude(), 1);
  web += "<span>m</span></div>";
  web += "</article>";

  int wifiPercent = wifiRssiToPercent(WiFi.RSSI());
  web += "<article class=\"card\">";
  web += "<div class=\"card-header\">";
  web += "<div class=\"card-title\">Wi‑Fi</div>";
  web += "<span class=\"chip\">";
  web += WiFi.SSID();
  web += "</span>";
  web += "</div>";
  web += "<div class=\"value\">";
  web += String(wifiPercent);
  web += "<span>%</span></div>";
  web += "<div class=\"meta\">IP: ";
  web += WiFi.localIP().toString();
  web += "</div>";
  web += "</article>";

  web += "</div>";  // grid Station

  // --- System info (uptime + local time) ---
  uint32_t sec = millis() / 1000ul;
  int timeHours = sec / 3600ul;
  int timeMins = (sec % 3600ul) / 60ul;
  int timeSecs = sec % 60ul;

  web += "<div class=\"section-title\">System</div>";
  web += "<div class=\"grid\">";
  web += "<article class=\"card\">";
  web += "<div class=\"card-header\"><div class=\"card-title\">Uptime</div></div>";
  web += "<div class=\"value\">";
  if (timeHours < 10) web += "0";
  web += String(timeHours);
  web += ":";
  if (timeMins < 10) web += "0";
  web += String(timeMins);
  web += ":";
  if (timeSecs < 10) web += "0";
  web += String(timeSecs);
  web += "</div>";
  web += "<div class=\"meta\">Local time: ";
  web += getTimeStringShort(time(nullptr));
  web += "</div>";
  web += "</article>";
  web += "</div>";

  web += "</main></body></html>";

  server.send(200, "text/html", web);
}

// ---------- Local JSON API (/json) ----------
void handleJson() {
  float bmpTemp = bmp.readTemperature();
  float bmpPress = bmp.readPressure();
  float bmpAlt = bmp.readAltitude();
  int wifiRssi = WiFi.RSSI();

  time_t nowUtc = time(nullptr);
  unsigned long nowMs = millis();

  String json = "{";
  json += "\"bmp180\":{";
  json += "\"temperature\":" + String(bmpTemp, 2) + ",";
  json += "\"pressure\":" + String(bmpPress, 2) + ",";
  json += "\"altitude\":" + String(bmpAlt, 1);
  json += "},";

  json += "\"wifi\":{";
  json += "\"ssid\":\"" + WiFi.SSID() + "\",";
  json += "\"rssi\":" + String(wifiRssi) + ",";
  json += "\"percent\":" + String(wifiRssiToPercent(wifiRssi));
  json += "},";

  json += "\"oregon\":[";
  bool first = true;
  for (byte i = 0; i < NOF_132; i++) {
    if (!tsensor[i].isreceived) continue;

    bool isStale = false;
    if (tsensor[i].rcvtime > 0) {
      unsigned long ageMs = nowMs - tsensor[i].rcvtime;
      isStale = (ageMs > 30UL * 60UL * 1000UL);
    }

    time_t lastUtc = 0;
    if (nowUtc > 100000 && tsensor[i].rcvtime > 0) {
      unsigned long ageMs = nowMs - tsensor[i].rcvtime;
      lastUtc = nowUtc - (ageMs / 1000UL);
    }

    if (!first) json += ",";
    first = false;

    json += "{";
    json += "\"channel\":" + String(tsensor[i].chnl) + ",";
    json += "\"temperature\":" + String(tsensor[i].temperature, 2) + ",";
    json += "\"humidity\":" + String(tsensor[i].humidity, 2) + ",";
    json += "\"battery\":" + String(tsensor[i].battery ? 1 : 0) + ",";
    json += "\"stale\":" + String(isStale ? "true" : "false") + ",";
    json += "\"last_update\":\"";
    if (lastUtc > 0) json += getTimeStringShort(lastUtc);
    json += "\"";
    json += "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// POST /reboot from /config
// Принудительная перезагрузка по POST /config (кнопка Reboot)
void handleReboot() {
  LOG(0, "Reset via /reboot (POST)");
  server.send(200, "text/html", "Rebooting");
  ESP.restart();
}

// ---------- Config page (/config) ----------
void handleConfig() {
  if (!isAuthorized()) return;

  bool updated = false;
  bool doReboot = false;

  if (server.method() == HTTP_POST) {
    if (server.hasArg("reboot")) {
      doReboot = true;
    }

    if (server.hasArg("level")) {
      int lvl = server.arg("level").toInt();
      if (lvl < 0) lvl = 0;
      if (lvl > 3) lvl = 3;
      if (debugLevel != (uint8_t)lvl) {
        debugLevel = (uint8_t)lvl;
        updated = true;
      }
    }
    if (server.hasArg("target")) {
      String t = server.arg("target");
      bool newLogToSerial = logToSerial;
      if (!FORCE_SERIAL_DEBUG) {
        newLogToSerial = (t == "serial");
      } else {
        newLogToSerial = true;
      }
      if (newLogToSerial != logToSerial) {
        logToSerial = newLogToSerial;
        updated = true;
      }
    }
    if (server.hasArg("tz")) {
      int tz = server.arg("tz").toInt();
      if (tz < -12) tz = -12;
      if (tz > 14) tz = 14;
      if (tzOffsetHours != (int8_t)tz) {
        tzOffsetHours = (int8_t)tz;
        updated = true;
        long gmtOffset_sec = (long)tzOffsetHours * 3600L;
        long daylightOffset_sec = 0;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      }
    }
    if (server.hasArg("rmain")) {
      int rm = server.arg("rmain").toInt();
      if (rm < 0) rm = 0;
      if (rm > 120) rm = 120;
      if (refreshMainSec != (uint8_t)rm) {
        refreshMainSec = (uint8_t)rm;
        updated = true;
      }
    }
    if (server.hasArg("rlog")) {
      int rl = server.arg("rlog").toInt();
      if (rl < 0) rl = 0;
      if (rl > 120) rl = 120;
      if (refreshLogSec != (uint8_t)rl) {
        refreshLogSec = (uint8_t)rl;
        updated = true;
      }
    }

    if (updated) {
      LOG(1, "Config updated: level=" + String(debugLevel) + " target=" + String(logToSerial ? "Serial" : "Web") + " tz=" + String(tzOffsetHours) + " rMain=" + String(refreshMainSec) + " rLog=" + String(refreshLogSec));
      saveLogConfig();
    }

    if (doReboot) {
      handleReboot();
      return;
    }
  }

  // Simple dark config page (no theme switch here)
  // Простая тёмная страница для настроек
  String web;
  web += "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Config</title>";
  web += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  web += "<style>body{font-family:Arial;background:#111;color:#eee;margin:0;padding:0;}"
         "a{color:#4FC3F7;text-decoration:none;}header{padding:10px 16px;background:#222;margin-bottom:10px;}"
         "label{display:block;margin-top:10px;}input,select{padding:4px 6px;margin-top:4px;}"
         "form{padding:0 16px 16px;}button,input[type=submit]{margin-top:10px;padding:6px 12px;border-radius:4px;border:none;background:#4FC3F7;color:#000;}"
         "h3{margin:16px 16px 4px;}ul{margin:4px 32px 16px;}</style>";
  web += "</head><body>";

  web += "<header>";
  web += "<b>Home Weather - Config</b> &nbsp; ";
  web += "<a href=\"/\">Main</a> | <a href=\"/json\">JSON</a> | <a href=\"/log\">Log</a>";
  web += "</header>";

  web += "<form method='POST' action='/config'>";

  // Debug level / уровень логирования
  web += "<label>Debug level (0-3): ";
  web += "<input type='number' name='level' min='0' max='3' value='";
  web += String(debugLevel);
  web += "'></label>";

  // Log target / куда выводить лог
  web += "<label>Log target:</label>";
  web += "<label><input type='radio' name='target' value='serial' ";
  if (logToSerial || FORCE_SERIAL_DEBUG) web += "checked";
  web += "> Serial</label>";
  web += "<label><input type='radio' name='target' value='web' ";
  if (!logToSerial && !FORCE_SERIAL_DEBUG) web += "checked";
  web += "> Web page</label>";

  if (FORCE_SERIAL_DEBUG) {
    web += "<p style='color:#FF5252;'>FORCE_SERIAL_DEBUG=1: Serial logging is forced, log target setting is ignored.</p>";
  }

  // Timezone / Часовой пояс
  web += "<label>Time zone (GMT offset, hours): ";
  web += "<input type='number' name='tz' min='-12' max='14' value='";
  web += String(tzOffsetHours);
  web += "'></label>";

  // Auto refresh / Автообновление
  web += "<label>Main page auto refresh (sec, 0 = off): ";
  web += "<input type='number' name='rmain' min='0' max='120' value='";
  web += String(refreshMainSec);
  web += "'></label>";

  web += "<label>Log page auto refresh (sec, 0 = off): ";
  web += "<input type='number' name='rlog' min='0' max='120' value='";
  web += String(refreshLogSec);
  web += "'></label>";

  web += "<p>Current time (device): ";
  web += getTimeStringShort(time(nullptr));
  web += "</p>";

  // Logging levels description (EN + RU)
  web += "<h3>Logging levels</h3>";
  web += "<ul>";
  web += "<li>0 - boot, WiFi, critical errors (запуск, Wi‑Fi, критические ошибки)</li>";
  web += "<li>1 - + narodmon send status (статус отправки на narodmon)</li>";
  web += "<li>2 - + Oregon sensor data (данные с датчиков Oregon)</li>";
  web += "<li>3 - + detailed debug, raw packets (подробная отладка, сырые пакеты)</li>";
  web += "</ul>";

  web += "<button type='submit'>Save</button>";

  web += "<hr><h3>Device</h3>";
  web += "<button type='submit' name='reboot' value='1' onclick=\"return confirm('Reboot device?');\">Reboot</button>";

  web += "</form>";

  web += "</body></html>";
  server.send(200, "text/html", web);
}

// ---------- Log page (/log) ----------
void handleLog() {
  if (!isAuthorized()) return;

  String web;
  web += "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Log</title>";
  web += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  if (refreshLogSec > 0) {
    // Auto refresh for log page / Автообновление страницы логов
    web += "<meta http-equiv=\"refresh\" content=\"";
    web += String(refreshLogSec);
    web += "\">";
  }
  web += "<style>body{font-family:monospace;background:#000;color:#0f0;margin:0;}"
         "header{padding:10px 16px;background:#111;color:#fff;}"
         "a{color:#4FC3F7;text-decoration:none;}"
         "pre{white-space:pre-wrap;margin:0;padding:8px 12px;}</style>";
  web += "</head><body>";

  web += "<header>Home Weather - Log &nbsp;<a href=\"/\">Main</a> | <a href=\"/config\" style=\"color:#4FC3F7;\">Config</a></header>";
  web += "<pre>";
  web += getLogBufferString();
  web += "</pre>";
  web += "</body></html>";

  server.send(200, "text/html", web);
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Start Server");

  loadLogConfig();

  if (FORCE_SERIAL_DEBUG) logToSerial = true;

  LOG(0, "Booting with level=" + String(debugLevel) + " target=" + String(logToSerial ? "Serial" : "Web") + " tz=" + String(tzOffsetHours) + " rMain=" + String(refreshMainSec) + " rLog=" + String(refreshLogSec));

  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  if (RED_LED != 255) pinMode(RED_LED, OUTPUT);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    LOG(0, "BMP180 init failed! Halting.");
    while (1) { delay(1000); }
  }
  LOG(0, "BMP180 init OK");

  wifi_connect();
  syncTime();

  oregon.start();
  oregon.receiver_dump = 0;
  LOG(0, "Oregon receiver started");

  server.on("/", handleRoot);
  server.on("/reboot", handleReboot);  // legacy endpoint / устаревший, но оставлен
  server.on("/json", handleJson);
  server.on("/config", handleConfig);
  server.on("/log", handleLog);
  server.begin();
  LOG(0, "HTTP server started");
}

void loop() {
  // ESP8266 timer wraparound guards (как в AI‑версии)
  if (micros() == 0xFFF00000)
    while (micros() == 0xFFF00000)
      ;

  if (millis() == 0xFFFFFC0F)
    while (millis() == 0xFFFFFC0F)
      ;

  unsigned long now = millis();

  // Old logic: decide sending only by numberofreceiving>0
  // Старая логика: отправка только если numberofreceiving>0 по хоть одному датчику
  bool isadatatosend = false;
  for (int i = 0; i < NOF_132; i++) {
    if (tsensor[i].numberofreceiving > 0) {
      isadatatosend = true;
      break;
    }
  }

  // Periodic send to narodmon.ru
  // Периодическая отправка данных на narodmon.ru
  if ((now - lastConnectionTime > postingInterval) && isadatatosend) {
    oregon.stop();
    digitalWrite(BLUE_LED, HIGH);
    LOG(1, "Sending data to narodmon...");
    if (senddata()) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      LOG(1, "Send OK");
    } else {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      LOG(1, "Send FAILED");
    }
    oregon.start();
    oregon.receiver_dump = 0;
  }

  // Capture Oregon frames
  // Приём пакетов от датчиков Oregon
  oregon.capture(false);
  if (oregon.captured) {
    yield();

    if (debugLevel >= 3) {
      String s = String((float)now / 1000, 1) + " s ";
      if (oregon.ver == 2) s += "2 ";
      if (oregon.ver == 3) s += "3 ";
      s += "pkt=";
      for (int q = 0; q < oregon.packet_length; q++) {
        if (oregon.valid_p[q] == 0x0F) s += String(oregon.packet[q], HEX);
        else s += ".";
      }
      s += " time=" + String(oregon.work_time) + " ms";
      LOG(3, s);
    }

    if (oregon.sens_type != THGN132 && oregon.sens_type != THN132) {
      LOG(3, "Unknown sensor type, skip");
      goto end_capture;
    }

    if (debugLevel >= 2) {
      String s = "Oregon: CHNL " + String(oregon.sens_chnl);
      s += " BAT ";
      s += (oregon.sens_battery ? "F " : "E ");
      s += "ID ";
      s += String(oregon.sens_id, HEX);
      s += " TMP ";
      s += String(oregon.sens_tmp, 2);
      s += "C HUM ";
      s += String(oregon.sens_hmdty, 2);
      LOG(2, s);
    }

    if (oregon.sens_chnl > 0 && oregon.sens_chnl <= NOF_132) {
      byte chnl = oregon.sens_chnl - 1;
      tsensor[chnl].chnl = oregon.sens_chnl;
      tsensor[chnl].type = oregon.sens_type;
      tsensor[chnl].battery = oregon.sens_battery;

      // Keep AI‑style smoothing; doesn’t affect reception, only values.
      // Сохраняем экспоненциальное сглаживание; на приём не влияет.
      if (tsensor[chnl].numberofreceiving == 0) {
        tsensor[chnl].temperature = oregon.sens_tmp;
        tsensor[chnl].humidity = oregon.sens_hmdty;
      } else {
        tsensor[chnl].temperature =
          SMOOTH_ALPHA * oregon.sens_tmp + (1.0f - SMOOTH_ALPHA) * tsensor[chnl].temperature;
        tsensor[chnl].humidity =
          SMOOTH_ALPHA * oregon.sens_hmdty + (1.0f - SMOOTH_ALPHA) * tsensor[chnl].humidity;
      }

      tsensor[chnl].numberofreceiving++;
      tsensor[chnl].rcvtime = now;
      tsensor[chnl].isreceived = true;
    }
  }
end_capture:

  server.handleClient();
}

// ---------- WiFi connect ----------
void wifi_connect() {
  unsigned long curmark = millis();
  bool blink = false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  LOG(0, "Connecting to " + String(ssid));

  while (WiFi.status() != WL_CONNECTED) {
    if (blink) digitalWrite(BLUE_LED, LOW);
    else digitalWrite(BLUE_LED, HIGH);
    blink = !blink;
    delay(500);

    if (millis() - curmark > CONNECTTIMEOUT) {
      blink = false;
      digitalWrite(BLUE_LED, HIGH);
      WiFi.disconnect();
      delay(3000);
      curmark = millis();
      WiFi.begin(ssid, password);
      LOG(0, "Reconnect to " + String(ssid));
    }
  }

  LOG(0, "WiFi connected, IP=" + WiFi.localIP().toString());
}

// ---------- NTP time sync ----------
void syncTime() {
  long gmtOffset_sec = (long)tzOffsetHours * 3600L;
  long daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  LOG(0, "Syncing time via NTP, tz=" + String(tzOffsetHours));
  for (int i = 0; i < 10; i++) {
    time_t now = time(nullptr);
    if (now > 100000) {
      LOG(0, "Time synced: " + getTimeString());
      return;
    }
    delay(500);
  }
  LOG(1, "Time sync failed");
}

// ---------- Send data to narodmon (HTTP JSON) ----------
bool senddata() {
  if (WiFi.status() != WL_CONNECTED) wifi_connect();

  String json = buildNarodmonJson();
  LOG(3, "Narodmon JSON:\n" + json);

  if (TESTMODE) {
    lastConnectionTime = millis();
    for (int i = 0; i < NOF_132; i++) tsensor[i].numberofreceiving = 0;
    return true;
  }

  if (!narodClient.connect("narodmon.ru", 80)) {
    LOG(1, "connection to narodmon.ru failed");
    return false;
  }

  String req = String("POST /json HTTP/1.0\r\n") + "Host: narodmon.ru\r\n" + "Content-Type: application/x-www-form-urlencoded\r\n" + "Content-Length: " + String(json.length()) + "\r\n\r\n" + json;

  narodClient.print(req);

  unsigned long curmark = millis();
  while (!narodClient.available()) {
    if (millis() - curmark > DISCONNECTTIMEOUT) {
      LOG(1, "narodmon.ru no response");
      narodClient.stop();
      return false;
    }
    delay(10);
  }

  String response;
  while (narodClient.available()) {
    response += narodClient.readString();
  }
  narodClient.stop();
  LOG(1, "Narodmon response:\n" + response);

  lastConnectionTime = millis();
  // сбрасываем только счётчики для усреднения, как в оригинале
  for (int i = 0; i < NOF_132; i++) tsensor[i].numberofreceiving = 0;

  if (response.indexOf("\"error\":\"OK\"") >= 0) {
    return true;
  }
  return false;
}

// ---------- JSON builder for narodmon ----------
String buildNarodmonJson() {
  String json = "{";
  json += "\"devices\":[{";
  json += "\"mac\":\"" mac "\"";

  json += ",\"sensors\":[";
  bool first = true;

  // Oregon sensors / Датчики Oregon
  for (byte i = 0; i < NOF_132; i++) {
    if (tsensor[i].numberofreceiving == 0) continue;

    if (!first) json += ",";
    first = false;

    // Temperature sensor name (human‑readable) / Имя сенсора температуры
    String chName = "Oregon Ch" + String(tsensor[i].chnl) + " Temp";
    if (tsensor[i].chnl == 1) chName = "Bedroom Temp";
    if (tsensor[i].chnl == 2) chName = "Outdoor Temp";
    if (tsensor[i].chnl == 3) chName = "Channel3 Temp";

    json += "{";
    json += "\"id\":\"T" + String(tsensor[i].chnl) + "\",";
    json += "\"name\":\"" + chName + "\",";
    json += "\"value\":" + String(tsensor[i].temperature, 2) + ",";
    json += "\"unit\":\"C\"";
    json += "}";

    // Humidity sensor name / Имя сенсора влажности
    json += ",{";
    String hName = "Oregon Ch" + String(tsensor[i].chnl) + " Hum";
    if (tsensor[i].chnl == 1) hName = "Bedroom Hum";
    if (tsensor[i].chnl == 2) hName = "Outdoor Hum";
    if (tsensor[i].chnl == 3) hName = "Channel3 Hum";
    json += "\"id\":\"H" + String(tsensor[i].chnl) + "\",";
    json += "\"name\":\"" + hName + "\",";
    json += "\"value\":" + String(tsensor[i].humidity, 2) + ",";
    json += "\"unit\":\"%\"";
    json += "}";
  }

  // BMP180 temperature / Температура BMP180
  if (!first) json += ",";
  first = false;
  json += "{";
  json += "\"id\":\"TBMP\",";
  json += "\"name\":\"BMP180 Temp\",";
  json += "\"value\":" + String(bmp.readTemperature(), 2) + ",";
  json += "\"unit\":\"C\"";
  json += "}";

  // BMP180 pressure (hPa) / Давление BMP180 (гПа)
  float press_hpa = bmp.readPressure() / 100.0;
  json += ",{";
  json += "\"id\":\"PBMP\",";
  json += "\"name\":\"BMP180 Pressure\",";
  json += "\"value\":" + String(press_hpa, 2) + ",";
  json += "\"unit\":\"hPa\"";
  json += "}";

  // WiFi RSSI (0..100%) / Уровень Wi‑Fi в %
  int wifiPercent = wifiRssiToPercent(WiFi.RSSI());
  json += ",{";
  json += "\"id\":\"WIFI\",";
  json += "\"name\":\"WiFi RSSI\",";
  json += "\"value\":" + String(wifiPercent) + ",";
  json += "\"unit\":\"%\"";
  json += "}";

  json += "]}]}";
  return json;
}

// ---------- EEPROM config ----------
void saveLogConfig() {
  LogConfig cfg;
  cfg.magic = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;
  cfg.debugLevel = debugLevel;
  cfg.logToSerial = logToSerial ? 1 : 0;
  cfg.tzOffsetHours = tzOffsetHours;
  cfg.refreshMainSec = refreshMainSec;
  cfg.refreshLogSec = refreshLogSec;

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

// Load config from EEPROM or use defaults
// Загрузка конфигурации из EEPROM или применяем значения по умолчанию
void loadLogConfig() {
  EEPROM.begin(EEPROM_SIZE);
  LogConfig cfg;
  EEPROM.get(0, cfg);

  if (cfg.magic == EEPROM_MAGIC && cfg.version == EEPROM_VERSION) {
    if (cfg.debugLevel <= 3) debugLevel = cfg.debugLevel;
    logToSerial = (cfg.logToSerial != 0);
    tzOffsetHours = cfg.tzOffsetHours;
    refreshMainSec = cfg.refreshMainSec;
    refreshLogSec = cfg.refreshLogSec;
    if (tzOffsetHours < -12 || tzOffsetHours > 14) tzOffsetHours = 9;
    if (refreshMainSec == 0) refreshMainSec = 30;
    if (refreshLogSec == 0) refreshLogSec = 5;
  } else {
    debugLevel = DEBUG_LEVEL_DEFAULT;
    logToSerial = true;
    tzOffsetHours = 9;
    refreshMainSec = 30;
    refreshLogSec = 5;
  }
}
