/*
  Кренометр · компас · путь — для Mitsubishi Pajero Sport (KH, 4M41)
  Плата: Waveshare ESP32-S3-Touch-AMOLED-1.43 (AMOLED 466x466 QSPI, CO5300/SH8601, FT3168, QMI8658)

  Экраны (свайп влево/вправо):
   1. Кренометр — крен и дифферент, силуэты машины, дуговые шкалы
   2. Компас    — по BNO085 (магнитный); пока его нет — по гироскопу (относительный)
   3. Путь      — трек по счислению: скорость с CAN + курс; пройдено, до старта
   4–7. Наддув, температура ОЖ, обороты, G-метр (можно отключить: EXTRA_PAGES 0)

  Управление:
   касание         — сбросить максимумы на текущем экране
   двойное касание — Wi-Fi вкл/выкл (точки страниц синие = Wi-Fi включён)
   удержание 3 с   — кренометр: обнулить углы; путь: начать новую поездку

  Подключения к разъёму SH1.0:
   CAN (SN65HVD230):  CTX -> GPIO17, CRX -> GPIO16; CANH -> OBD 6, CANL -> OBD 14
   BNO085 (I2C):      на разъём I2C платы: SDA -> GPIO47, SCL -> GPIO48, питание 3.3 В
                      модуль кладётся ГОРИЗОНТАЛЬНО, стрелкой Y вперёд по ходу машины

  Библиотеки (Менеджер библиотек):
   "GFX Library for Arduino" (moononournation)
   "Adafruit BNO08x" (вместе с зависимостями) — если BNO085 не нужен, USE_BNO085 0
  Файл fonts.h должен лежать в той же папке, что и этот .ino.

  Arduino IDE: "ESP32S3 Dev Module", Flash 16MB, PSRAM "OPI PSRAM", USB CDC On Boot "Enabled",
  Partition Scheme "16M Flash (3MB APP/9.9MB FATFS)".
  Обновление по Wi-Fi: http://192.168.4.1, файл krenometr.ino.bin (Скетч → Экспорт бинарного файла).

  v3.10 — Android-приложение «Кренометр» на магнитоле:
   - UDP 4210: приложение раз в секунду шлёт «K1 w=0/1», кренометр отвечает пакетами JSON 25 раз/с
   - сеть «Krenometr» может работать постоянно (без отключения через 10 мин); адаптер ELM327 при этом
     ищется только на его канале (канал запоминается), чтобы магнитолу не выбивало из сети
   - экран кренометра можно выключить (из приложения или со страницы 192.168.4.1); касание — включить
   - новые адреса: /scr?on=0|1, /ap?on=0|1, /zero, /trip, /trk, /cmdend, /info

  v3.11 — связь с магнитолой по USB и GPS магнитолы:
   - кабель USB к магнитоле: питание и данные. Приложение шлёт строки «K1 w=0», кренометр отвечает
     строками JSON 25 раз/с; запросы «Q<номер> /адрес?…» → ответ «R<номер> текст» (\n внутри текста — перевод строки)
   - строки «G1 широта долгота точность_м скорость_м/с» (по UDP или USB): путь подтягивается к GPS,
     а без скорости с ELM327 путь считается по скорости GPS
*/

#pragma GCC optimize("O2")
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Update.h>
#include "driver/twai.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include <Arduino_GFX_Library.h>
#include <math.h>
#include <time.h>
#include <algorithm>
#include "fonts.h"
#include "font_date.h"
#include "dial_vintage.h"

// Объявление заранее: Arduino сам вставляет прототипы функций в начало файла,
// и тип Gauge должен быть известен уже там
struct Gauge;
struct ClockData;

// ===================== НАСТРОЙКИ =====================
#define ROLL_WARN      15     // крен: жёлтая зона, градусы
#define ROLL_DANGER    25     // крен: красная зона
#define PITCH_WARN     15     // дифферент: жёлтая зона
#define PITCH_DANGER   25     // дифферент: красная зона
#define SCALE_MAX      45     // предел дуговых шкал кренометра

#define FORWARD_SIGN   1      // дифферент наоборот — поставьте -1
#define ROLL_SIGN      1      // крен наоборот — поставьте -1

#define USE_BNO085     1      // магнитный компас BNO085
#define BNO_FWD_AXIS   2      // какая ось BNO085 смотрит вперёд: 0 = X, 1 = Y, 2 = Z
#define TILT_FROM_BNO  1      // крен и дифферент считать по BNO085, когда он есть (0 — по датчику платы)
#define BNO_FWD_SIGN   1      // если компас врёт на 180° — поставьте -1
#define MAG_DECLINATION 12.0f // магнитное склонение, Москва ≈ +12°
#define HEADING_TRIM   0.0f   // доводка курса, если модуль стоит чуть криво

#define TRACK_STEP_M   5.0f   // точка трека каждые N метров
#define TRACK_MAX      3000   // максимум точек (при переполнении трек прореживается)

#define EXTRA_PAGES    1      // экраны наддува, ОЖ, оборотов и G-метра
#define BOOST_MAX      2.0f   // бар
#define BOOST_WARN     1.3f
#define BOOST_DANGER   1.5f
#define COOL_COLD      60
#define COOL_WARN      100
#define COOL_DANGER    106
#define RPM_MAX        4500
#define RPM_WARN       3800
#define RPM_DANGER     4100

#define FUSION_TAU     4.0f   // сек: насколько медленно акселерометр поправляет гироскоп
#define DISPLAY_TAU    0.12f  // сек: сглаживание отображения
#define BRIGHTNESS     220    // яркость AMOLED 0..255
#define PANEL_CO5300   1      // контроллер экрана: 1 = CO5300, 0 = SH8601 (если картинки нет — попробуйте 0)
#define PANEL_COL_OFFSET 6    // сдвиг столбцов CO5300 у 1.43"; если картинка сдвинута вбок — 0
#define QSPI_HZ        40000000  // частота шины экрана; можно попробовать 80000000 (если картинка не портится)
#define LONGPRESS_MS   3000
#define DISPLAY_ROT    1      // поворот картинки: 0 — нет, 1 — 90° по часовой, 2 — 180°, 3 — 90° против (меняется и на странице 192.168.4.1)

// Откуда брать данные машины (скорость, обороты, наддув…):
// 0 — ниоткуда, 1 — Wi-Fi адаптер ELM327 (как приложение «ELM CAN Терминал»), 2 — свой CAN-трансивер SN65HVD230
#define CAR_LINK       1
#define USE_CAN        (CAR_LINK != 0)
// Настройки ELM327 по умолчанию (как в приложении-терминале); меняются на странице 192.168.4.1
#define ELM_SSID       "WiFi_OBDII"
#define ELM_PASS       ""
#define ELM_HOST       "192.168.0.10"
#define ELM_PORT       35000
#define ELM_PROTO      "6"     // ATSP: 6 — CAN 11 бит 500 кбит/с
#define FW_VERSION     "3.11"
#define APP_UDP_PORT   4210    // поток данных для Android-приложения
#define APP_PERIOD_MS  40      // 25 пакетов в секунду
#define APP_TIMEOUT_MS 3000    // приложение молчит дольше — перестаём слать
#define WIFI_SSID      "Krenometr"
#define WIFI_PASS      "krenometr123"
#define WIFI_TIMEOUT_MIN 10
// =====================================================

#if USE_BNO085
#include <Adafruit_BNO08x.h>
#endif

// ---- Выводы ----
// Waveshare ESP32-S3-Touch-AMOLED-1.43 (по вики Waveshare)
#define PIN_LCD_CS    9
#define PIN_LCD_SCK   10
#define PIN_LCD_D0    11
#define PIN_LCD_D1    12
#define PIN_LCD_D2    13
#define PIN_LCD_D3    14
#define PIN_LCD_RST   21
#define PIN_LCD_EN    42      // питание AMOLED
#define PIN_I2C_SDA   47      // общая шина: тач, QMI8658, RTC, разъём I2C (сюда же BNO085)
#define PIN_I2C_SCL   48
#define PIN_CAN_TX    17
#define PIN_CAN_RX    16

#define TOUCH_ADDR    0x38    // FT3168
#define DEG           (PI / 180.0f)

// ---- Дисплей ----
#define PW 466                // физическое разрешение экрана
// библиотека GFX только включает экран; дальше кадры шлёт свой быстрый драйвер (lcdFast*)
Arduino_DataBus *bus = new Arduino_ESP32QSPI(PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3, true);
#if PANEL_CO5300
Arduino_CO5300 *panel = new Arduino_CO5300(bus, PIN_LCD_RST, 0, PW, PW, PANEL_COL_OFFSET, 0, 0, 0);
#else
Arduino_SH8601 *panel = new Arduino_SH8601(bus, PIN_LCD_RST, 0, PW, PW);
#endif
// Кадр рисуется полосами во внутренней памяти и сразу отправляется на экран (см. present())

// ---- Цвета iOS (тёмная тема) ----
static constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t IOS_BG      = rgb(0, 0, 0);
constexpr uint16_t IOS_WHITE   = rgb(255, 255, 255);
constexpr uint16_t IOS_GRAY    = rgb(142, 142, 147);   // secondaryLabel
constexpr uint16_t IOS_GRAY2   = rgb(99, 99, 102);     // systemGray2
constexpr uint16_t IOS_GRAY4   = rgb(58, 58, 60);      // systemGray4
constexpr uint16_t IOS_TRACK   = rgb(44, 44, 46);      // systemGray5
constexpr uint16_t IOS_CARD    = rgb(28, 28, 30);      // systemGray6
constexpr uint16_t IOS_GREEN   = rgb(48, 209, 88);
constexpr uint16_t IOS_YELLOW  = rgb(255, 214, 10);
constexpr uint16_t IOS_ORANGE  = rgb(255, 159, 10);
constexpr uint16_t IOS_RED     = rgb(255, 69, 58);
constexpr uint16_t IOS_BLUE    = rgb(10, 132, 255);
constexpr uint16_t IOS_TEAL    = rgb(100, 210, 255);

// ===================== Общие данные между задачами =====================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile float g_roll = 0, g_pitch = 0;
volatile float g_gLon = 0, g_gLat = 0;
volatile bool  g_imuOk = false;
volatile bool  g_calibrating = true;
volatile bool  g_touch = false;
volatile int16_t g_tx = 0, g_ty = 0;
volatile bool  g_zeroRequest = false;
volatile uint32_t g_zeroDoneAt = 0;

// курс
volatile float g_heading = 0;          // градусы 0..360
volatile int   g_tiltSrc = 1;          // 1 — датчик платы, 2 — BNO085
volatile int   g_headingSrc = 0;       // 0 — нет, 1 — гироскоп, 2 — магнитный (BNO085)
volatile int   g_magAccuracy = 0;      // 0..3, точность калибровки BNO085
volatile bool  g_bnoFound = false;

// путь (метры: x — на восток, y — на север, от точки старта)
volatile float g_posX = 0, g_posY = 0, g_odometer = 0;
volatile bool  g_tripReset = false;
volatile uint32_t g_tripId = 0;        // растёт при каждой новой поездке (приложение перечитывает трек)
// GPS магнитолы (v3.11): скорость — запасная для счисления, координаты — мягкая поправка пути
volatile float    g_gpsAcc = -1, g_gpsSpeed = 0;   // точность, м; скорость, м/с
volatile uint32_t g_gpsAt = 0;
volatile float    g_gpsCorrX = 0, g_gpsCorrY = 0;  // накопленная поправка, м (забирает задача датчиков)
float trackX[TRACK_MAX], trackY[TRACK_MAX];
volatile int trackN = 0;
SemaphoreHandle_t trackLock;

// CAN
volatile bool     g_canOk = false;
volatile float    g_speedKmh = 0;
volatile uint32_t g_speedAt = 0;
volatile bool     g_speedComp = false;
volatile int      g_rpm = -1;      volatile uint32_t g_rpmAt = 0;
volatile int      g_coolant = -999; volatile uint32_t g_coolAt = 0;
volatile float    g_volt = -1;
volatile float    g_mapKpa = 0;    volatile uint32_t g_mapAt = 0;
volatile float    g_baroKpa = 0;   volatile uint32_t g_baroAt = 0;
volatile int      g_iat = -999;    volatile uint32_t g_iatAt = 0;

// ===================== Векторы =====================
static inline float dot3(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static inline void cross3(const float a[3], const float b[3], float o[3]) {
  o[0] = a[1]*b[2] - a[2]*b[1];
  o[1] = a[2]*b[0] - a[0]*b[2];
  o[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float norm3(const float a[3]) { return sqrtf(dot3(a, a)); }
static inline void normalize3(float a[3]) {
  float n = norm3(a);
  if (n > 1e-6f) { a[0] /= n; a[1] /= n; a[2] /= n; }
}
static inline float wrap360(float a) { a = fmodf(a, 360.0f); return a < 0 ? a + 360.0f : a; }

// ===================== I2C =====================
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ===================== Часы реального времени PCF85063 =====================
#define RTC_ADDR 0x51
struct ClockData { uint8_t hh, mm, ss, day, mon, wday; uint16_t year; bool ok, valid; uint32_t secAt; };
ClockData g_clock = {10, 10, 0, 1, 1, 0, 2026, false, false, 0};
volatile bool g_rtcSetReq = false;
volatile uint32_t g_rtcSetEpoch = 0;          // местное время, секунды от 1970

static uint8_t fromBcd(uint8_t v) { return (v >> 4) * 10 + (v & 15); }
static uint8_t toBcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// Вызывается из задачи датчиков (там же вся работа с I2C)
void rtcService() {
  static uint32_t lastRead = 0;
  static int lastSec = -1;
  if (g_rtcSetReq) {
    time_t t = (time_t)g_rtcSetEpoch;
    struct tm tm;
    gmtime_r(&t, &tm);
    uint8_t b[8] = { 0x04, toBcd(tm.tm_sec), toBcd(tm.tm_min), toBcd(tm.tm_hour), toBcd(tm.tm_mday),
                     (uint8_t)tm.tm_wday, toBcd(tm.tm_mon + 1), toBcd(tm.tm_year % 100) };
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(b, 8);
    Wire.endTransmission();
    g_rtcSetReq = false;
    lastRead = 0; lastSec = -1;
  }
  if (millis() - lastRead < 100) return;
  lastRead = millis();
  uint8_t r[7];
  ClockData c;
  portENTER_CRITICAL(&mux); c = g_clock; portEXIT_CRITICAL(&mux);
  if (!i2cRead(RTC_ADDR, 0x04, r, 7)) { c.ok = false; }
  else {
    c.ok = true;
    c.valid = !(r[0] & 0x80);                    // флаг остановки генератора: время потеряно
    int sec = fromBcd(r[0] & 0x7F);
    if (sec != lastSec) { c.secAt = millis(); lastSec = sec; }
    c.ss = sec; c.mm = fromBcd(r[1] & 0x7F); c.hh = fromBcd(r[2] & 0x3F);
    c.day = fromBcd(r[3] & 0x3F); c.wday = r[4] & 7; c.mon = fromBcd(r[5] & 0x1F); c.year = 2000 + fromBcd(r[6]);
  }
  portENTER_CRITICAL(&mux); g_clock = c; portEXIT_CRITICAL(&mux);
}

// ===================== QMI8658 =====================
uint8_t qmiAddr = 0x6B;

bool qmiInit() {
  const uint8_t addrs[2] = {0x6B, 0x6A};
  bool found = false;
  for (uint8_t i = 0; i < 2 && !found; i++) {
    uint8_t who = 0;
    if (i2cRead(addrs[i], 0x00, &who, 1) && who == 0x05) { qmiAddr = addrs[i]; found = true; }
  }
  if (!found) return false;
  i2cWrite(qmiAddr, 0x60, 0xB0);   // сброс
  delay(20);
  i2cWrite(qmiAddr, 0x02, 0x40);   // автоинкремент адреса
  i2cWrite(qmiAddr, 0x03, 0x15);   // акселерометр ±4g, ~235 Гц
  i2cWrite(qmiAddr, 0x04, 0x55);   // гироскоп ±512°/с, ~235 Гц
  i2cWrite(qmiAddr, 0x06, 0x11);   // ФНЧ
  i2cWrite(qmiAddr, 0x08, 0x03);   // включить
  delay(30);
  return true;
}

bool qmiRead(float a[3], float w[3]) {
  uint8_t b[12];
  if (!i2cRead(qmiAddr, 0x35, b, 12)) return false;
  int16_t r[6];
  for (int i = 0; i < 6; i++) r[i] = (int16_t)(b[2*i] | (b[2*i + 1] << 8));
  const float aS = 1.0f / 8192.0f;
  const float gS = (1.0f / 64.0f) * DEG;
  for (int i = 0; i < 3; i++) { a[i] = r[i] * aS; w[i] = r[i + 3] * gS; }
  return true;
}

// ===================== Тач FT3168 =====================
void touchInit() {}

bool touchPressed() {
  uint8_t b[5];
  if (!i2cRead(TOUCH_ADDR, 0x02, b, 5)) return false;
  if ((b[0] & 0x0F) == 0) return false;
  // координаты экрана 466x466 -> логические 240x240
  int16_t tx = (int16_t)(((((b[1] & 0x0F) << 8) | b[2])) * 240 / PW);
  int16_t ty = (int16_t)(((((b[3] & 0x0F) << 8) | b[4])) * 240 / PW);
  extern int g_rot;                              // касание — в координаты повёрнутой картинки
  switch (g_rot) {
    case 1:  g_tx = ty;       g_ty = 239 - tx; break;
    case 2:  g_tx = 239 - tx; g_ty = 239 - ty; break;
    case 3:  g_tx = 239 - ty; g_ty = tx;       break;
    default: g_tx = tx;       g_ty = ty;       break;
  }
  return true;
}

// ===================== Ориентация =====================
float u[3]    = {0, 0, 1};   // «вверх» в осях QMI8658
float bias[3] = {0, 0, 0};
float F[3]    = {0, 0, 1};   // «вперёд» в осях QMI8658
float R[3]    = {1, 0, 0};   // «вправо»
Preferences prefs;

void setZeroFromCurrent(bool save) {
  float z[3] = {0, 0, 1};
  float p = dot3(z, u);
  float f[3] = { z[0] - p*u[0], z[1] - p*u[1], z[2] - p*u[2] };
  if (norm3(f) < 0.5f) {
    float y[3] = {0, 1, 0};
    p = dot3(y, u);
    f[0] = y[0] - p*u[0]; f[1] = y[1] - p*u[1]; f[2] = y[2] - p*u[2];
  }
  normalize3(f);
  for (int i = 0; i < 3; i++) F[i] = f[i] * FORWARD_SIGN;
  cross3(F, u, R);
  normalize3(R);
  if (save) {
    float buf[6] = {F[0], F[1], F[2], R[0], R[1], R[2]};
    prefs.putBytes("frame", buf, sizeof(buf));
  }
}

bool loadZero() {
  float buf[6];
  if (prefs.getBytesLength("frame") != sizeof(buf)) return false;
  prefs.getBytes("frame", buf, sizeof(buf));
  for (int i = 0; i < 3; i++) { F[i] = buf[i]; R[i] = buf[i + 3]; }
  return true;
}

void calibrateGyro() {
  for (int attempt = 0; attempt < 5; attempt++) {
    float sw[3] = {0, 0, 0}, sa[3] = {0, 0, 0}, sw2 = 0;
    const int N = 300;
    int got = 0;
    for (int i = 0; i < N; i++) {
      float a[3], w[3];
      if (qmiRead(a, w)) {
        for (int k = 0; k < 3; k++) { sw[k] += w[k]; sa[k] += a[k]; }
        sw2 += dot3(w, w);
        got++;
      }
      delay(5);
    }
    if (got < N / 2) continue;
    float mw[3] = { sw[0]/got, sw[1]/got, sw[2]/got };
    float var = sw2 / got - dot3(mw, mw);
    for (int k = 0; k < 3; k++) { bias[k] = mw[k]; u[k] = sa[k] / got; }
    normalize3(u);
    if (var < 0.0004f) break;
  }
}

// Гироскоп + акселерометр; при известной скорости вычитаем ускорение машины
void fuse(const float aIn[3], const float wRaw[3], float dt, bool comp, float v, float dvdt) {
  float w[3] = { wRaw[0] - bias[0], wRaw[1] - bias[1], wRaw[2] - bias[2] };
  float c[3];
  cross3(u, w, c);
  for (int i = 0; i < 3; i++) u[i] += c[i] * dt;
  normalize3(u);

  float a[3] = { aIn[0], aIn[1], aIn[2] };
  if (comp) {
    float wxF[3];
    cross3(w, F, wxF);
    for (int i = 0; i < 3; i++) a[i] -= (dvdt * F[i] + v * wxF[i]) / 9.81f;
  }

  float an = norm3(a);
  if (an < 0.3f) return;
  float k = dt / (comp ? FUSION_TAU * 0.5f : FUSION_TAU);

  float dev = fabsf(an - 1.0f);
  const float D0 = comp ? 0.06f : 0.03f, D1 = comp ? 0.25f : 0.15f;
  if (dev > D1) k = 0;
  else if (dev > D0) { float t = 1.0f - (dev - D0) / (D1 - D0); k *= t * t; }

  float yaw = fabsf(dot3(w, u));
  const float Y0 = comp ? 0.25f : 0.05f, Y1 = comp ? 0.60f : 0.20f;
  if (yaw > Y1) k = 0;
  else if (yaw > Y0) k *= 1.0f - (yaw - Y0) / (Y1 - Y0);

  for (int i = 0; i < 3; i++) u[i] += k * (a[i] / an - u[i]);
  normalize3(u);

  static float stillTime = 0;
  if (sqrtf(dot3(w, w)) < 0.02f && dev < 0.02f) stillTime += dt; else stillTime = 0;
  if (stillTime > 1.0f)
    for (int i = 0; i < 3; i++) bias[i] += 0.002f * (wRaw[i] - bias[i]);
}

// ===================== BNO085 (магнитный компас) =====================
#if USE_BNO085
Adafruit_BNO08x bno;
float bnoF[3] = {0, 1, 0};           // «вперёд» в осях BNO085
float bnoR[3] = {1, 0, 0};           // «вправо» в осях BNO085
bool  bnoFrameOk = false;
float bnoQ[4] = {1, 0, 0, 0};        // r, i, j, k (тело -> мир, мир: X восток, Y север, Z вверх)
uint32_t bnoAt = 0;

// повернуть вектор кватернионом q (или обратным, если inv)
void qrot(const float q[4], const float v[3], float o[3], bool inv) {
  float w = q[0], x = inv ? -q[1] : q[1], y = inv ? -q[2] : q[2], z = inv ? -q[3] : q[3];
  float tx = 2 * (y * v[2] - z * v[1]);
  float ty = 2 * (z * v[0] - x * v[2]);
  float tz = 2 * (x * v[1] - y * v[0]);
  o[0] = v[0] + w * tx + (y * tz - z * ty);
  o[1] = v[1] + w * ty + (z * tx - x * tz);
  o[2] = v[2] + w * tz + (x * ty - y * tx);
}

// ось «вперёд» модуля: выбранная ось, спроецированная на горизонталь
void bnoSaveFrame();
void bnoSetFrame(bool save) {
  float up[3], zw[3] = {0, 0, 1};
  qrot(bnoQ, zw, up, true);           // «вверх» в осях модуля
  float ax[3] = {0, 0, 0};
  ax[BNO_FWD_AXIS] = BNO_FWD_SIGN;
  float p = dot3(ax, up);
  float f[3] = { ax[0] - p*up[0], ax[1] - p*up[1], ax[2] - p*up[2] };
  if (norm3(f) < 0.3f) return;        // ось смотрит вертикально — не годится
  normalize3(f);
  for (int i = 0; i < 3; i++) bnoF[i] = f[i];
  cross3(bnoF, up, bnoR);
  normalize3(bnoR);
  bnoFrameOk = true;
  if (save) bnoSaveFrame();
}

void bnoSaveFrame() {
  float buf[6] = {bnoF[0], bnoF[1], bnoF[2], bnoR[0], bnoR[1], bnoR[2]};
  prefs.putBytes("bnoFR", buf, sizeof(buf));
}

// Крен и дифферент по BNO085 — так же, как по датчику платы: «вверх» в осях модуля против осей «вперёд»/«вправо»
bool bnoTilt(float &roll, float &pitch) {
  if (!bnoFrameOk || millis() - bnoAt > 300) return false;
  float up[3], zw[3] = {0, 0, 1};
  qrot(bnoQ, zw, up, true);
  pitch = asinf(constrain(dot3(up, bnoF), -1.0f, 1.0f)) / DEG;
  roll  = -asinf(constrain(dot3(up, bnoR), -1.0f, 1.0f)) / DEG * ROLL_SIGN;
  return true;
}

// Сам определяет, не смотрит ли ось модуля назад: сравнивает знак дифферента с датчиком платы.
// Если долго «наоборот» — разворачивает ось (заодно исправляется компас на 180°).
void bnoCheckSign(float pitchQmi, float pitchBno) {
  static float score = 0; static int n = 0;
  if (fabsf(pitchQmi) < 3 || fabsf(pitchBno) < 3) return;
  score += (pitchQmi * pitchBno > 0) ? 1 : -1;
  if (++n < 200) return;
  if (score < -120) {
    for (int i = 0; i < 3; i++) { bnoF[i] = -bnoF[i]; bnoR[i] = -bnoR[i]; }
    bnoSaveFrame();
    Serial.println("BNO085: forward axis flipped");
  }
  score = 0; n = 0;
}

bool bnoBegin() {
  if (!bno.begin_I2C(0x4A, &Wire) && !bno.begin_I2C(0x4B, &Wire)) return false;
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);   // 100 Гц
  float buf[6];
  if (prefs.getBytesLength("bnoFR") == sizeof(buf)) {
    prefs.getBytes("bnoFR", buf, sizeof(buf));
    for (int i = 0; i < 3; i++) { bnoF[i] = buf[i]; bnoR[i] = buf[i + 3]; }
    bnoFrameOk = true;
  }
  return true;
}

// true — пришли новые данные, heading заполнен
bool bnoPoll(float &heading) {
  if (bno.wasReset()) bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  sh2_SensorValue_t v;
  bool got = false;
  while (bno.getSensorEvent(&v)) {
    if (v.sensorId != SH2_ROTATION_VECTOR) continue;
    bnoQ[0] = v.un.rotationVector.real;
    bnoQ[1] = v.un.rotationVector.i;
    bnoQ[2] = v.un.rotationVector.j;
    bnoQ[3] = v.un.rotationVector.k;
    g_magAccuracy = v.status & 0x03;
    bnoAt = millis();
    got = true;
  }
  if (!got) return false;
  if (!bnoFrameOk) bnoSetFrame(true);    // первый запуск: машина стоит как обычно
  if (!bnoFrameOk) return false;
  float fw[3];
  qrot(bnoQ, bnoF, fw, false);
  heading = wrap360(atan2f(fw[0], fw[1]) / DEG + MAG_DECLINATION + HEADING_TRIM);
  return true;
}
#endif

// ===================== Задача датчиков (ядро 0) =====================
void trackResetLocked() {
  trackN = 0;
  trackX[0] = 0; trackY[0] = 0; trackN = 1;
}

void trackAppend(float x, float y) {
  if (xSemaphoreTake(trackLock, 0) != pdTRUE) return;   // интерфейс читает — в следующий раз
  if (trackN >= TRACK_MAX) {                             // переполнение: оставляем каждую вторую
    int n = 0;
    for (int i = 0; i < trackN; i += 2) { trackX[n] = trackX[i]; trackY[n] = trackY[i]; n++; }
    trackN = n;
  }
  trackX[trackN] = x; trackY[trackN] = y; trackN = trackN + 1;
  xSemaphoreGive(trackLock);
}

void sensorTask(void *) {
  if (!qmiInit()) { Serial.println("ERROR: QMI8658 not found"); g_imuOk = false; g_calibrating = false; vTaskDelete(NULL); return; }
  Serial.printf("QMI8658 at 0x%02X\n", qmiAddr);
  g_imuOk = true;
  touchInit();
#if USE_BNO085
  g_bnoFound = bnoBegin();
#endif

  calibrateGyro();
  if (!loadZero()) setZeroFromCurrent(true);
  g_calibrating = false;

  float rollF = 0, pitchF = 0, gLon = 0, gLat = 0;
  bool first = true;
  uint32_t last = micros();
  uint32_t lastTouch = 0;

  float vPrev = 0, vF = 0, dvdt = 0;
  uint32_t lastSpdAt = 0;

  float gyroHdg = 0;                 // курс по гироскопу (относительный, пока нет магнитного)
  bool  hdgAbsolute = false;
  float posX = 0, posY = 0, odo = 0, lastPtX = 0, lastPtY = 0;
  xSemaphoreTake(trackLock, portMAX_DELAY);
  trackResetLocked();
  xSemaphoreGive(trackLock);

  for (;;) {
    float a[3], w[3];
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f;
    last = now;
    if (dt > 0.05f) dt = 0.05f;

    // --- скорость с CAN ---
    uint32_t spdAt = g_speedAt;
    bool comp = USE_CAN && g_canOk && spdAt != 0 && (millis() - spdAt) < (CAR_LINK == 1 ? 1500 : 500);
    if (comp && spdAt != lastSpdAt) {
      float v = g_speedKmh / 3.6f;
      float dts = (spdAt - lastSpdAt) / 1000.0f;
      if (lastSpdAt != 0 && dts > 0.01f && dts < 0.6f) {
        float kk = dts / 0.35f; if (kk > 1) kk = 1;
        dvdt += kk * ((v - vPrev) / dts - dvdt);
      }
      vPrev = v;
      lastSpdAt = spdAt;
    }
    if (comp) { float kv = dt / 0.15f; if (kv > 1) kv = 1; vF += kv * (vPrev - vF); }
    else { vF = vPrev = dvdt = 0; lastSpdAt = 0; }
    g_speedComp = comp;

    if (qmiRead(a, w)) {
      fuse(a, w, dt, comp, vF, dvdt);

      if (g_zeroRequest) {
        setZeroFromCurrent(true);
#if USE_BNO085
        if (g_bnoFound && millis() - bnoAt < 500) bnoSetFrame(true);
#endif
        rollF = pitchF = 0;
        g_zeroRequest = false;
        g_zeroDoneAt = millis();
      }

      float pitch = asinf(constrain(dot3(u, F), -1.0f, 1.0f)) / DEG;
      float roll  = -asinf(constrain(dot3(u, R), -1.0f, 1.0f)) / DEG * ROLL_SIGN;
#if USE_BNO085
      {
        float rb, pb;
        if (g_bnoFound && bnoTilt(rb, pb)) {
          bnoCheckSign(pitch, pb);
          if (TILT_FROM_BNO) { roll = rb; pitch = pb; g_tiltSrc = 2; }
        } else g_tiltSrc = 1;
      }
#endif
      if (first) { rollF = roll; pitchF = pitch; first = false; }
      float kd = dt / DISPLAY_TAU;
      rollF  += kd * (roll  - rollF);
      pitchF += kd * (pitch - pitchF);

      float lin[3] = { a[0] - u[0], a[1] - u[1], a[2] - u[2] };
      float kg = dt / 0.15f; if (kg > 1) kg = 1;
      gLon += kg * (dot3(lin, F) - gLon);
      gLat += kg * (dot3(lin, R) * ROLL_SIGN - gLat);

      // --- курс: гироскоп всегда, магнитный — если есть ---
      float wb[3] = { w[0] - bias[0], w[1] - bias[1], w[2] - bias[2] };
      gyroHdg = wrap360(gyroHdg - dot3(wb, u) * dt / DEG);   // поворот влево — курс уменьшается
      float hdg = gyroHdg;
      int src = 1;
#if USE_BNO085
      float mh;
      static uint32_t bnoRetryAt = 0;
      if (!g_bnoFound && millis() - bnoRetryAt > 3000) { bnoRetryAt = millis(); g_bnoFound = bnoBegin(); }
      if (g_bnoFound && bnoPoll(mh)) { gyroHdg = mh; hdgAbsolute = true; }
      if (g_bnoFound && millis() - bnoAt < 500 && bnoFrameOk) { hdg = gyroHdg; src = 2; }
#endif
      (void)hdgAbsolute;

      // --- счисление пути ---
      if (g_tripReset) {
        posX = posY = odo = lastPtX = lastPtY = 0;
        xSemaphoreTake(trackLock, portMAX_DELAY);
        trackResetLocked();
        xSemaphoreGive(trackLock);
        g_tripId = g_tripId + 1;
        g_tripReset = false;
      }
      // поправка от GPS магнитолы
      {
        float cx, cy;
        portENTER_CRITICAL(&mux);
        cx = g_gpsCorrX; cy = g_gpsCorrY; g_gpsCorrX = 0; g_gpsCorrY = 0;
        portEXIT_CRITICAL(&mux);
        posX += cx; posY += cy;
      }
      // скорость: с ELM327, а если её нет — с GPS магнитолы
      bool gpsV = !comp && g_gpsAt != 0 && millis() - g_gpsAt < 2500 && g_gpsAcc > 0 && g_gpsAcc < 30;
      float vPath = comp ? vF : (gpsV ? (float)g_gpsSpeed : 0.0f);
      if (vPath > 0.3f) {
        float ds = vPath * dt;
        posX += ds * sinf(hdg * DEG);
        posY += ds * cosf(hdg * DEG);
        odo += ds;
        float ex = posX - lastPtX, ey = posY - lastPtY;
        if (ex * ex + ey * ey > TRACK_STEP_M * TRACK_STEP_M) {
          trackAppend(posX, posY);
          lastPtX = posX; lastPtY = posY;
        }
      }

      portENTER_CRITICAL(&mux);
      g_roll = rollF; g_pitch = pitchF;
      g_gLon = gLon; g_gLat = gLat;
      g_heading = hdg; g_headingSrc = src;
      g_posX = posX; g_posY = posY; g_odometer = odo;
      portEXIT_CRITICAL(&mux);
    }

    if (millis() - lastTouch > 30) {
      lastTouch = millis();
      g_touch = touchPressed();
    }
    rtcService();
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

// ===================== CAN / OBD2 (ядро 0) =====================
#define PID_SPEED   0x0D
#define PID_RPM     0x0C
#define PID_COOLANT 0x05
#define PID_VOLT    0x42
#define PID_MAP     0x0B
#define PID_BARO    0x33
#define PID_IAT     0x0F

bool canInstall() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.tx_queue_len = 4;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f;
  f.acceptance_code = 0x7E8u << 21;                 // ответы ЭБУ 0x7E8..0x7EF
  f.acceptance_mask = (0x7u << 21) | 0x1FFFFFu;
  f.single_filter = true;
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  gpio_pullup_en((gpio_num_t)PIN_CAN_RX);
  return twai_start() == ESP_OK;
}

void canSendRequest(uint8_t pid) {
  twai_message_t m = {};
  m.identifier = 0x7DF;
  m.extd = 0;
  m.data_length_code = 8;
  uint8_t d[8] = {0x02, 0x01, pid, 0x55, 0x55, 0x55, 0x55, 0x55};
  memcpy(m.data, d, 8);
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

bool canParse(const twai_message_t &r) {
  if (r.extd || r.identifier < 0x7E8 || r.identifier > 0x7EF) return false;
  if (r.data_length_code < 4 || r.data[1] != 0x41) return false;
  uint8_t pid = r.data[2], A = r.data[3], B = r.data_length_code > 4 ? r.data[4] : 0;
  uint32_t now = millis();
  switch (pid) {
    case PID_SPEED:   g_speedKmh = A; g_speedAt = now; break;
    case PID_RPM:     g_rpm = ((A << 8) | B) / 4; g_rpmAt = now; break;
    case PID_COOLANT: g_coolant = (int)A - 40; g_coolAt = now; break;
    case PID_VOLT:    g_volt = ((A << 8) | B) / 1000.0f; break;
    case PID_MAP:     g_mapKpa = A; g_mapAt = now; break;
    case PID_BARO:    g_baroKpa = A; g_baroAt = now; break;
    case PID_IAT:     g_iat = (int)A - 40; g_iatAt = now; break;
    default: return false;
  }
  return true;
}

void canTask(void *) {
  if (!canInstall()) { vTaskDelete(NULL); return; }
  const uint8_t seq[8] = {PID_SPEED, PID_MAP, PID_SPEED, PID_RPM, PID_SPEED, PID_MAP, PID_SPEED, 0};
  const uint8_t slow[4] = {PID_COOLANT, PID_IAT, PID_VOLT, PID_BARO};
  uint8_t idx = 0, slowIdx = 0;
  uint32_t lastReply = 0;

  for (;;) {
    bool alive = lastReply != 0 && millis() - lastReply < 2000;
    g_canOk = alive;
    uint8_t pid = seq[idx];
    if (pid == 0) { pid = slow[slowIdx]; slowIdx = (slowIdx + 1) % 4; }
    canSendRequest(pid);
    idx = (idx + 1) % 8;

    uint32_t wait = alive ? 40 : 1000;
    uint32_t start = millis();
    while (millis() - start < wait) {
      twai_message_t r;
      uint32_t left = wait - (millis() - start);
      if (twai_receive(&r, pdMS_TO_TICKS(left)) != ESP_OK) break;
      if (canParse(r)) lastReply = millis();
    }
    twai_status_info_t s;
    if (twai_get_status_info(&s) == ESP_OK) {
      if (s.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
      else if (s.state == TWAI_STATE_STOPPED) twai_start();
      if (!alive && s.msgs_to_tx > 0) twai_clear_transmit_queue();
    }
  }
}

// ===================== ELM327 по Wi-Fi (ядро 0) =====================
// Делает то же, что приложение-терминал: TCP 192.168.0.10:35000, ATZ ATE0 ATL0 ATS1 ATH1 ATAT1 ATSP6,
// затем по кругу опрашивает ЭБУ двигателя (7E0, режим 01) и иногда АКПП (7E1 21 02 — масло ATF).
extern bool wifiOn;
struct ElmCfg { char ssid[33]; char pass[65]; char host[40]; uint16_t port; char proto[4]; };
ElmCfg elmCfg;
volatile bool g_elmReconnect = false;
volatile int  g_elmState = 0;          // 0 — ищет Wi-Fi адаптера, 1 — нет TCP, 2 — адаптер есть, ЭБУ молчит, 3 — всё работает
volatile int  g_atf = -999;  volatile uint32_t g_atfAt = 0;
char g_elmId[24] = "";                 // ответ ATI
// терминал со страницы 192.168.4.1
char elmTermCmd[64];
volatile bool elmTermPending = false;
char elmTermOut[1024] = "";
volatile uint32_t elmTermSeq = 0;
uint32_t elmTermLastAt = 0;       // пока работают с терминалом (минуту), опрос стоит, как в приложении
Preferences prefsElm;
// Канал и BSSID сети адаптера: пока работает своя сеть «Krenometr», адаптер ищем только на этом
// канале — без перебора каналов, который выбивает магнитолу из сети.
volatile uint8_t elmChan = 0;          // 0 — канал ещё не известен
uint8_t elmBssid[6] = {0};
bool    elmBssidOk = false;
uint32_t elmScanAt = 0;               // когда последний раз искали сеть адаптера

void elmLoadCfg() {
  prefsElm.begin("elm", false);
  elmChan = prefsElm.isKey("ch") ? prefsElm.getUChar("ch", 0) : 0;
  elmBssidOk = prefsElm.isKey("bs") && prefsElm.getBytes("bs", elmBssid, 6) == 6;
  strlcpy(elmCfg.ssid, prefsElm.getString("ssid", ELM_SSID).c_str(), sizeof(elmCfg.ssid));
  strlcpy(elmCfg.pass, prefsElm.getString("pass", ELM_PASS).c_str(), sizeof(elmCfg.pass));
  strlcpy(elmCfg.host, prefsElm.getString("host", ELM_HOST).c_str(), sizeof(elmCfg.host));
  elmCfg.port = prefsElm.getUShort("port", ELM_PORT);
  strlcpy(elmCfg.proto, prefsElm.getString("proto", ELM_PROTO).c_str(), sizeof(elmCfg.proto));
}

WiFiClient elm;

// Отправить команду и дождаться приглашения '>'. Ответ — строки через '\n'.
bool elmCommand(const char *cmd, char *buf, size_t n, uint32_t tmo = 3000) {
  while (elm.available()) elm.read();
  elm.print(cmd); elm.print('\r');
  size_t k = 0;
  uint32_t t0 = millis();
  bool ok = false;
  while (millis() - t0 < tmo) {
    if (!elm.connected()) break;
    int c = elm.read();
    if (c < 0) { vTaskDelay(1); continue; }
    if (c == '>') { ok = true; break; }
    if (c == 0) continue;                        // некоторые клоны шлют нули
    if (c == '\r') c = '\n';
    if (k + 1 < n) buf[k++] = (char)c;
  }
  buf[k] = 0;
  return ok;
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Разобрать строку «7E8 10 0C 61 02 …»: заголовок (3 знака) и байты. false — строка не из hex.
static bool elmParseLine(const char *l, int &hdr, uint8_t *b, int &nb) {
  hdr = -1; nb = 0;
  while (*l) {
    while (*l == ' ') l++;
    if (!*l) break;
    int len = 0, v = 0;
    while (l[len] && l[len] != ' ') { int h = hexv(l[len]); if (h < 0) return false; v = v * 16 + h; len++; }
    if (len == 3 && hdr < 0 && nb == 0) hdr = v;
    else if (len == 2 && nb < 16) b[nb++] = (uint8_t)v;
    else return false;
    l += len;
  }
  return nb > 0;
}

// Найти в ответе «(svc+0x40) pid …» и вернуть байты после него (их число или -1).
// Как в приложении-терминале: многокадровые ответы собираются, а недособранные тоже
// просматриваются — дешёвый ELM327 у ABS (785) часто получает только первый кадр,
// а скорости колёс лежат уже в нём. Шаблон ищется в любом месте сообщения.
int elmData(char *buf, uint8_t svc, uint8_t pid, uint8_t *d, int dmax) {
  uint8_t msgs[4][64]; int lens[4] = {0, 0, 0, 0}; int nm = 0;
  int cur = -1, total = 0;
  char *save = nullptr;
  for (char *l = strtok_r(buf, "\n", &save); l; l = strtok_r(nullptr, "\n", &save)) {
    int hdr, nb; uint8_t b[16];
    if (!elmParseLine(l, hdr, b, nb)) continue;
    if (hdr < 0) {                               // без заголовка — как есть
      if (nm < 4) { lens[nm] = std::min(nb, 64); memcpy(msgs[nm], b, lens[nm]); nm++; }
      continue;
    }
    uint8_t pci = b[0];
    if ((pci >> 4) == 0) {
      if (nm < 4) { int len = std::min((int)(pci & 15), nb - 1); lens[nm] = len; memcpy(msgs[nm], b + 1, len); nm++; }
      cur = -1;
    } else if ((pci >> 4) == 1 && nb >= 2) {
      if (nm < 4) { cur = nm++; total = std::min(64, ((pci & 15) << 8) | b[1]); lens[cur] = 0;
        for (int i = 2; i < nb && lens[cur] < total; i++) msgs[cur][lens[cur]++] = b[i]; }
    } else if ((pci >> 4) == 2 && cur >= 0) {
      for (int i = 1; i < nb && lens[cur] < total; i++) msgs[cur][lens[cur]++] = b[i];
    }
  }
  for (int m = 0; m < nm; m++)
    for (int i = 0; i + 1 < lens[m]; i++)
      if (msgs[m][i] == svc + 0x40 && msgs[m][i + 1] == pid) {
        int n = std::min(lens[m] - i - 2, dmax);
        memcpy(d, msgs[m] + i + 2, n);
        return n;
      }
  return -1;
}

static char elmBuf[1024];
static char elmHeader[8] = "";

void elmSetHeader(const char *h) {
  if (strcmp(elmHeader, h) == 0) return;
  char c[16]; snprintf(c, sizeof(c), "ATSH%s", h);
  elmCommand(c, elmBuf, sizeof(elmBuf));
  strlcpy(elmHeader, h, sizeof(elmHeader));
}

// Запрос режима 01 к двигателю; true — ответ разобран
bool elmPid(uint8_t pid) {
  elmSetHeader("7E0");
  char c[8]; snprintf(c, sizeof(c), "01%02X", pid);
  elmCommand(c, elmBuf, sizeof(elmBuf), 2000);
  uint8_t d[8];
  int n = elmData(elmBuf, 0x01, pid, d, sizeof(d));
  if (n < 1) return false;
  uint32_t now = millis();
  int A = d[0], B = n > 1 ? d[1] : 0;
  switch (pid) {
    case PID_SPEED:   g_speedKmh = A; g_speedAt = now; break;
    case PID_RPM:     g_rpm = (A * 256 + B) / 4; g_rpmAt = now; break;
    case PID_COOLANT: g_coolant = A - 40; g_coolAt = now; break;
    case PID_MAP:     g_mapKpa = A; g_mapAt = now; break;
    case PID_BARO:    g_baroKpa = A; g_baroAt = now; break;
    case PID_IAT:     g_iat = A - 40; g_iatAt = now; break;
  }
  return true;
}

void elmAtf() {
  elmSetHeader("7E1");
  elmCommand("2102", elmBuf, sizeof(elmBuf), 2500);
  uint8_t d[32];
  int n = elmData(elmBuf, 0x21, 0x02, d, sizeof(d));
  if (n > 5) { g_atf = d[5] - 40; g_atfAt = millis(); }   // 61 02 A B C D E F…: F − 40 = ATF
}

// Скорости колёс из ABS: 784 → 785, 21 20: D0 перед-право, D1 перед-лево, D2 зад-право, D3 зад-лево
// (так найдено приложением-терминалом на Pajero Sport 2008; единицы условные, важна только разница)
volatile uint8_t  g_wheel[4] = {0, 0, 0, 0};
volatile uint32_t g_wheelAt = 0;
volatile bool     g_wantWheels = false;      // экран колёс открыт — опрашиваем ABS чаще
bool elmWheels() {
  elmSetHeader("784");
  elmCommand("2120", elmBuf, sizeof(elmBuf), 1500);
  uint8_t d[16];
  int n = elmData(elmBuf, 0x21, 0x20, d, sizeof(d));
  if (n < 4) return false;
  for (int i = 0; i < 4; i++) g_wheel[i] = d[i];
  g_wheelAt = millis();
  return true;
}

void elmVolt() {
  elmCommand("ATRV", elmBuf, sizeof(elmBuf));
  float v = atof(elmBuf);
  if (v > 5 && v < 20) g_volt = v;
}

void elmTerminal() {
  if (!elmTermPending) return;
  char cmd[64]; strlcpy(cmd, elmTermCmd, sizeof(cmd));
  for (char *q = cmd; *q; q++) *q = toupper(*q);
  bool ok = elmCommand(cmd, elmTermOut, sizeof(elmTermOut), 10000);
  if (!ok && !elmTermOut[0]) strlcpy(elmTermOut, "(таймаут)", sizeof(elmTermOut));
  elmHeader[0] = 0;                              // после ручных команд заголовок выставим заново
  elmTermLastAt = millis();
  elmTermSeq = elmTermSeq + 1;
  elmTermPending = false;
}

void elmTask(void *) {
  elmLoadCfg();
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);                  // переподключением управляем сами
  uint32_t lastBegin = 0;
  bool begun = false;
  for (;;) {
    if (g_elmReconnect) {
      g_elmReconnect = false;
      elm.stop(); WiFi.disconnect(); begun = false; lastBegin = 0;
      elmLoadCfg();
    }
    // 1. Wi-Fi сеть адаптера
    if (WiFi.status() != WL_CONNECTED) {
      g_elmState = 0; g_canOk = false;
      // Пока включена своя сеть «Krenometr», адаптер ищем только на его канале (запомнен в NVS):
      // перебор каналов гоняет радио, и магнитола теряет сеть. Канал неизвестен — короткий поиск
      // раз в минуту; известен, но адаптера нет 5 минут — проверка, не сменил ли он канал.
      if (wifiOn) {
        uint32_t scanEvery = elmChan == 0 ? 60000UL : 300000UL;
        if (elmScanAt == 0 || millis() - elmScanAt > scanEvery) {
          elmScanAt = millis();
          if (begun) { WiFi.disconnect(); begun = false; }   // идёт попытка подключения — поиск не даст
          int n = WiFi.scanNetworks(false, false, false, 120);
          for (int i = 0; i < n; i++) {
            if (WiFi.SSID(i) == elmCfg.ssid) {
              if (elmChan != (uint8_t)WiFi.channel(i)) begun = false;   // канал сменился — подключиться сразу
              elmChan = (uint8_t)WiFi.channel(i);
              memcpy(elmBssid, WiFi.BSSID(i), 6); elmBssidOk = true;
              prefsElm.putUChar("ch", elmChan);
              prefsElm.putBytes("bs", elmBssid, 6);
              Serial.printf("ELM327: %s, канал %d\n", elmCfg.ssid, elmChan);
              break;
            }
          }
          WiFi.scanDelete();
        }
        if (elmChan != 0 && (!begun || millis() - lastBegin > 15000)) {
          WiFi.disconnect();
          WiFi.begin(elmCfg.ssid, elmCfg.pass[0] ? elmCfg.pass : nullptr, elmChan, elmBssidOk ? elmBssid : nullptr);
          begun = true; lastBegin = millis();
        }
        if (elmTermPending) { strlcpy(elmTermOut, elmChan ? "(адаптер не отвечает: зажигание выключено?)" : "(сеть адаптера не найдена)", sizeof(elmTermOut)); elmTermSeq = elmTermSeq + 1; elmTermPending = false; }
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      if (!begun || millis() - lastBegin > 15000) {
        WiFi.disconnect();                       // прошлая попытка — остановить поиск
        WiFi.begin(elmCfg.ssid, elmCfg.pass[0] ? elmCfg.pass : nullptr);
        begun = true; lastBegin = millis();
      }
      if (elmTermPending) { strlcpy(elmTermOut, "(нет Wi-Fi адаптера)", sizeof(elmTermOut)); elmTermSeq = elmTermSeq + 1; elmTermPending = false; }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    elmScanAt = millis();                        // сеть адаптера есть — перепроверять канал незачем
    if (WiFi.channel() != elmChan) {             // запомнить канал для режима «сеть всегда включена»
      elmChan = (uint8_t)WiFi.channel();
      memcpy(elmBssid, WiFi.BSSID(), 6); elmBssidOk = true;
      prefsElm.putUChar("ch", elmChan);
      prefsElm.putBytes("bs", elmBssid, 6);
    }
    // 2. TCP и инициализация — как в приложении-терминале
    if (!elm.connected()) {
      g_elmState = 1; g_canOk = false;
      if (!elm.connect(elmCfg.host, elmCfg.port, 3000)) {
        if (elmTermPending) { strlcpy(elmTermOut, "(адаптер не принимает соединение — занят другим устройством?)", sizeof(elmTermOut)); elmTermSeq = elmTermSeq + 1; elmTermPending = false; }
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      elm.setNoDelay(true);
      elmCommand("ATZ", elmBuf, sizeof(elmBuf), 5000);
      vTaskDelay(pdMS_TO_TICKS(300));
      elmCommand("ATE0", elmBuf, sizeof(elmBuf));
      elmCommand("ATL0", elmBuf, sizeof(elmBuf));
      elmCommand("ATS1", elmBuf, sizeof(elmBuf));
      elmCommand("ATH1", elmBuf, sizeof(elmBuf));
      elmCommand("ATAT1", elmBuf, sizeof(elmBuf));
      char c[12]; snprintf(c, sizeof(c), "ATSP%s", elmCfg.proto);
      elmCommand(c, elmBuf, sizeof(elmBuf));
      elmCommand("ATI", elmBuf, sizeof(elmBuf));
      { char *q = elmBuf; while (*q == '\n') q++; strlcpy(g_elmId, q, sizeof(g_elmId)); char *nl = strchr(g_elmId, '\n'); if (nl) *nl = 0; }
      elmCommand("ATCF 700", elmBuf, sizeof(elmBuf));   // ответы с любых 0x7xx
      elmCommand("ATCM 700", elmBuf, sizeof(elmBuf));
      elmHeader[0] = 0;
      g_elmState = 2;
    }
    // 3. Опрос: скорость через раз (она нужна для счисления пути), остальное по очереди
    static const uint8_t seq[8] = {PID_SPEED, PID_MAP, PID_SPEED, PID_RPM, PID_SPEED, PID_MAP, PID_SPEED, 0};
    static uint8_t idx = 0, slowIdx = 0;
    static uint32_t lastOk = 0;
    elmTerminal();
    if (elmTermLastAt && millis() - elmTermLastAt < 60000) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }
    static bool wheelTurn = false;
    uint8_t pid = 0xFF;                          // 0xFF — в этот раз опрашивали ABS
    bool ok = false;
    if (g_wantWheels && (wheelTurn = !wheelTurn)) {
      elmWheels();                               // экран колёс: колёса и скорость по очереди
    } else {
      static uint8_t wcnt = 0;
      pid = g_wantWheels ? (++wcnt % 10 == 0 ? PID_COOLANT : PID_SPEED) : seq[idx];
      if (!g_wantWheels) idx = (idx + 1) % 8;
      if (pid) ok = elmPid(pid);
      else {
        // ОЖ — через раз среди медленных, остальное по очереди
        switch (slowIdx) {
          case 1: ok = elmPid(PID_IAT); break;
          case 3: elmVolt(); ok = true; break;
          case 5: ok = elmPid(PID_BARO); break;
          case 7: elmAtf(); ok = true; break;
          default: ok = elmPid(PID_COOLANT); break;
        }
        slowIdx = (slowIdx + 1) % 8;
      }
    }
    if (ok && pid && pid != 0xFF) lastOk = millis();
    bool alive = lastOk && millis() - lastOk < 3000;
    g_canOk = alive;
    g_elmState = alive ? 3 : 2;
    if (!alive) vTaskDelay(pdMS_TO_TICKS(300));   // зажигание выключено — не долбим адаптер
  }
}

// =====================================================================
//                    ГРАФИКА СО СГЛАЖИВАНИЕМ (iOS-стиль)
// Всё рисуется прямо в кадровый буфер с субпиксельным сглаживанием
// через «поля расстояний» (SDF): круги, дуги, линии, скруглённые прямоугольники.
// =====================================================================
// Весь интерфейс описан в логических координатах 240x240,
// а рисуется в настоящем разрешении PW x PW с коэффициентом SC.
#define SW PW
constexpr float SC = PW / 240.0f;
// Пиксели в буфере хранятся сразу в порядке байтов экрана (старший байт первым),
// чтобы DMA отправлял их без перестановки.
uint16_t *FB = nullptr;            // буфер, в который сейчас рисуется полоса
uint16_t *bandBuf[2] = {nullptr, nullptr};
int g_rot = DISPLAY_ROT;           // поворот картинки (0..3 × 90° по часовой)
uint16_t *rotFB = nullptr;         // полный кадр в PSRAM для поворота
int bandRows = 0;                  // высота полосы (строк)
int bandY0 = 0, bandY1 = 0;        // текущая полоса: строки [bandY0, bandY1)

static inline uint16_t sw16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

static inline void blendPx(int x, int y, uint16_t c, uint32_t a) {   // a: 0..255
  if ((unsigned)x >= SW || y < bandY0 || y >= bandY1 || a == 0) return;
  uint16_t &d = FB[(y - bandY0) * SW + x];
  if (a >= 255) { d = sw16(c); return; }
  uint16_t o = sw16(d);
  int r1 = o >> 11, g1 = (o >> 5) & 63, b1 = o & 31;
  int r2 = c >> 11, g2 = (c >> 5) & 63, b2 = c & 31;
  int ai = (int)a;
  r1 += ((r2 - r1) * ai) >> 8;
  g1 += ((g2 - g1) * ai) >> 8;
  b1 += ((b2 - b1) * ai) >> 8;
  d = sw16((uint16_t)((r1 << 11) | (g1 << 5) | b1));
}

// ===================== Быстрая отправка на экран (DMA) =====================
spi_device_handle_t lcdDev = nullptr;
#define LCD_CHUNK_ROWS 16                       // строк в одной DMA-передаче (не больше 16 КБ)
spi_transaction_t lcdTr[8];
int lcdInflight = 0, lcdInflightBuf = -1;
uint32_t tRenderUs = 0, tSendUs = 0, pxSent = 0;

static void lcdCmd(uint8_t reg, const uint8_t *data, int n) {
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
  t.cmd = 0x02;
  t.addr = ((uint32_t)reg) << 8;
  if (n > 0 && n <= 4) { t.flags |= SPI_TRANS_USE_TXDATA; memcpy(t.tx_data, data, n); }
  else if (n > 0) t.tx_buffer = data;
  t.length = n * 8;
  spi_device_polling_transmit(lcdDev, &t);
}

static void lcdWait() {                         // дождаться окончания передачи предыдущей полосы
  spi_transaction_t *rt;
  while (lcdInflight > 0) { spi_device_get_trans_result(lcdDev, &rt, portMAX_DELAY); lcdInflight--; }
  lcdInflightBuf = -1;
}

bool lcdFastInit() {
  spi_device_interface_config_t dc;
  memset(&dc, 0, sizeof(dc));
  dc.command_bits = 8;
  dc.address_bits = 24;
  dc.mode = 0;
  dc.clock_source = SPI_CLK_SRC_DEFAULT;
  dc.clock_speed_hz = QSPI_HZ;
  dc.spics_io_num = PIN_LCD_CS;                  // выбор кристалла — аппаратно на каждую передачу
  dc.flags = SPI_DEVICE_HALFDUPLEX;
  dc.queue_size = 8;
  return spi_bus_add_device(SPI2_HOST, &dc, &lcdDev) == ESP_OK;
}

void lcdSetBrightness(uint8_t b) {
  if (lcdDev) { lcdWait(); lcdCmd(0x51, &b, 1); }
  else panel->setBrightness(b);
}

// Отправить полосу из буфера bi: окно по строкам, затем пиксели кусками через DMA (без ожидания)
static void lcdSendBand(int bi, int y0, int rows) {
  lcdWait();
  uint8_t ca[4] = { 0, 0, 0, 0 }, ra[4];
  uint16_t x0 = PANEL_CO5300 ? PANEL_COL_OFFSET : 0, x1 = x0 + SW - 1;
  ca[0] = x0 >> 8; ca[1] = x0 & 0xFF; ca[2] = x1 >> 8; ca[3] = x1 & 0xFF;
  ra[0] = y0 >> 8; ra[1] = y0 & 0xFF; ra[2] = (y0 + rows - 1) >> 8; ra[3] = (y0 + rows - 1) & 0xFF;
  lcdCmd(0x2A, ca, 4);
  lcdCmd(0x2B, ra, 4);
  lcdCmd(0x2C, nullptr, 0);
  int n = 0;
  for (int r = 0; r < rows && n < 8; r += LCD_CHUNK_ROWS, n++) {
    int h = std::min(LCD_CHUNK_ROWS, rows - r);
    spi_transaction_t &t = lcdTr[n];
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_MODE_QIO;
    t.cmd = 0x32;
    t.addr = 0x003C00;                          // «продолжить запись в память»
    t.tx_buffer = bandBuf[bi] + r * SW;
    t.length = (uint32_t)SW * h * 16;
    spi_device_queue_trans(lcdDev, &t, portMAX_DELAY);
  }
  lcdInflight = n;
  lcdInflightBuf = bi;
}

// Закрасить область по функции расстояния: d < 0 внутри, 0 — край
template <typename SDF>
void shade(float fx0, float fy0, float fx1, float fy1, uint16_t c, uint8_t alpha, SDF sd) {
  int x0 = std::max(0, (int)floorf(fx0) - 1), y0 = std::max(bandY0, (int)floorf(fy0) - 1);
  int x1 = std::min(SW - 1, (int)ceilf(fx1) + 1), y1 = std::min(bandY1 - 1, (int)ceilf(fy1) + 1);
  if (y0 > y1 || x0 > x1) return;                    // фигура вне текущей полосы
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++) {
      float d = sd((float)x, (float)y);
      if (d >= 0.5f) continue;
      float cov = d <= -0.5f ? 1.0f : 0.5f - d;
      blendPx(x, y, c, (uint32_t)(cov * alpha));
    }
}

// То же, но для каждой строки задаются отрезки по x, где фигура может быть (до двух отрезков).
// span(y, xs) пишет пары [xs0,xs1], [xs2,xs3] и возвращает число пар.
template <typename SPAN, typename SDF>
void shadeRows(float fy0, float fy1, uint16_t c, uint8_t alpha, SPAN span, SDF sd) {
  int y0 = std::max(bandY0, (int)floorf(fy0) - 1), y1 = std::min(bandY1 - 1, (int)ceilf(fy1) + 1);
  float xs[4];
  for (int y = y0; y <= y1; y++) {
    int n = span((float)y, xs);
    for (int i = 0; i < n; i++) {
      int xa = std::max(0, (int)floorf(xs[2 * i]) - 1), xb = std::min(SW - 1, (int)ceilf(xs[2 * i + 1]) + 1);
      for (int x = xa; x <= xb; x++) {
        float d = sd((float)x, (float)y);
        if (d >= 0.5f) continue;
        float cov = d <= -0.5f ? 1.0f : 0.5f - d;
        blendPx(x, y, c, (uint32_t)(cov * alpha));
      }
    }
  }
}

// Отрезок строки y внутри кольца с радиусами rIn..rOut (квадраты), с обрезкой по [bx0, bx1]
static inline int annulusSpan(float y, float cx, float cy, float in2, float out2, float bx0, float bx1, float *xs) {
  float dy = y - cy, ao2 = out2 - dy * dy;
  if (ao2 <= 0) return 0;
  float ao = sqrtf(ao2), ai2 = in2 - dy * dy;
  int n = 0;
  auto add = [&](float a, float b) {
    a = std::max(a, bx0); b = std::min(b, bx1);
    if (a <= b) { xs[2 * n] = a; xs[2 * n + 1] = b; n++; }
  };
  if (ai2 <= 0) add(cx - ao, cx + ao);
  else { float ai = sqrtf(ai2); add(cx - ao, cx - ai); add(cx + ai, cx + ao); }
  return n;
}

// Для выпуклой фигуры с рёбрами nx*x + ny*y + nc <= t: отрезок по x в строке y
static inline bool convexSpan(const float *nx, const float *ny, const float *nc, int m, float y, float t, float &lo, float &hi) {
  lo = -1e9f; hi = 1e9f;
  for (int i = 0; i < m; i++) {
    float rhs = t - ny[i] * y - nc[i];
    if (nx[i] > 1e-6f) hi = std::min(hi, rhs / nx[i]);
    else if (nx[i] < -1e-6f) lo = std::max(lo, rhs / nx[i]);
    else if (rhs < 0) return false;
  }
  return lo <= hi;
}

void fillCircleAA(float cx, float cy, float r, uint16_t c, uint8_t alpha = 255) {
  cx *= SC; cy *= SC; r *= SC;
  float rin = std::max(0.0f, r - 0.5f), rout = r + 0.5f;
  float in2 = rin * rin, out2 = rout * rout;
  float span2 = (r + 1) * (r + 1);
  shadeRows(cy - r, cy + r, c, alpha, [&](float y, float *xs) {
    return annulusSpan(y, cx, cy, 0, span2, -1e9f, 1e9f, xs);
  }, [&](float x, float y) {
    float d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
    if (d2 <= in2) return -1.0f;                     // внутри — корень не нужен
    if (d2 >= out2) return 1.0f;
    return sqrtf(d2) - r;
  });
}

void ringAA(float cx, float cy, float r, float w, uint16_t c, uint8_t alpha = 255) {
  cx *= SC; cy *= SC; r *= SC; w *= SC;
  float R0 = r + w / 2;
  float h = w / 2;
  float in2 = std::max(0.0f, r - h - 1), out2 = r + h + 1;
  in2 *= in2; out2 *= out2;
  float fi = std::max(0.0f, r - h + 0.5f), fo = std::max(0.0f, r + h - 0.5f);
  float fin2 = fi * fi, fout2 = fo * fo;
  shadeRows(cy - R0, cy + R0, c, alpha, [&](float y, float *xs) {
    return annulusSpan(y, cx, cy, in2, out2, -1e9f, 1e9f, xs);
  }, [&](float x, float y) {
    float d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
    if (d2 > out2 || d2 < in2) return 1.0f;          // быстро отбрасываем пиксели вне кольца
    if (d2 >= fin2 && d2 <= fout2) return -1.0f;     // целиком внутри кольца
    return fabsf(sqrtf(d2) - r) - h;
  });
}

// Линия-«капсула» со скруглёнными концами
void lineAA(float x0, float y0, float x1, float y1, float w, uint16_t c, uint8_t alpha = 255) {
  x0 *= SC; y0 *= SC; x1 *= SC; y1 *= SC; w *= SC;
  float dx = x1 - x0, dy = y1 - y0, L2 = dx * dx + dy * dy;
  float h = w / 2;
  float hin2 = std::max(0.0f, h - 0.5f); hin2 *= hin2;
  float hout2 = (h + 0.5f) * (h + 0.5f);
  shade(std::min(x0, x1) - h, std::min(y0, y1) - h, std::max(x0, x1) + h, std::max(y0, y1) + h, c, alpha,
        [&](float x, float y) {
          float t = L2 > 0 ? ((x - x0) * dx + (y - y0) * dy) / L2 : 0;
          t = t < 0 ? 0 : (t > 1 ? 1 : t);
          float ex = x - (x0 + t * dx), ey = y - (y0 + t * dy);
          float e2 = ex * ex + ey * ey;
          if (e2 <= hin2) return -1.0f;
          if (e2 >= hout2) return 1.0f;
          return sqrtf(e2) - h;
        });
}

// Дуга со скруглёнными концами. Углы в радианах, 0 — вправо, по часовой.
void arcAA(float cx, float cy, float r, float w, float a0, float a1, uint16_t c, uint8_t alpha = 255) {
  if (a1 < a0) { float t = a0; a0 = a1; a1 = t; }
  float span = a1 - a0;
  if (span < 0.001f) {
    float px = cx + r * cosf(a0), py = cy + r * sinf(a0);
    fillCircleAA(px, py, w / 2, c, alpha);
    return;
  }
  cx *= SC; cy *= SC; r *= SC; w *= SC;
  // границы: концы дуги и попавшие внутрь «полюса»
  float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
  auto addPt = [&](float a) {
    float px = cx + r * cosf(a), py = cy + r * sinf(a);
    bx0 = std::min(bx0, px); by0 = std::min(by0, py); bx1 = std::max(bx1, px); by1 = std::max(by1, py);
  };
  addPt(a0); addPt(a1);
  for (int k = -4; k <= 8; k++) { float a = k * PI / 2; if (a > a0 && a < a1) addPt(a); }
  float h = w / 2;
  float ex0 = cx + r * cosf(a0), ey0 = cy + r * sinf(a0);
  float ex1 = cx + r * cosf(a1), ey1 = cy + r * sinf(a1);
  float in2 = std::max(0.0f, r - h - 1), out2 = r + h + 1;
  in2 *= in2; out2 *= out2;
  float fi = std::max(0.0f, r - h + 0.5f), fo = std::max(0.0f, r + h - 0.5f);
  float fin2 = fi * fi, fout2 = fo * fo;
  // направление на начало и конец дуги: попадание в сектор — по векторным произведениям, без atan2
  float u0x = cosf(a0), u0y = sinf(a0), u1x = cosf(a1), u1y = sinf(a1);
  bool full = span >= 2 * PI - 1e-4f, small = span <= PI;
  float sx0 = bx0 - h - 1, sx1 = bx1 + h + 1;
  shadeRows(by0 - h, by1 + h, c, alpha, [&](float y, float *xs) {
    return annulusSpan(y, cx, cy, in2, out2, sx0, sx1, xs);
  }, [&](float x, float y) {
    float dx = x - cx, dy = y - cy;
    float d2 = dx * dx + dy * dy;
    if (d2 > out2 || d2 < in2) return 1.0f;          // вне кольца — ни дуги, ни её концов
    bool inSpan;
    if (full) inSpan = true;
    else {
      float c0 = u0x * dy - u0y * dx;                // > 0: точка «после» начала
      float c1 = dx * u1y - dy * u1x;                // > 0: точка «до» конца
      inSpan = small ? (c0 >= 0 && c1 >= 0) : !(c0 < 0 && c1 < 0);
    }
    if (inSpan) {
      if (d2 >= fin2 && d2 <= fout2) return -1.0f;
      return fabsf(sqrtf(d2) - r) - h;
    }
    float q0 = (x - ex0) * (x - ex0) + (y - ey0) * (y - ey0);
    float q1 = (x - ex1) * (x - ex1) + (y - ey1) * (y - ey1);
    float q = std::min(q0, q1);
    if (q >= (h + 0.5f) * (h + 0.5f)) return 1.0f;
    return sqrtf(q) - h;
  });
}

// Скруглённый прямоугольник, повёрнутый на угол ang (радианы) вокруг своего центра
void rrectAA(float cx, float cy, float hw, float hh, float rad, float ang, uint16_t c, uint8_t alpha = 255) {
  cx *= SC; cy *= SC; hw *= SC; hh *= SC; rad *= SC;
  float cs = cosf(ang), sn = sinf(ang);
  float ext = sqrtf(hw * hw + hh * hh) + 1;
  // рёбра повёрнутого прямоугольника: |dx*cs+dy*sn| <= hw, |-dx*sn+dy*cs| <= hh
  float nx[4] = { cs, -cs, -sn, sn }, ny[4] = { sn, -sn, cs, -cs };
  float nc[4] = { -(cs * cx + sn * cy) - hw,  (cs * cx + sn * cy) - hw,
                  -(-sn * cx + cs * cy) - hh, (-sn * cx + cs * cy) - hh };
  float ilo = 0, ihi = -1;
  float inner = -(rad + 0.5f);                       // глубже радиуса скругления — точно внутри
  shadeRows(cy - ext, cy + ext, c, alpha, [&](float y, float *xs) {
    float lo, hi;
    if (!convexSpan(nx, ny, nc, 4, y, 0.5f, lo, hi)) return 0;
    if (!convexSpan(nx, ny, nc, 4, y, inner, ilo, ihi)) { ilo = 0; ihi = -1; }
    xs[0] = lo; xs[1] = hi; return 1;
  }, [&](float x, float y) {
    if (x >= ilo && x <= ihi) return -1.0f;
    float dx = x - cx, dy = y - cy;
    float lx = fabsf(dx * cs + dy * sn) - (hw - rad);
    float ly = fabsf(-dx * sn + dy * cs) - (hh - rad);
    if (lx <= 0 && ly <= 0) return std::max(lx, ly) - rad;   // не в углу — корень не нужен
    float ox = std::max(lx, 0.0f), oy = std::max(ly, 0.0f);
    return sqrtf(ox * ox + oy * oy) + std::min(std::max(lx, ly), 0.0f) - rad;
  });
}

// Выпуклый многоугольник (вершины по часовой на экране)
void polyAA(const float *pxIn, const float *pyIn, int n, uint16_t c, uint8_t alpha = 255) {
  float px[12], py[12];
  if (n > 12) n = 12;
  for (int i = 0; i < n; i++) { px[i] = pxIn[i] * SC; py[i] = pyIn[i] * SC; }
  float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
  for (int i = 0; i < n; i++) { bx0 = std::min(bx0, px[i]); by0 = std::min(by0, py[i]); bx1 = std::max(bx1, px[i]); by1 = std::max(by1, py[i]); }
  float area = 0;
  for (int i = 0; i < n; i++) { int j = (i + 1) % n; area += px[i] * py[j] - px[j] * py[i]; }
  float sgn = area > 0 ? 1.0f : -1.0f;
  // заранее: нормали рёбер, чтобы на каждый пиксель не считать корни
  float nx[12], ny[12], nc[12];
  int m = 0;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    float ex = px[j] - px[i], ey = py[j] - py[i];
    float L = sqrtf(ex * ex + ey * ey);
    if (L < 1e-4f) continue;
    nx[m] = sgn * ey / L; ny[m] = -sgn * ex / L;
    nc[m] = -(nx[m] * px[i] + ny[m] * py[i]);
    m++;
  }
  float ilo = 0, ihi = -1;
  shadeRows(by0, by1, c, alpha, [&](float y, float *xs) {
    float lo, hi;
    if (!convexSpan(nx, ny, nc, m, y, 0.5f, lo, hi)) return 0;
    if (!convexSpan(nx, ny, nc, m, y, -0.5f, ilo, ihi)) { ilo = 0; ihi = -1; }
    xs[0] = lo; xs[1] = hi; return 1;
  }, [&](float x, float y) {
    if (x >= ilo && x <= ihi) return -1.0f;
    float d = -1e9f;
    for (int i = 0; i < m; i++) d = std::max(d, nx[i] * x + ny[i] * y + nc[i]);
    return d;
  });
}

// ---- Текст ----
static const AAGlyph *findGlyph(const AAFont &f, uint16_t cp) {
  int lo = 0, hi = f.n - 1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    uint16_t c = f.g[mid].code;
    if (c == cp) return &f.g[mid];
    if (c < cp) lo = mid + 1; else hi = mid - 1;
  }
  return nullptr;
}

static uint16_t utf8Next(const char *&s) {
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) return c;
  if ((c & 0xE0) == 0xC0) { uint16_t r = (c & 0x1F) << 6; r |= (*s++ & 0x3F); return r; }
  if ((c & 0xF0) == 0xE0) { uint16_t r = (c & 0x0F) << 12; r |= (*s++ & 0x3F) << 6; r |= (*s++ & 0x3F); return r; }
  return '?';
}

int textWidth(const AAFont &f, const char *s) {
  int w = 0;
  while (*s) { const AAGlyph *g = findGlyph(f, utf8Next(s)); if (g) w += g->adv; }
  return w;
}

enum { AL_LEFT, AL_CENTER, AL_RIGHT };

// y — базовая линия
void text(float fx, float y, const char *s, const AAFont &f, uint16_t c, int align = AL_CENTER, uint8_t alpha = 255) {
  int x = (int)lroundf(fx * SC);
  y *= SC;
  if (align == AL_CENTER) x -= textWidth(f, s) / 2;
  else if (align == AL_RIGHT) x -= textWidth(f, s);
  int by = (int)lroundf(y);
  while (*s) {
    const AAGlyph *g = findGlyph(f, utf8Next(s));
    if (!g) continue;
    const uint8_t *p = f.bmp + g->off;
    int stride = (g->w + 1) / 2;
    int gy = by + g->yo;
    int rA = std::max(0, bandY0 - gy), rB = std::min((int)g->h, bandY1 - gy);   // только строки текущей полосы
    for (int r = rA; r < rB; r++)
      for (int col = 0; col < g->w; col++) {
        uint8_t b = p[r * stride + col / 2];
        uint8_t v = (col & 1) ? (b & 0x0F) : (b >> 4);
        if (v) blendPx(x + g->xo + col, by + g->yo + r, c, (uint32_t)v * 17 * alpha / 255);
      }
    x += g->adv;
  }
}

// число в стиле iOS: запятая вместо точки, длинный минус
void fmtNum(char *out, size_t n, float v, int dec, const char *suffix = "") {
  char tmp[32];
  snprintf(tmp, sizeof(tmp), "%.*f", dec, fabsf(v));
  for (char *p = tmp; *p; p++) if (*p == '.') *p = ',';
  bool neg = v < 0 && (int)lroundf(fabsf(v) * powf(10, dec)) != 0;
  snprintf(out, n, "%s%s%s", neg ? "\xE2\x88\x92" : "", tmp, suffix);
}

// ===================== Элементы интерфейса =====================
uint16_t zone(float v, float warn, float danger) {
  float a = fabsf(v);
  if (a >= danger) return IOS_RED;
  if (a >= warn) return IOS_YELLOW;
  return IOS_GREEN;
}

// ---- Силуэты: тёмно-синий Pajero Sport (KH) с боксом на крыше ----
constexpr uint16_t CAR_BLUE   = rgb(60, 88, 126);    // синий металлик
constexpr uint16_t CAR_DARK   = rgb(38, 56, 82);
constexpr uint16_t CAR_LIGHT  = rgb(104, 136, 176);  // блик
constexpr uint16_t CAR_GLASS  = rgb(16, 20, 28);
constexpr uint16_t CAR_TIRE   = rgb(50, 50, 54);
constexpr uint16_t CAR_RIM    = rgb(176, 178, 184);  // литые диски
constexpr uint16_t CAR_RIM2   = rgb(118, 120, 126);
constexpr uint16_t CAR_TRIM   = rgb(30, 30, 34);     // чёрный пластик
constexpr uint16_t CAR_BOX    = rgb(60, 60, 64);     // бокс на крыше
constexpr uint16_t CAR_BOX_HL = rgb(96, 96, 102);
constexpr uint16_t CAR_CHROME = rgb(200, 204, 212);
constexpr uint16_t CAR_TAIL   = rgb(220, 50, 42);
constexpr uint16_t CAR_AMBER  = rgb(255, 150, 40);

// Преобразование локальных координат силуэта: масштаб k, поворот a, центр (cx, cy)
struct CarXf {
  float cx, cy, cs, sn, k, a;
  CarXf(float x, float y, float ang, float scale) : cx(x), cy(y), cs(cosf(ang) * scale), sn(sinf(ang) * scale), k(scale), a(ang) {}
  void p(float lx, float ly, float &ox, float &oy) const { ox = cx + lx * cs - ly * sn; oy = cy + lx * sn + ly * cs; }
  void poly(std::initializer_list<float> xy, uint16_t c) const {
    float px[10], py[10];
    int n = 0;
    const float *v = xy.begin();
    for (size_t i = 0; i + 1 < xy.size() && n < 10; i += 2, n++) p(v[i], v[i + 1], px[n], py[n]);
    polyAA(px, py, n, c);
  }
  void rr(float lx, float ly, float hw, float hh, float r, uint16_t c) const {
    float x, y; p(lx, ly, x, y); rrectAA(x, y, hw * k, hh * k, r * k, a, c);
  }
  void circ(float lx, float ly, float r, uint16_t c) const {
    float x, y; p(lx, ly, x, y); fillCircleAA(x, y, r * k, c);
  }
  void line(float x0, float y0, float x1, float y1, float w, uint16_t c) const {
    float ax, ay, bx, by; p(x0, y0, ax, ay); p(x1, y1, bx, by); lineAA(ax, ay, bx, by, w * k, c);
  }
  void arc(float lx, float ly, float r, float w, float d0, float d1, uint16_t c) const {
    float x, y; p(lx, ly, x, y); arcAA(x, y, r * k, w * k, d0 * DEG + a, d1 * DEG + a, c);
  }
};

// Вид сзади: положительный крен — правый борт ниже (по часовой)
void carRear(float cx, float cy, float rollDeg, float scale = 0.85f) {
  CarXf t(cx, cy, rollDeg * DEG, scale);
  // колёса, запаска под днищем
  t.rr(-25, 16, 7.5f, 9, 3, CAR_TIRE);
  t.rr( 25, 16, 7.5f, 9, 3, CAR_TIRE);
  t.rr(0, 17, 12, 2.5f, 1.5f, CAR_TIRE);
  // расширители арок
  t.rr(-35, 8, 3, 6.5f, 2, CAR_TRIM);
  t.rr( 35, 8, 3, 6.5f, 2, CAR_TRIM);
  // зеркала
  t.rr(-38, -13, 3.5f, 2.5f, 1.5f, CAR_BLUE);
  t.rr( 38, -13, 3.5f, 2.5f, 1.5f, CAR_BLUE);
  // крыша со стойками и низ кузова
  t.poly({-31, -4, 31, -4, 27, -24, 23, -28, -23, -28, -27, -24}, CAR_BLUE);
  t.poly({-35, -6, 35, -6, 36, 9, 34, 14, -34, 14, -36, 9}, CAR_BLUE);
  // бокс на крыше и поперечины
  t.line(-22, -31, -22, -28, 2, CAR_TRIM);
  t.line( 22, -31, 22, -28, 2, CAR_TRIM);
  t.rr(0, -34, 25, 3.8f, 3.5f, CAR_BOX);
  t.line(-21, -36.5f, 21, -36.5f, 1, CAR_BOX_HL);
  // заднее стекло, дворник
  t.poly({-24, -12, 24, -12, 21, -24, -21, -24}, CAR_GLASS);
  t.line(0, -13, -9, -16, 1, CAR_TRIM);
  // большие угловые фонари
  t.poly({-36, -6, -21, -6, -22, -1, -36, 1}, CAR_TAIL);
  t.poly({21, -6, 36, -6, 36, 1, 22, -1}, CAR_TAIL);
  t.rr(-31, -1.5f, 3, 1.2f, 0.6f, CAR_AMBER);
  t.rr( 31, -1.5f, 3, 1.2f, 0.6f, CAR_AMBER);
  // номер, нижняя часть бампера, отражатели
  t.rr(0, 4, 9, 3.5f, 1, rgb(222, 222, 228));
  t.rr(0, 12, 34, 3, 1.5f, CAR_TRIM);
  t.rr(-26, 11.5f, 3, 1.2f, 0.6f, CAR_TAIL);
  t.rr( 26, 11.5f, 3, 1.2f, 0.6f, CAR_TAIL);
}

// Вид сбоку, мордой вправо: нос вверх — против часовой
void carSide(float cx, float cy, float pitchDeg, float scale = 0.82f) {
  CarXf t(cx, cy, -pitchDeg * DEG, scale);
  // бокс на крыше и поперечины
  t.line(-30, -31, -30, -28, 2, CAR_TRIM);
  t.line(4, -31, 4, -28, 2, CAR_TRIM);
  t.poly({-38, -31, -36, -37, -20, -39.5f, 8, -38.5f, 15, -33, 13, -31}, CAR_BOX);
  t.line(-33, -37.5f, 6, -38, 1, CAR_BOX_HL);
  // рейлинг
  t.line(-41, -29.5f, 12, -29.5f, 1.8f, CAR_TRIM);
  // салон и кузов
  t.poly({-49, -7, -45, -24, -39, -28, 14, -28, 33, -9}, CAR_BLUE);
  t.poly({-50, -7, 38, -9, 48, -3, 50, 6, -50, 7}, CAR_BLUE);
  // остекление: одна тонированная полоса + чёрные стойки
  t.poly({-45, -11, -42, -24, -37, -25.5f, 12, -25.5f, 28, -11}, CAR_GLASS);
  t.line(-23, -25, -23, -11, 2.2f, CAR_TRIM);
  t.line(-1.5f, -25, -1.5f, -11, 2.2f, CAR_TRIM);
  t.line(-22, -26.5f, 12, -26.5f, 1, CAR_TRIM);          // дефлекторы окон
  // зеркало
  t.poly({26, -12, 31, -12, 31, -7, 27, -7}, CAR_BLUE);
  // ручки, нижняя линия дверей
  t.line(-17, -6, -13, -6, 1.3f, CAR_CHROME);
  t.line(5, -6, 9, -6, 1.3f, CAR_CHROME);
  t.line(-24, 2, 31, 1.5f, 1, CAR_DARK);
  t.line(-1, -9, -1, 6, 0.8f, CAR_DARK);                 // шов между дверями
  // фара и фонарь
  t.poly({44, -6, 49, -2, 49, 0, 45, -1}, rgb(230, 236, 246));
  t.poly({-50, -7, -44, -8, -44, -2, -50, -1}, CAR_TAIL);
  // колёсные арки, расширители, брызговики, колёса
  for (float wx : {-29.0f, 30.0f}) {
    t.circ(wx, 11, 12.5f, IOS_BG);
    t.arc(wx, 11, 12.8f, 3, 188, 352, CAR_TRIM);
    t.rr(wx - 12.5f, 14, 1.4f, 5, 0.7f, CAR_TRIM);
    t.circ(wx, 11, 10.5f, CAR_TIRE);
    t.circ(wx, 11, 6.8f, CAR_RIM);
    t.circ(wx, 11, 4.4f, CAR_RIM2);
    t.circ(wx, 11, 1.8f, CAR_TRIM);
  }
}

// Шкала-«кольцо активности» по дуге: от 0 до значения, ±SCALE_MAX на ±62°
void tiltArc(float centerDeg, float dir, float value, float warn, float danger) {
  const float R = 101, W = 7, SPAN = 62;
  arcAA(120, 120, R, W, (centerDeg - SPAN) * DEG, (centerDeg + SPAN) * DEG, IOS_TRACK);
  float v = constrain(value, -(float)SCALE_MAX, (float)SCALE_MAX);
  float a0 = centerDeg * DEG, a1 = (centerDeg + dir * v * SPAN / SCALE_MAX) * DEG;
  uint16_t c = zone(value, warn, danger);
  arcAA(120, 120, R, W, std::min(a0, a1), std::max(a0, a1), c);
  // засечки границ зон
  for (float z : {warn, danger}) for (float s : {-1.0f, 1.0f}) {
    float a = (centerDeg + dir * s * z * SPAN / SCALE_MAX) * DEG;
    fillCircleAA(120 + R * cosf(a), 120 + R * sinf(a), 1.3f, IOS_BG);
  }
  // бегунок
  fillCircleAA(120 + R * cosf(a1), 120 + R * sinf(a1), 6.5f, IOS_WHITE);
}

// Всплывающее уведомление (как в iOS)
char toastText[40] = "";
uint32_t toastAt = 0;
void toast(const char *s) { strncpy(toastText, s, sizeof(toastText) - 1); toastAt = millis(); }
void drawToast() {
  if (!toastText[0] || millis() - toastAt > 1600) return;
  uint32_t age = millis() - toastAt;
  uint8_t al = age > 1300 ? (uint8_t)(255 - (age - 1300) * 255 / 300) : 255;
  int w = (int)(textWidth(F_SMALL, toastText) / SC) + 28;
  rrectAA(120, 120, w / 2.0f, 17, 17, 0, IOS_CARD, al);
  text(120, 125, toastText, F_SMALL, IOS_WHITE, AL_CENTER, al);
}

// Кольцо удержания
void holdRing(float p) {
  arcAA(120, 120, 116, 4, -PI / 2, -PI / 2 + 2 * PI * p, IOS_BLUE);
}

// Точки страниц
bool wifiOn = false;
void pageDots(int page, int n) {
  float x0 = 120 - (n - 1) * 4.5f;
  for (int i = 0; i < n; i++) {
    uint16_t c = i == page ? (wifiOn ? IOS_BLUE : IOS_WHITE) : (wifiOn ? rgb(10, 60, 120) : IOS_GRAY4);
    fillCircleAA(x0 + i * 9, 233, 2.6f, c);
  }
}

// ===================== Экраны =====================
float maxRoll = 0, maxPitch = 0;

// Кренометр в стиле автомобильных инклинометров: слева шкала крена, справа — дифферента,
// сверху машина сзади, снизу — сбоку, посередине красная линия горизонта.
constexpr uint16_t TL_AMBER = rgb(245, 166, 35);
constexpr uint16_t TL_HOT   = rgb(240, 90, 40);     // 30–40°
constexpr uint16_t TL_RED   = rgb(214, 40, 40);
constexpr uint16_t TL_BLUE  = rgb(30, 140, 230);
constexpr float    TL_K     = 1.25f;                // градус угла = 1,25° дуги

// side = -1 — левая шкала, +1 — правая; value > 0 рисуется вниз (левая) / вверх (правая)
static float tlAng(int side, float v) { return side < 0 ? (180 - v * TL_K) * DEG : (-v * TL_K) * DEG; }

void tiltScale(int side) {
  const float RT = 90;                              // внешний край рисок
  char b[8];
  for (int v = -44; v <= 44; v += 2) {
    if (v == 0) continue;
    float a = tlAng(side, (float)v), ca = cosf(a), sa = sinf(a);
    int av = abs(v);
    bool major = av % 10 == 0;
    uint16_t c = av >= 30 ? TL_HOT : IOS_WHITE;
    float r0 = major ? RT - 9 : (av % 10 == 5 ? RT - 6 : RT - 4);
    if (av > 40) continue;
    lineAA(120 + r0 * ca, 120 + r0 * sa, 120 + RT * ca, 120 + RT * sa, major ? 1.6f : 1.0f, c);
    if (major) {
      snprintf(b, sizeof(b), "%d", av);
      text(120 + (RT + 12) * ca, 120 + (RT + 12) * sa + 4.5f, b, F_SMALL, c);
    }
  }
  // концевые «уголки» оранжевой зоны 30–40
  for (int sgn : {-1, 1}) {
    float a0 = tlAng(side, sgn * 30.0f), a1 = tlAng(side, sgn * 40.0f);
    arcAA(120, 120, RT, 1.4f, std::min(a0, a1), std::max(a0, a1), TL_HOT);
    float ca = cosf(a1), sa = sinf(a1);
    lineAA(120 + (RT - 10) * ca, 120 + (RT - 10) * sa, 120 + RT * ca, 120 + RT * sa, 1.6f, TL_HOT);
  }
}

void tiltPointer(int side, float v, uint16_t c) {
  float a = tlAng(side, constrain(v, -45.0f, 45.0f)), ca = cosf(a), sa = sinf(a);
  const float tip = 77, base = 66, hw = 6;
  float px[3] = { 120 + tip * ca, 120 + base * ca - hw * sa, 120 + base * ca + hw * sa };
  float py[3] = { 120 + tip * sa, 120 + base * sa + hw * ca, 120 + base * sa - hw * ca };
  polyAA(px, py, 3, c);
}

void pageTilt(float roll, float pitch) {
  tiltScale(-1);
  tiltScale(1);

  // горизонт: красная линия, синие метки и «−» / «+»
  lineAA(36, 120, 204, 120, 1.3f, TL_RED);
  { float px[3] = {37, 31, 31}, py[3] = {120, 116.5f, 123.5f}; polyAA(px, py, 3, TL_BLUE); }
  { float px[3] = {203, 209, 209}, py[3] = {120, 116.5f, 123.5f}; polyAA(px, py, 3, TL_BLUE); }
  lineAA(15, 120, 23, 120, 2.2f, IOS_WHITE);
  lineAA(217, 120, 225, 120, 2.2f, IOS_WHITE);
  lineAA(221, 116, 221, 124, 2.2f, IOS_WHITE);

  uint16_t rc = zone(roll, ROLL_WARN, ROLL_DANGER), pc = zone(pitch, PITCH_WARN, PITCH_DANGER);
  tiltPointer(-1, roll, rc == IOS_RED ? IOS_RED : TL_AMBER);
  tiltPointer(1, pitch, pc == IOS_RED ? IOS_RED : TL_AMBER);

  // машины и значения
  carRear(120, 72, roll, 0.68f);
  carSide(120, 152, pitch, 0.66f);
  char b[16];
  fmtNum(b, sizeof(b), lroundf(roll), 0);
  text(120, 110, b, F_VAL, rc == IOS_GREEN ? IOS_WHITE : rc);
  fmtNum(b, sizeof(b), lroundf(pitch), 0);
  text(120, 203, b, F_VAL, pc == IOS_GREEN ? IOS_WHITE : pc);
}

const char *dirName(float h) {
  static const char *n[8] = {"С", "СВ", "В", "ЮВ", "Ю", "ЮЗ", "З", "СЗ"};
  return n[(int)((wrap360(h) + 22.5f) / 45.0f) % 8];
}

void pageCompass(float hdg, int src, int acc) {
  // шкала поворачивается: сверху всегда направление движения
  for (int b = 0; b < 360; b += 3) {
    float a = (b - hdg - 90) * DEG, cs = cosf(a), sn = sinf(a);
    bool major = b % 30 == 0;
    float r1 = major ? 94 : 100, r2 = 108;
    lineAA(120 + r1 * cs, 120 + r1 * sn, 120 + r2 * cs, 120 + r2 * sn, major ? 2.2f : 1.2f,
           major ? IOS_WHITE : IOS_GRAY2);
  }
  // красный треугольник на севере
  {
    float a = (-hdg - 90) * DEG, cs = cosf(a), sn = sinf(a);
    float px[3] = {120 + 119 * cs, 120 + 111 * cs - 5 * sn, 120 + 111 * cs + 5 * sn};
    float py[3] = {120 + 119 * sn, 120 + 111 * sn + 5 * cs, 120 + 111 * sn - 5 * cs};
    polyAA(px, py, 3, IOS_RED);
  }
  // цифры и стороны света
  static const char *card[4] = {"С", "В", "Ю", "З"};
  for (int b = 0; b < 360; b += 30) {
    float a = (b - hdg - 90) * DEG, cs = cosf(a), sn = sinf(a);
    if (b % 90 == 0) {
      text(120 + 70 * cs, 120 + 70 * sn + 7, card[b / 90], F_MED, b == 0 ? IOS_RED : IOS_WHITE);
    } else {
      char t[6]; snprintf(t, sizeof(t), "%d", b);
      text(120 + 82 * cs, 120 + 82 * sn + 4, t, F_TINY, IOS_GRAY);
    }
  }
  // неподвижная метка сверху
  lineAA(120, 2, 120, 24, 3, IOS_WHITE);

  char b[16];
  snprintf(b, sizeof(b), "%d°", (int)lroundf(wrap360(hdg)) % 360);
  text(124, 132, b, F_BIG, IOS_WHITE);
  text(120, 156, dirName(hdg), F_SMALL, IOS_GRAY);
  const char *s = src == 2 ? (acc >= 2 ? "магнитный" : "калибровка")
                 : (USE_BNO085 && !g_bnoFound) ? "по гироскопу · BNO085 не найден"
                 : USE_BNO085 ? "по гироскопу · BNO085 молчит" : "по гироскопу";
  text(120, 174, s, F_TINY, src == 2 && acc < 2 ? IOS_ORANGE : IOS_GRAY2);
}

// ---- Простой кренометр + пробуксовка колёс ----
// Шарик катится в низкую сторону (нос вниз — вперёд, правый борт вниз — вправо).
// У каждого колеса — столбик: насколько оно крутится быстрее (вверх) или медленнее (вниз) остальных.
float wheelDev[4] = {0, 0, 0, 0};       // сглаженное отклонение от среднего, доли
uint32_t wheelSeenAt = 0;

// Фон «прибора»: к центру светлее (тёмно-синий), к краю почти чёрный.
// Плавный переход с упорядоченным сглаживанием (дизерингом), иначе в 16-битном цвете видны кольца.
static uint8_t xdR[256], xdG[256], xdB[256];
static bool xdBgInit = false;
void xdBackground() {
  if (!xdBgInit) {
    for (int i = 0; i < 256; i++) {
      float k = powf(i / 255.0f, 0.8f);                 // i — доля квадрата радиуса
      xdR[i] = 30 + (6 - 30) * k; xdG[i] = 40 + (8 - 40) * k; xdB[i] = 54 + (12 - 54) * k;
    }
    xdBgInit = true;
  }
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  const int O = SW / 2;
  const int R2 = O * O;
  for (int y = bandY0; y < bandY1; y++) {
    int dy = y - O;
    uint16_t *row = FB + (y - bandY0) * SW;
    const uint8_t *bm = bayer[y & 3];
    for (int x = 0; x < SW; x++) {
      int dx = x - O, d2 = dx * dx + dy * dy;
      int i = d2 >= R2 ? 255 : d2 * 255 / R2;
      int t = bm[x & 3];                                  // 0..15
      int r = std::min(255, xdR[i] + (t >> 1)), g = std::min(255, xdG[i] + (t >> 2)), b = std::min(255, xdB[i] + (t >> 1));
      row[x] = sw16((uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)));
    }
  }
}

// Контур скруглённого прямоугольника (линии + дуги на углах)
void rrOutline(float cx, float cy, float hw, float hh, float r, float w, uint16_t c) {
  lineAA(cx - hw + r, cy - hh, cx + hw - r, cy - hh, w, c);
  lineAA(cx - hw + r, cy + hh, cx + hw - r, cy + hh, w, c);
  lineAA(cx - hw, cy - hh + r, cx - hw, cy + hh - r, w, c);
  lineAA(cx + hw, cy - hh + r, cx + hw, cy + hh - r, w, c);
  arcAA(cx - hw + r, cy - hh + r, r, w, PI, 1.5f * PI, c);
  arcAA(cx + hw - r, cy - hh + r, r, w, 1.5f * PI, 2 * PI, c);
  arcAA(cx + hw - r, cy + hh - r, r, w, 0, 0.5f * PI, c);
  arcAA(cx - hw + r, cy + hh - r, r, w, 0.5f * PI, PI, c);
}

void pageWheels(float roll, float pitch) {
  xdBackground();
  // светящееся кольцо по краю и засечки
  const float RR = 111.6f;
  ringAA(120, 120, RR, 7, rgb(20, 70, 120), 90);
  ringAA(120, 120, RR, 3.5f, rgb(30, 100, 160), 140);
  ringAA(120, 120, RR, 1.2f, rgb(52, 128, 190));
  for (int d = 0; d < 360; d += 30) {
    float a = (d - 90) * DEG, ca = cosf(a), sa = sinf(a);
    if (d % 90 == 0) lineAA(120 + (RR - 5) * ca, 120 + (RR - 5) * sa, 120 + (RR + 0.5f) * ca, 120 + (RR + 0.5f) * sa, 1.6f, rgb(140, 200, 240));
    else lineAA(120 + (RR - 2.6f) * ca, 120 + (RR - 2.6f) * sa, 120 + RR * ca, 120 + RR * sa, 0.8f, rgb(70, 120, 160));
  }
  // пороги: 15° и 25°
  const float KB = 74.4f / ROLL_DANGER;              // лог. пикселей на градус
  ringAA(120, 120, ROLL_WARN * KB, 0.7f, rgb(120, 100, 30));
  ringAA(120, 120, ROLL_DANGER * KB, 0.7f, rgb(130, 40, 36));

  // машина контуром, как на стекле прибора
  const float CX = 120, CY = 120;
  const uint16_t OUT = rgb(90, 110, 135), OUT2 = rgb(70, 88, 110);
  rrOutline(CX, CY, 23, 46, 14, 1.1f, OUT);
  rrOutline(CX, CY - 22, 17, 6, 3, 0.8f, OUT2);      // лобовое
  rrOutline(CX, CY + 34, 16, 4, 2.5f, 0.8f, OUT2);   // заднее

  // колёса: порядок данных ABS — ПП, ПЛ, ЗП, ЗЛ; опора — медиана
  bool fresh = g_wheelAt && millis() - g_wheelAt < 1500;
  float w[4], srt[4];
  for (int i = 0; i < 4; i++) srt[i] = w[i] = g_wheel[i];
  std::sort(srt, srt + 4);
  float avg = (srt[1] + srt[2]) / 2;
  bool moving = fresh && avg >= 3;
  static uint32_t lastT = 0;
  float dt = lastT ? (millis() - lastT) / 1000.0f : 0; lastT = millis();
  float k = std::min(1.0f, dt / 0.4f);
  const float wx[4] = { 27, -27, 27, -27 }, wy[4] = { -30, -30, 30, 30 };
  for (int i = 0; i < 4; i++) {
    float target = moving ? (w[i] - avg) / avg : 0;
    wheelDev[i] += k * (target - wheelDev[i]);
    float dv = wheelDev[i];
    uint16_t c = !moving ? rgb(58, 66, 78)
               : dv > 0.08f ? IOS_RED : dv > 0.04f ? IOS_ORANGE : dv < -0.04f ? IOS_BLUE : IOS_GREEN;
    float x = CX + wx[i], y = CY + wy[i];
    if (moving && dv > 0.04f) {                      // ореол у буксующего колеса
      rrectAA(x, y, 10, 17, 9, 0, c, 45);
      rrectAA(x, y, 7.5f, 14.5f, 7, 0, c, 70);
    }
    rrectAA(x, y, 5, 12, 4, 0, c);
    float bx = CX + (wx[i] > 0 ? 45 : -45);
    lineAA(bx, y - 26, bx, y + 26, 4, rgb(40, 52, 66));
    if (moving) {
      float L = constrain(dv / 0.15f, -1.0f, 1.0f) * 26;
      if (fabsf(L) > 1) { lineAA(bx, y, bx, y - L, 9, c, 50); lineAA(bx, y, bx, y - L, 4, c); }
    }
  }

  // прицел и пузырь (катится в низкую сторону)
  const uint16_t RET = rgb(200, 210, 220);
  ringAA(120, 120, 14, 0.9f, RET);
  for (int d = 0; d < 360; d += 90) {
    float a = d * DEG, ca = cosf(a), sa = sinf(a);
    lineAA(120 + 11 * ca, 120 + 11 * sa, 120 + 19 * ca, 120 + 19 * sa, 0.9f, RET);
  }
  float bx = 120 + constrain(roll / ROLL_DANGER, -1.1f, 1.1f) * 74.4f;
  float by = 120 + constrain(pitch / PITCH_DANGER, -1.1f, 1.1f) * 74.4f;
  float lvl = std::max(fabsf(roll) / ROLL_DANGER, fabsf(pitch) / PITCH_DANGER);
  float lvlW = std::max(fabsf(roll) / ROLL_WARN, fabsf(pitch) / PITCH_WARN);
  uint16_t bc = lvl >= 1 ? IOS_RED : lvlW >= 1 ? IOS_YELLOW : IOS_GREEN;
  fillCircleAA(bx, by, 17, bc, 28);
  fillCircleAA(bx, by, 13, bc, 50);
  fillCircleAA(bx, by, 9, bc);
  uint8_t r5 = bc >> 11, g6 = (bc >> 5) & 63, b5 = bc & 31;
  uint16_t hl = rgb(std::min(255, r5 * 8 + 90), std::min(255, g6 * 4 + 90), std::min(255, b5 * 8 + 90));
  fillCircleAA(bx - 2.6f, by - 3.2f, 2.6f, hl);

  if (!fresh) text(120, 214, "нет данных ABS", F_TINY, rgb(90, 104, 120));
}

// ---- Авиагоризонт ----
constexpr uint16_t AH_SKY = rgb(26, 104, 196);   // небо
constexpr uint16_t AH_GND = rgb(128, 82, 42);    // земля
#define AH_PX_PER_DEG 2.8f                       // сдвиг горизонта, лог. пикселей на градус тангажа

// Небо/земля: по строкам прямо в буфер, сглаживание только на линии горизонта.
// (nx, ny) — нормаль горизонта в сторону земли, (cxL, cyL) — точка на горизонте (лог. коорд.)
void horizonFill(float cxL, float cyL, float nx, float ny) {
  const float O = 120 * SC, Rr = 120 * SC + 1;
  const float cx = cxL * SC, cy = cyL * SC;
  const uint16_t s = sw16(AH_SKY), g = sw16(AH_GND);
  for (int y = bandY0; y < bandY1; y++) {
    float dy = y - O, h2 = Rr * Rr - dy * dy;
    if (h2 <= 0) continue;
    float h = sqrtf(h2);
    int xa = std::max(0, (int)(O - h)), xb = std::min(SW - 1, (int)(O + h) + 1);
    uint16_t *row = FB + (y - bandY0) * SW;
    float d = nx * (xa - cx) + ny * (y - cy);
    for (int x = xa; x <= xb; x++, d += nx) {
      if (d <= -0.5f) row[x] = s;
      else if (d >= 0.5f) row[x] = g;
      else { row[x] = s; blendPx(x, y, AH_GND, (uint32_t)((d + 0.5f) * 255)); }
    }
  }
}

void ahTriangle(float ang, float rTip, float rBase, float halfW, uint16_t c) {
  float ca = cosf(ang), sa = sinf(ang);
  float px[3] = { 120 + rTip * ca, 120 + rBase * ca - halfW * sa, 120 + rBase * ca + halfW * sa };
  float py[3] = { 120 + rTip * sa, 120 + rBase * sa + halfW * ca, 120 + rBase * sa - halfW * ca };
  polyAA(px, py, 3, c);
}

void pageHorizon(float roll, float pitch, float gLat) {
  // Машина накренилась вправо — мир на экране поворачивается против часовой.
  float a = roll * DEG;
  float nx = sinf(a), ny = cosf(a);                // нормаль к горизонту, «вниз», к земле
  float tx = ny, ty = -nx;                         // направление вдоль горизонта
  float k = AH_PX_PER_DEG;
  float hcx = 120 + nx * pitch * k, hcy = 120 + ny * pitch * k;   // нос вверх — горизонт уходит вниз

  horizonFill(hcx, hcy, nx, ny);
  lineAA(hcx - tx * 130, hcy - ty * 130, hcx + tx * 130, hcy + ty * 130, 1.6f, IOS_WHITE);

  // Лестница тангажа: каждые 5°, подписи на 10°; ниже горизонта — пунктир
  char b[16];
  int lo = (int)floorf((pitch - 22) / 5) * 5, hi = (int)ceilf((pitch + 22) / 5) * 5;
  for (int th = lo; th <= hi; th += 5) {
    if (th == 0 || abs(th) > 60) continue;
    float lx = hcx - nx * th * k, ly = hcy - ny * th * k;
    float ddx = lx - 120, ddy = ly - 120;
    if (ddx * ddx + ddy * ddy > 46 * 46) continue;   // только в центральном «окне»
    bool major = th % 10 == 0;
    float hw = major ? 26 : 12;
    if (th > 0 || !major) {
      lineAA(lx - tx * hw, ly - ty * hw, lx + tx * hw, ly + ty * hw, 1.4f, IOS_WHITE);
    } else {                                         // ниже горизонта — штрихи
      for (float s : {-1.0f, 1.0f}) {
        lineAA(lx + s * tx * hw, ly + s * ty * hw, lx + s * tx * 16, ly + s * ty * 16, 1.4f, IOS_WHITE);
        lineAA(lx + s * tx * 11, ly + s * ty * 11, lx + s * tx * 4, ly + s * ty * 4, 1.4f, IOS_WHITE);
      }
    }
    if (major) {
      snprintf(b, sizeof(b), "%d", abs(th));
      for (float s : {-1.0f, 1.0f})
        text(lx + s * tx * (hw + 10), ly + s * ty * (hw + 10) + 4, b, F_TINY, IOS_WHITE);
    }
  }

  // Шкала крена (неподвижная): дуга ±60°, засечки 10, 20, 30, 45, 60
  const float RS = 98;
  arcAA(120, 120, RS, 1.5f, (-90 - 60) * DEG, (-90 + 60) * DEG, IOS_WHITE);
  for (int t : {10, 20, 30, 45, 60}) for (int s : {-1, 1}) {
    float an = (-90 + s * t) * DEG;
    float r2 = (t == 30 || t == 60) ? RS + 10 : RS + 6;
    lineAA(120 + RS * cosf(an), 120 + RS * sinf(an), 120 + r2 * cosf(an), 120 + r2 * sinf(an), 1.6f, IOS_WHITE);
  }
  // пороги предупреждения/опасности на шкале
  for (float z : {(float)ROLL_WARN, (float)ROLL_DANGER}) for (int s : {-1, 1}) {
    float an = (-90 + s * z) * DEG;
    fillCircleAA(120 + RS * cosf(an), 120 + RS * sinf(an), 2.6f, z == ROLL_WARN ? IOS_YELLOW : IOS_RED);
  }
  ahTriangle(-90 * DEG, RS + 1, RS + 11, 6, IOS_WHITE);            // нулевой индекс
  uint16_t rc = zone(roll, ROLL_WARN, ROLL_DANGER);
  ahTriangle((-90 - roll) * DEG, RS - 1, RS - 12, 6.5f, rc == IOS_GREEN ? IOS_WHITE : rc);   // указатель

  // Силуэт «самолёта» — неподвижный, жёлтый с тёмной обводкой
  const uint16_t AC = IOS_YELLOW, OL = rgb(20, 20, 20);
  for (int pass = 0; pass < 2; pass++) {
    float w = pass ? 4.0f : 6.5f; uint16_t c = pass ? AC : OL;
    lineAA(56, 120, 90, 120, w, c);
    lineAA(90, 120, 100, 130, w, c);
    lineAA(184, 120, 150, 120, w, c);
    lineAA(150, 120, 140, 130, w, c);
    fillCircleAA(120, 120, pass ? 3.2f : 4.6f, c);
  }

  // Показания крена и тангажа
  uint16_t pc = zone(pitch, PITCH_WARN, PITCH_DANGER);
  rrectAA(60, 174, 27, 17, 9, 0, IOS_BG, 170);
  rrectAA(180, 174, 27, 17, 9, 0, IOS_BG, 170);
  text(60, 168, "крен", F_TINY, IOS_GRAY);
  text(180, 168, "тангаж", F_TINY, IOS_GRAY);
  fmtNum(b, sizeof(b), lroundf(fabsf(roll)), 0, "°");
  text(60, 185, b, F_SMALL, rc == IOS_GREEN ? IOS_WHITE : rc);
  fmtNum(b, sizeof(b), lroundf(pitch), 0, "°");
  text(180, 185, b, F_SMALL, pc == IOS_GREEN ? IOS_WHITE : pc);

  // Шарик (как в указателе скольжения): уходит под уклон и наружу поворота
  float slip = sinf(a) - gLat;                     // в g
  rrectAA(120, 204, 34, 8, 8, 0, IOS_BG, 170);
  lineAA(110, 197, 110, 211, 1.3f, IOS_WHITE);
  lineAA(130, 197, 130, 211, 1.3f, IOS_WHITE);
  fillCircleAA(120 + constrain(slip, -0.5f, 0.5f) * 52, 204, 6.2f, IOS_WHITE);
}

// ---- Часы ----
const char *const WD_SHORT[7] = {"ВС", "ПН", "ВТ", "СР", "ЧТ", "ПТ", "СБ"};

// Стрелка: тонкая «шейка» от центра и широкая часть, с тёмной обводкой
void clockHand(float ang, float len, float w, uint16_t c) {
  float ca = cosf(ang), sa = sinf(ang);
  const float neck = 15;
  for (int pass = 0; pass < 2; pass++) {
    float e = pass ? 0 : 2.4f; uint16_t cc = pass ? c : IOS_BG;
    lineAA(120, 120, 120 + neck * ca, 120 + neck * sa, 2.6f + e, cc);
    lineAA(120 + neck * ca, 120 + neck * sa, 120 + len * ca, 120 + len * sa, w + e, cc);
  }
}

void clockModern(const ClockData &c, bool have, float s, float m, float h) {

  // циферблат: минутные и часовые риски
  for (int i = 0; i < 60; i++) {
    float an = (i * 6 - 90) * DEG, ca = cosf(an), sa = sinf(an);
    if (i % 5 == 0) lineAA(120 + 98 * ca, 120 + 98 * sa, 120 + 110 * ca, 120 + 110 * sa, 3.2f, IOS_WHITE);
    else            lineAA(120 + 104 * ca, 120 + 104 * sa, 120 + 110 * ca, 120 + 110 * sa, 1.2f, IOS_GRAY2);
  }
  // цифры
  char b[32];
  for (int i = 1; i <= 12; i++) {
    float an = (i * 30 - 90) * DEG;
    snprintf(b, sizeof(b), "%d", i);
    text(120 + 82 * cosf(an), 120 + 82 * sinf(an) + 7.5f, b, F_MED, i % 3 == 0 ? IOS_WHITE : IOS_GRAY);
  }

  if (have) {
    // день недели — у цифры 10, дата — у цифры 2: тем же шрифтом и на одной линии с цифрами
    const float yb = 120 - 82 * 0.5f + 7.5f;     // базовая линия цифр 10 и 2
    text(73, yb, WD_SHORT[c.wday % 7], F_MED, IOS_ORANGE, AL_LEFT);
    snprintf(b, sizeof(b), "%02d.%02d", c.day, c.mon);
    text(171, yb, b, F_MED, IOS_WHITE, AL_RIGHT);
  } else {
    text(120, 150, c.ok ? "Время не задано" : "Нет часов RTC", F_TINY, IOS_ORANGE);
    text(120, 166, "Wi-Fi часов, затем", F_TINY, IOS_GRAY);
    text(120, 180, "192.168.4.1", F_TINY, IOS_GRAY);
  }

  // стрелки
  clockHand((h * 30 - 90) * DEG, 56, 7.5f, IOS_WHITE);
  clockHand((m * 6 - 90) * DEG, 94, 6.0f, IOS_WHITE);
  fillCircleAA(120, 120, 5.5f, IOS_WHITE);
  if (have) {
    float an = (s * 6 - 90) * DEG, ca = cosf(an), sa = sinf(an);
    lineAA(120 - 20 * ca, 120 - 20 * sa, 120 + 106 * ca, 120 + 106 * sa, 1.8f, IOS_ORANGE);
    fillCircleAA(120, 120, 4.2f, IOS_ORANGE);
  }
  fillCircleAA(120, 120, 1.7f, IOS_BG);
}

// ---- Часы «прибор»: светлый циферблат в духе советской круговой линейки ----
constexpr uint16_t VT_INK    = rgb(26, 28, 30);
constexpr uint16_t VT_INK2   = rgb(84, 88, 90);
constexpr uint16_t VT_NEEDLE = rgb(118, 16, 24);    // тёмно-вишнёвые стрелки
constexpr uint16_t VT_SEC    = rgb(176, 26, 30);
constexpr uint16_t VT_BRASS  = rgb(206, 176, 96);

// Тонкая стрелка-игла (ромб): хвост, расширение у центра, острый кончик
void needle(float ang, float len, float tail, float w, uint16_t c) {
  float ca = cosf(ang), sa = sinf(ang);
  float px[4] = { 120 - tail * ca, 120 - w * sa, 120 + len * ca, 120 + w * sa };
  float py[4] = { 120 - tail * sa, 120 + w * ca, 120 + len * sa, 120 - w * ca };
  polyAA(px, py, 4, c);
}

void clockVintage(const ClockData &c, bool have, float s, float m, float h) {
  // неподвижная часть (ободок, шкалы, цифры) — готовая картинка из dial_vintage.h
  memcpy(FB, DIAL_VT + bandY0 * SW, (size_t)(bandY1 - bandY0) * SW * 2);

  // окошко «день + число» на шкале секунд у «3 часов» (размеры в пикселях экрана)
  char b[24];
  if (have) snprintf(b, sizeof(b), "%s %d", WD_SHORT[c.wday % 7], c.day);
  else strcpy(b, "— —");
  const float P = 1.0f / SC;                       // пиксель экрана в логических единицах
  float hw = textWidth(F_DATE, b) / 2.0f + 7.5f;
  float wx = 233 + 213.2f - hw;
  rrectAA(wx * P, 120, (hw + 1.5f) * P, 18.5f * P, 0, 0, rgb(120, 124, 126));
  rrectAA(wx * P, 120, hw * P, 17 * P, 0, 0, rgb(250, 250, 247));
  text(wx * P, 120 + 13 * P, b, F_DATE, VT_INK);

  // стрелки
  needle((h * 30 - 90) * DEG, 139.8f * P, 23.3f * P, 4.4f * P, VT_INK);
  needle((m * 6 - 90) * DEG, 205.0f * P, 30.3f * P, 3.3f * P, VT_INK);
  fillCircleAA(120, 120, 9 * P, VT_INK);
  if (have) {
    float an = (s * 6 - 90) * DEG, ca = cosf(an), sa = sinf(an);
    lineAA(120 - 46.6f * P * ca, 120 - 46.6f * P * sa, 120 + 216.7f * P * ca, 120 + 216.7f * P * sa, 1.6f * P, VT_SEC);
  }
  fillCircleAA(120, 120, 6 * P, VT_SEC);
  fillCircleAA(120, 120, 2.6f * P, VT_BRASS);
}

int clockStyle = 0;                                // 0 — iOS, 1 — «прибор»

void pageClock() {
  ClockData c;
  portENTER_CRITICAL(&mux); c = g_clock; portEXIT_CRITICAL(&mux);
  bool have = c.ok && c.valid;
  float frac = have ? std::min(0.999f, (millis() - c.secAt) / 1000.0f) : 0;
  float s = have ? c.ss + frac : 0;
  float m = have ? c.mm + s / 60 : 10;
  float h = have ? (c.hh % 12) + m / 60 : 10 + 10 / 60.0f;
  if (clockStyle == 1) clockVintage(c, have, s, m, h);
  else clockModern(c, have, s, m, h);
}

// ---- Путь ----
static float niceScale(float metersNeeded) {
  static const float steps[] = {50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000};
  for (float s : steps) if (metersNeeded <= s) return s;
  return 500000;
}

void fmtDist(char *out, size_t n, float m) {
  if (m < 1000) snprintf(out, n, "%d м", (int)lroundf(m));
  else fmtNum(out, n, m / 1000.0f, m < 10000 ? 1 : 0, " км");
}

int16_t trkPx[TRACK_MAX], trkPy[TRACK_MAX];

void pagePath(float hdg, int src, float posX, float posY, float odo) {
  const float VIEW_R = 84;        // радиус карты в пикселях
  // масштаб: чтобы весь трек и точка старта помещались вокруг текущей позиции
  int n = 0;
  float far2 = posX * posX + posY * posY;
  xSemaphoreTake(trackLock, portMAX_DELAY);
  n = trackN;
  for (int i = 0; i < n; i++) {
    float dx = trackX[i] - posX, dy = trackY[i] - posY;
    float d2 = dx * dx + dy * dy;
    if (d2 > far2) far2 = d2;
  }
  float scaleM = niceScale(sqrtf(far2) * 1.05f);   // метров на VIEW_R пикселей
  float k = VIEW_R / scaleM;
  int m = 0;
  for (int i = 0; i < n; i++) {
    int16_t x = (int16_t)lroundf(120 + (trackX[i] - posX) * k);
    int16_t y = (int16_t)lroundf(120 - (trackY[i] - posY) * k);
    if (m > 0 && abs(x - trkPx[m - 1]) < 2 && abs(y - trkPy[m - 1]) < 2) continue;
    trkPx[m] = x; trkPy[m] = y; m++;
  }
  xSemaphoreGive(trackLock);

  // фон карты
  fillCircleAA(120, 120, VIEW_R + 14, IOS_CARD);
  ringAA(120, 120, VIEW_R * 0.5f, 1, IOS_TRACK);
  ringAA(120, 120, VIEW_R, 1, IOS_TRACK);
  // «С» сверху (карта ориентирована на север)
  {
    float px[3] = {120, 115, 125}, py[3] = {120 - VIEW_R - 9, 120 - VIEW_R + 1, 120 - VIEW_R + 1};
    polyAA(px, py, 3, IOS_RED);
  }

  // трек
  for (int i = 1; i < m; i++) lineAA(trkPx[i - 1], trkPy[i - 1], trkPx[i], trkPy[i], 4, IOS_BLUE);
  if (m > 0) lineAA(trkPx[m - 1], trkPy[m - 1], 120, 120, 4, IOS_BLUE);

  // старт
  float sx = 120 - posX * k, sy = 120 + posY * k;
  fillCircleAA(sx, sy, 6.5f, IOS_WHITE);
  fillCircleAA(sx, sy, 4.5f, IOS_GREEN);

  // текущая позиция: «луч» направления + точка как в Картах
  {
    float a = (hdg - 90) * DEG, half = 28 * DEG;
    float px[3] = {120, 120 + 30 * cosf(a - half), 120 + 30 * cosf(a + half)};
    float py[3] = {120, 120 + 30 * sinf(a - half), 120 + 30 * sinf(a + half)};
    polyAA(px, py, 3, IOS_BLUE, 90);
    fillCircleAA(120, 120, 9, IOS_WHITE);
    fillCircleAA(120, 120, 6.5f, IOS_BLUE);
  }

  char b[64];
  fmtDist(b, sizeof(b), odo);
  text(120, 24, b, F_SMALL, IOS_WHITE);

  float toStart = sqrtf(posX * posX + posY * posY);
  char d[32];
  fmtDist(d, sizeof(d), toStart);
  snprintf(b, sizeof(b), "до старта %s", d);
  text(120, 213, b, F_TINY, IOS_GRAY);

  // масштаб: половина радиуса карты
  char sc[16];
  fmtDist(sc, sizeof(sc), scaleM / 2);
  lineAA(120 - VIEW_R / 4, 196, 120 + VIEW_R / 4, 196, 2, IOS_GRAY2);
  text(120, 192, sc, F_TINY, IOS_GRAY2);

  if (!(USE_CAN && g_canOk)) {
    rrectAA(120, 160, 74, 13, 13, 0, IOS_BG, 200);
    text(120, 164, "нет скорости с CAN", F_TINY, IOS_ORANGE);
  } else if (src != 2) {
    text(120, 52, "курс по гироскопу", F_TINY, IOS_GRAY2);
  }
}

// ---- Дополнительные приборы ----
struct Gauge {
  const char *title, *unit;
  float vmin, vmax, cold, warn, danger;
  int dec;
};

uint16_t gaugeColor(const Gauge &g, float v) {
  if (!isnan(g.cold) && v < g.cold) return IOS_TEAL;
  if (v >= g.danger) return IOS_RED;
  if (v >= g.warn) return IOS_YELLOW;
  return IOS_GREEN;
}

void pageGauge(const Gauge &g, float v, bool valid, float peak, const char *sub) {
  const float R = 100, W = 13, A0 = 135 * DEG, A1 = 405 * DEG;
  arcAA(120, 120, R, W, A0, A1, IOS_TRACK);
  text(120, 70, g.title, F_SMALL, IOS_GRAY);
  char b[48], lo[16], hi[16];
  fmtNum(lo, sizeof(lo), g.vmin, g.dec > 0 ? 0 : 0);
  fmtNum(hi, sizeof(hi), g.vmax, 0);
  text(120 + R * cosf(A0) + 4, 120 + R * sinf(A0) + 22, lo, F_TINY, IOS_GRAY2);
  text(120 + R * cosf(A1) - 4, 120 + R * sinf(A1) + 22, hi, F_TINY, IOS_GRAY2);

  if (!valid) {
    text(120, 142, "\xE2\x80\x94", F_HUGE, IOS_GRAY4);
    text(120, 170, "нет данных", F_SMALL, IOS_GRAY2);
    return;
  }
  float t = (constrain(v, g.vmin, g.vmax) - g.vmin) / (g.vmax - g.vmin);
  uint16_t c = gaugeColor(g, v);
  arcAA(120, 120, R, W, A0, A0 + (A1 - A0) * t, c);
  if (!isnan(peak) && peak > g.vmin) {
    float tp = (constrain(peak, g.vmin, g.vmax) - g.vmin) / (g.vmax - g.vmin);
    float ap = A0 + (A1 - A0) * tp;
    fillCircleAA(120 + R * cosf(ap), 120 + R * sinf(ap), 3, IOS_WHITE);
  }
  fmtNum(b, sizeof(b), v, g.dec);
  text(120, 146, b, F_HUGE, IOS_WHITE);
  text(120, 170, g.unit, F_SMALL, IOS_GRAY);
  if (!isnan(peak) && peak > g.vmin) {
    char p[24]; fmtNum(p, sizeof(p), peak, g.dec);
    snprintf(b, sizeof(b), "макс %s", p);
    text(120, 190, b, F_TINY, IOS_GRAY2);
  }
  if (sub && sub[0]) text(120, 206, sub, F_TINY, IOS_GRAY2);
}

const Gauge G_BOOST   = {"Наддув", "бар", 0, BOOST_MAX, NAN, BOOST_WARN, BOOST_DANGER, 2};
const Gauge G_COOLANT = {"Температура ОЖ", "°C", 40, 120, COOL_COLD, COOL_WARN, COOL_DANGER, 0};
const Gauge G_RPM     = {"Обороты", "об/мин", 0, RPM_MAX, NAN, RPM_WARN, RPM_DANGER, 0};

#define GM_SCALE 96.0f
#define GM_TRAIL 40
float gmTx[GM_TRAIL], gmTy[GM_TRAIL];
uint8_t gmPos = 0;
bool gmInit = false;
float gmAcc = 0, gmBrake = 0, gmLeft = 0, gmRight = 0;

void gmeterUpdate(float lon, float lat) {
  float x = 120 + constrain(lat, -1.1f, 1.1f) * GM_SCALE, y = 120 - constrain(lon, -1.1f, 1.1f) * GM_SCALE;
  if (!gmInit) { for (int i = 0; i < GM_TRAIL; i++) { gmTx[i] = x; gmTy[i] = y; } gmInit = true; }
  gmTx[gmPos] = x; gmTy[gmPos] = y; gmPos = (gmPos + 1) % GM_TRAIL;
  gmAcc = std::max(gmAcc, lon); gmBrake = std::max(gmBrake, -lon);
  gmLeft = std::max(gmLeft, -lat); gmRight = std::max(gmRight, lat);
}

void pageGMeter(float lon, float lat) {
  for (float r : {0.25f, 0.5f, 0.75f, 1.0f}) ringAA(120, 120, r * GM_SCALE, r == 1.0f ? 1.5f : 1, IOS_TRACK);
  lineAA(24, 120, 216, 120, 1, IOS_TRACK);
  lineAA(120, 24, 120, 216, 1, IOS_TRACK);
  text(124, 120 - 0.5f * GM_SCALE + 12, "0,5", F_TINY, IOS_GRAY4, AL_LEFT);
  for (int i = 0; i < GM_TRAIL; i++) {
    int age = (gmPos - 1 - i + GM_TRAIL) % GM_TRAIL;     // 0 — свежая
    fillCircleAA(gmTx[i], gmTy[i], 2.2f, IOS_BLUE, (uint8_t)(200 - age * 4));
  }
  float total = sqrtf(lon * lon + lat * lat);
  uint16_t c = total > 0.6f ? IOS_RED : (total > 0.35f ? IOS_YELLOW : IOS_GREEN);
  float x = 120 + constrain(lat, -1.1f, 1.1f) * GM_SCALE, y = 120 - constrain(lon, -1.1f, 1.1f) * GM_SCALE;
  fillCircleAA(x, y, 10, IOS_WHITE);
  fillCircleAA(x, y, 7.5f, c);

  char b[24];
  fmtNum(b, sizeof(b), total, 2, " g");
  text(120, 176, b, F_MED, IOS_WHITE);
  fmtNum(b, sizeof(b), gmAcc, 2);   text(120, 40, b, F_TINY, IOS_GRAY);
  fmtNum(b, sizeof(b), gmBrake, 2); text(120, 210, b, F_TINY, IOS_GRAY);
  fmtNum(b, sizeof(b), gmLeft, 2);  text(36, 116, b, F_TINY, IOS_GRAY);
  fmtNum(b, sizeof(b), gmRight, 2); text(204, 116, b, F_TINY, IOS_GRAY);
}

// ===================== Wi-Fi и обновление (OTA) =====================
WebServer server(80);
bool serverStarted = false;
uint32_t wifiLastActivity = 0;
volatile int otaPercent = -1;
bool otaError = false;

const char PAGE_HEAD[] PROGMEM = R"HTML(<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark"><title>Кренометр</title><style>
body{font-family:-apple-system,system-ui,sans-serif;background:#000;color:#fff;margin:0;padding:20px 16px;max-width:520px}
h1{font-size:34px;font-weight:700;margin:8px 0 2px}.sub{color:#8e8e93;font-size:15px;margin-bottom:20px}
.card{background:#1c1c1e;border-radius:14px;padding:16px;margin-bottom:14px}
.v{font-size:34px;font-weight:600}.l{color:#8e8e93;font-size:13px;margin-bottom:2px}
.row{display:flex;gap:14px}.row>div{flex:1}
input[type=file]{width:100%;color:#8e8e93;margin:12px 0}
button{width:100%;padding:14px;border:0;border-radius:12px;background:#0a84ff;color:#fff;font-size:17px;font-weight:600}
progress{width:100%;height:6px;margin-top:12px;display:none;accent-color:#0a84ff}
#st{margin-top:10px;color:#8e8e93;font-size:14px}
input:not([type=file]){width:100%;box-sizing:border-box;padding:10px;border-radius:10px;border:0;background:#2c2c2e;color:#fff;font-size:16px;margin:4px 0 8px}
pre{background:#000;color:#30d158;border-radius:10px;padding:10px;font-size:13px;max-height:260px;overflow:auto;white-space:pre-wrap;margin:10px 0 0}
</style></head><body>)HTML";

void sendIndex() {
  wifiLastActivity = millis();
  String h = FPSTR(PAGE_HEAD);
  h += "<h1>Кренометр</h1><div class='sub'>Прошивка " FW_VERSION "</div>";
  h += R"HTML(<div class='card row'><div><div class='l'>Крен</div><div class='v' id='r'>—</div></div>
<div><div class='l'>Дифферент</div><div class='v' id='p'>—</div></div></div>
<div class='card row'><div><div class='l'>Курс</div><div class='v' id='h'>—</div></div>
<div><div class='l'>Пройдено</div><div class='v' id='o'>—</div></div></div>
<div class='card row'><div><div class='l'>Скорость</div><div class='v' id='s'>—</div></div>
<div><div class='l'>Наддув</div><div class='v' id='b'>—</div></div></div>
<div class='card row'><div><div class='l'>Темп. ОЖ</div><div class='v' id='t'>—</div></div>
<div><div class='l'>Напряжение</div><div class='v' id='u'>—</div></div></div>
<div class='l' id='cs' style='margin:-4px 4px 16px'></div><button onclick="fetch('/rot')" style='margin-bottom:14px;background:#2c2c2e'>Повернуть экран на 90°</button>)HTML";
  h += String("<button onclick=\"fetch('/scr?on=") + (g_screenOn ? "0" : "1") + "').then(()=>location.reload())\" style='margin-bottom:14px;background:#2c2c2e'>"
       + (g_screenOn ? "Выключить экран кренометра" : "Включить экран кренометра") + "</button>"
       + "<button onclick=\"fetch('/ap?on=" + (g_apAlways ? "0" : "1") + "').then(()=>location.reload())\" style='margin-bottom:14px;background:#2c2c2e'>"
       + (g_apAlways ? "Сеть Krenometr: всегда (для приложения) — сделать на 10 мин" : "Сеть Krenometr: 10 мин — сделать всегда") + "</button>";
#if CAR_LINK == 1
  h += "<div class='card'><b>Адаптер ELM327</b><div class='l' id='es' style='margin:4px 0 10px'>—</div>"
       "<div class='l'>Сеть Wi-Fi адаптера</div><input id='es1' value='" + String(elmCfg.ssid) + "'>"
       "<div class='l'>Пароль (если нет — пусто)</div><input id='es2' value='" + String(elmCfg.pass) + "'>"
       "<div class='row'><div><div class='l'>IP</div><input id='es3' value='" + String(elmCfg.host) + "'></div>"
       "<div><div class='l'>Порт</div><input id='es4' value='" + String(elmCfg.port) + "'></div>"
       "<div><div class='l'>Протокол</div><input id='es5' value='" + String(elmCfg.proto) + "'></div></div>"
       "<button onclick='sv()' style='margin-top:12px'>Сохранить и переподключить</button></div>"
       "<div class='card'><b>Терминал</b><div class='l' style='margin:4px 0 8px'>Команды ELM327 и запросы к ЭБУ: ATRV, ATSH7E0, 2101, ATSH7E1, 2102, 03…</div>"
       "<div class='row'><div style='flex:3'><input id='tc' placeholder='ATRV' autocapitalize='characters'></div>"
       "<div><button onclick='tx()'>Отпр.</button></div></div><pre id='to'></pre></div>";
#endif
  h += R"HTML(
<div class='card'><b>Обновление прошивки</b>
<div class='l' style='margin-top:4px'>Файл krenometr.ino.bin (Скетч → Экспорт бинарного файла)</div>
<input type='file' id='f' accept='.bin'><button onclick='up()'>Загрузить</button>
<progress id='pg' max='100' value='0'></progress><div id='st'></div></div>
<script>
const $=i=>document.getElementById(i),c=(v,d)=>v.toFixed(d).replace('.',',').replace('-','−');
fetch('/t?e='+Math.round(Date.now()/1000-new Date().getTimezoneOffset()*60));
setInterval(()=>fetch('/a').then(x=>x.json()).then(d=>{
$('r').textContent=c(d.r,1)+'°';$('p').textContent=c(d.p,1)+'°';
$('h').textContent=Math.round(d.h)%360+'°';$('o').textContent=d.o<1000?Math.round(d.o)+' м':c(d.o/1000,1)+' км';
$('s').textContent=d.c?d.s+' км/ч':'—';$('b').textContent=d.c&&d.m>0?c(d.b,2)+' бар':'—';
$('t').textContent=d.c&&d.t>-100?d.t+'°C':'—';$('u').textContent=d.c&&d.u>0?c(d.u,1)+' В':'—';
$('cs').textContent=(d.c?'Машина: данные идут':'Машина: нет данных')+(d.f>-100?' · ATF '+d.f+'°C':'')+' · компас: '+(d.hs==2?'магнитный':'гироскоп')+(d.bn==0?' (BNO085 не найден)':d.bn==1?' (BNO085 найден, данных нет)':'')+' · крен: '+(d.ts==2?'BNO085':'датчик платы');
if($('es'))$('es').textContent=['Ищу Wi-Fi сеть адаптера…','Сеть есть, адаптер не отвечает (занят магнитолой?)','Адаптер подключён, ЭБУ молчит (зажигание?)','Подключено, данные идут'][d.e]||'—';
}).catch(()=>{}),500);
function sv(){const q=i=>encodeURIComponent($(i).value);fetch('/elmcfg?s='+q('es1')+'&p='+q('es2')+'&h='+q('es3')+'&o='+q('es4')+'&r='+q('es5')).then(()=>{$('es').textContent='Сохранено, переподключаюсь…'})}
let ts=-1;function tx(){const c=$('tc').value.trim();if(!c)return;$('to').textContent+='> '+c+'\n';fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>{if(t=='busy'){$('to').textContent+='(занят)\n';return}ts=+t;pl()})}
function pl(){fetch('/cmdr').then(r=>r.text()).then(t=>{const i=t.indexOf('\n'),n=+t.slice(0,i),o=t.slice(i+1);if(n>ts){$('to').textContent+=o.replace(/\n+/g,'\n').trim()+'\n';$('to').scrollTop=1e9}else setTimeout(pl,300)})}
function up(){const f=$('f').files[0];if(!f){$('st').textContent='Выберите файл';return}
const x=new XMLHttpRequest(),fd=new FormData();fd.append('fw',f,f.name);
$('pg').style.display='block';$('st').textContent='Загрузка…';
x.upload.onprogress=e=>{if(e.lengthComputable)$('pg').value=e.loaded/e.total*100};
x.onload=()=>{$('st').textContent=x.status==200?'Готово, кренометр перезагружается':'Ошибка: '+x.responseText};
x.onerror=()=>{$('st').textContent='Связь прервалась'};
x.open('POST','/update');x.send(fd)}
</script></body></html>)HTML";
  server.send(200, "text/html; charset=utf-8", h);
}

#if CAR_LINK == 1
void handleElmCfg() {
  wifiLastActivity = millis();
  prefsElm.putString("ssid", server.arg("s"));
  prefsElm.putString("pass", server.arg("p"));
  prefsElm.putString("host", server.arg("h"));
  prefsElm.putUShort("port", (uint16_t)server.arg("o").toInt());
  prefsElm.putString("proto", server.arg("r"));
  prefsElm.remove("ch"); prefsElm.remove("bs");   // другая сеть — канал узнаем заново
  elmScanAt = 0;
  g_elmReconnect = true;
  server.send(200, "text/plain", "ok");
}
void handleTermCmd() {
  wifiLastActivity = millis();
  if (elmTermPending) { server.send(200, "text/plain", "busy"); return; }
  strlcpy(elmTermCmd, server.arg("c").c_str(), sizeof(elmTermCmd));
  elmTermOut[0] = 0;
  elmTermPending = true;
  server.send(200, "text/plain", String(elmTermSeq));
}
void handleTermResult() {
  wifiLastActivity = millis();
  String r = String(elmTermSeq) + "\n" + (elmTermPending ? String("…") : String(elmTermOut));
  server.send(200, "text/plain; charset=utf-8", r);
}
#endif

// ===================== v3.10: Android-приложение =====================
volatile bool     g_screenOn = true;     // экран кренометра (выключается из приложения)
volatile bool     g_apAlways = true;     // своя сеть работает постоянно, без отключения через 10 минут
volatile bool     g_appWheels = false;   // в приложении открыт экран колёс — опрашивать ABS
volatile uint32_t g_appSeenAt = 0;       // когда приложение последний раз отзывалось
extern Preferences prefsUi;

bool appActive() { return g_appSeenAt != 0 && millis() - g_appSeenAt < APP_TIMEOUT_MS; }

void screenSet(bool on) {
  g_screenOn = on;
  prefsUi.putBool("scr", on);
}

#if CAR_LINK == 1
void handleTermEnd() {                   // приложение закрыло терминал — опрос возобновляется сразу
  wifiLastActivity = millis();
  elmTermLastAt = 0;
  server.send(200, "text/plain", "ok");
}
#endif

void handleScreen() {
  wifiLastActivity = millis();
  if (server.hasArg("on")) screenSet(server.arg("on") != "0");
  server.send(200, "text/plain", g_screenOn ? "1" : "0");
}

void handleApAlways() {
  wifiLastActivity = millis();
  if (server.hasArg("on")) { g_apAlways = server.arg("on") != "0"; prefsUi.putBool("ap", g_apAlways); }
  server.send(200, "text/plain", g_apAlways ? "1" : "0");
}

void handleZero() {
  wifiLastActivity = millis();
  g_zeroRequest = true;
  maxRoll = maxPitch = 0;
  toast("Ноль установлен");
  server.send(200, "text/plain", "ok");
}

void handleTrip() {
  wifiLastActivity = millis();
  g_tripReset = true;
  toast("Новая поездка");
  server.send(200, "text/plain", "ok");
}

// ---------- общие для Wi-Fi и USB ----------
void appInfoJson(char *b, size_t n) {
  char id[24]; strlcpy(id, g_elmId, sizeof(id));
  for (char *q = id; *q; q++) if (*q == '"' || *q == '\\' || (uint8_t)*q < 32) *q = ' ';
  snprintf(b, n, "{\"fw\":\"" FW_VERSION "\",\"elm\":\"%s\",\"ch\":%d,\"scr\":%d,\"ap\":%d,\"rot\":%d}",
           id, (int)elmChan, g_screenOn ? 1 : 0, g_apAlways ? 1 : 0, g_rot);
}

// Трек текущей поездки: первая строка «номер_поездки число_точек», дальше «x y» в метрах
void trackDump(void (*emit)(const char *, int, void *), void *ctx) {
  xSemaphoreTake(trackLock, portMAX_DELAY);
  int n = trackN;
  uint32_t tid = g_tripId;
  float *buf = (float *)heap_caps_malloc((size_t)(n > 0 ? n : 1) * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
  if (buf) for (int i = 0; i < n; i++) { buf[2 * i] = trackX[i]; buf[2 * i + 1] = trackY[i]; }
  xSemaphoreGive(trackLock);
  if (!buf) n = 0;
  char chunk[1100];
  int len = snprintf(chunk, sizeof(chunk), "%lu %d\n", (unsigned long)tid, n);
  for (int i = 0; i < n; i++) {
    len += snprintf(chunk + len, sizeof(chunk) - len, "%.1f %.1f\n", buf[2 * i], buf[2 * i + 1]);
    if (len > (int)sizeof(chunk) - 40) { emit(chunk, len, ctx); len = 0; }
  }
  if (len) emit(chunk, len, ctx);
  if (buf) free(buf);
}

static void emitHttp(const char *s, int n, void *) { server.sendContent(s, n); }

void handleTrack() {
  wifiLastActivity = millis();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain", "");
  trackDump(emitHttp, nullptr);
  server.sendContent("");
}

void handleInfo() {
  wifiLastActivity = millis();
  char b[200];
  appInfoJson(b, sizeof(b));
  server.send(200, "application/json", b);
}

// GPS магнитолы: «G1 широта долгота точность скорость». Начало координат — первый фикс поездки,
// привязанный к текущей точке счисления; дальше путь мягко подтягивается к GPS.
void appGps(const char *line) {
  double lat, lon; float acc, spd;
  if (sscanf(line + 2, "%lf %lf %f %f", &lat, &lon, &acc, &spd) != 4) return;
  if (acc <= 0 || lat < -90 || lat > 90 || lon < -180 || lon > 180) return;
  g_gpsAcc = acc; g_gpsSpeed = spd < 0 ? 0 : spd; g_gpsAt = millis();
  static bool have = false;
  static uint32_t trip = 0;
  static double lat0 = 0, lon0 = 0;
  static float offX = 0, offY = 0;
  float px, py;
  portENTER_CRITICAL(&mux); px = g_posX; py = g_posY; portEXIT_CRITICAL(&mux);
  if (!have || trip != g_tripId || acc > 50) {
    if (acc > 50) return;                        // слишком грубо даже для начала координат
    have = true; trip = g_tripId; lat0 = lat; lon0 = lon; offX = px; offY = py;
    return;
  }
  float gx = offX + (float)((lon - lon0) * cos(lat0 * PI / 180.0) * 111320.0);
  float gy = offY + (float)((lat - lat0) * 110540.0);
  float k = acc <= 10 ? 0.25f : acc <= 25 ? 0.10f : 0.0f;
  if (k <= 0) return;
  portENTER_CRITICAL(&mux);
  g_gpsCorrX += k * (gx - px); g_gpsCorrY += k * (gy - py);
  portEXIT_CRITICAL(&mux);
}

// Пакет данных для приложения (JSON одной строкой), возвращает длину
uint32_t appSeq = 0;
int appPacket(char *pkt, size_t sz) {
  float roll, pitch, lon, lat, hdg, px, py, odo;
  int hs;
  portENTER_CRITICAL(&mux);
  roll = g_roll; pitch = g_pitch; lon = g_gLon; lat = g_gLat; hdg = g_heading; hs = g_headingSrc;
  px = g_posX; py = g_posY; odo = g_odometer;
  portEXIT_CRITICAL(&mux);
  uint32_t now = millis();
  auto fr = [now](uint32_t at, uint32_t ms) { return at != 0 && now - at < ms; };
  bool spdOk = USE_CAN && g_canOk && fr(g_speedAt, 1500);
  bool mapOk = USE_CAN && fr(g_mapAt, 3000);
  float baro = fr(g_baroAt, 60000) ? (float)g_baroKpa : 101.3f;
  long wAge = g_wheelAt ? (long)(now - g_wheelAt) : -1;
  return snprintf(pkt, sz,
    "{\"v\":1,\"n\":%lu,\"cal\":%d,\"imu\":%d,"
    "\"r\":%.2f,\"p\":%.2f,\"lo\":%.3f,\"la\":%.3f,"
    "\"h\":%.1f,\"hs\":%d,\"ma\":%d,\"bn\":%d,\"ts\":%d,"
    "\"x\":%.1f,\"y\":%.1f,\"o\":%.1f,\"ti\":%lu,\"tn\":%d,"
    "\"e\":%d,\"c\":%d,\"sp\":%d,\"rpm\":%d,\"bo\":%.2f,\"bok\":%d,"
    "\"cl\":%d,\"ia\":%d,\"u\":%.1f,\"f\":%d,"
    "\"w\":[%d,%d,%d,%d],\"wa\":%ld,"
    "\"scr\":%d,\"ap\":%d,\"zd\":%lu,\"tm\":%d,\"ga\":%.0f}",
    (unsigned long)appSeq++, g_calibrating ? 1 : 0, g_imuOk ? 1 : 0,
    roll, pitch, lon, lat,
    hdg, hs, (int)g_magAccuracy, !USE_BNO085 ? -1 : !g_bnoFound ? 0 : hs == 2 ? 2 : 1, (int)g_tiltSrc,
    px, py, odo, (unsigned long)g_tripId, (int)trackN,
    CAR_LINK == 1 ? (int)g_elmState : -1, g_canOk ? 1 : 0,
    spdOk ? (int)g_speedKmh : -1,
    USE_CAN && fr(g_rpmAt, 3000) ? (int)g_rpm : -1,
    mapOk ? (g_mapKpa - baro) / 100.0f : 0.0f, mapOk ? 1 : 0,
    USE_CAN && fr(g_coolAt, 30000) ? (int)g_coolant : -999,
    USE_CAN && fr(g_iatAt, 5000) ? (int)g_iat : -999,
    g_canOk ? (float)g_volt : -1.0f,
    fr(g_atfAt, 30000) ? (int)g_atf : -999,
    (int)g_wheel[0], (int)g_wheel[1], (int)g_wheel[2], (int)g_wheel[3], wAge,
    g_screenOn ? 1 : 0, g_apAlways ? 1 : 0, (unsigned long)g_zeroDoneAt,
    (elmTermLastAt && now - elmTermLastAt < 60000) ? 1 : 0,
    fr(g_gpsAt, 3000) ? (float)g_gpsAcc : -1.0f);
}

void appHello(const char *in) {
  g_appWheels = strstr(in, "w=1") != nullptr;
  g_appSeenAt = millis();
}

// ---------- USB: запросы «Q<номер> /адрес?параметры» ----------
// значение параметра из «/x?a=1&c=AT%20RV» (с раскодированием %XX и +)
static bool qArg(const char *path, const char *key, char *out, size_t n) {
  const char *q = strchr(path, '?');
  size_t kl = strlen(key);
  while (q) {
    q++;
    if (strncmp(q, key, kl) == 0 && q[kl] == '=') {
      const char *v = q + kl + 1;
      size_t k = 0;
      while (*v && *v != '&' && k + 1 < n) {
        if (*v == '%' && v[1] && v[2]) { char h[3] = {v[1], v[2], 0}; out[k++] = (char)strtol(h, nullptr, 16); v += 3; }
        else if (*v == '+') { out[k++] = ' '; v++; }
        else out[k++] = *v++;
      }
      out[k] = 0;
      return true;
    }
    q = strchr(q, '&');
  }
  return false;
}

static bool pathIs(const char *path, const char *name) {
  size_t l = strlen(name);
  return strncmp(path, name, l) == 0 && (path[l] == 0 || path[l] == '?');
}

// вывод ответа в USB: «\n» внутри текста превращается в два знака «\» «n»
static void usbEsc(const char *s, int n, void *) {
  char o[256]; int k = 0;
  for (int i = 0; i < n; i++) {
    char c = s[i];
    if (c == '\r') continue;
    if (k > (int)sizeof(o) - 3) { Serial.write((const uint8_t *)o, k); k = 0; }
    if (c == '\n') { o[k++] = '\\'; o[k++] = 'n'; }
    else if (c == '\\') { o[k++] = '\\'; o[k++] = '\\'; }
    else o[k++] = c;
  }
  if (k) Serial.write((const uint8_t *)o, k);
}

void usbRequest(const char *line) {
  char *sp = nullptr;
  long id = strtol(line + 1, &sp, 10);
  if (!sp || *sp != ' ') return;
  const char *path = sp + 1;
  char a[80];
  char body[260];
  body[0] = 0;
  bool track = false;
  g_appSeenAt = millis();
  if (pathIs(path, "/scr")) { if (qArg(path, "on", a, sizeof(a))) screenSet(strcmp(a, "0") != 0); strcpy(body, g_screenOn ? "1" : "0"); }
  else if (pathIs(path, "/ap")) {
    if (qArg(path, "on", a, sizeof(a))) { g_apAlways = strcmp(a, "0") != 0; prefsUi.putBool("ap", g_apAlways); }
    strcpy(body, g_apAlways ? "1" : "0");
  }
  else if (pathIs(path, "/zero")) { g_zeroRequest = true; maxRoll = maxPitch = 0; toast("Ноль установлен"); strcpy(body, "ok"); }
  else if (pathIs(path, "/trip")) { g_tripReset = true; toast("Новая поездка"); strcpy(body, "ok"); }
  else if (pathIs(path, "/rot")) { g_rot = (g_rot + 1) & 3; prefsUi.putInt("rot", g_rot); snprintf(body, sizeof(body), "%d", g_rot); }
  else if (pathIs(path, "/t")) {
    if (qArg(path, "e", a, sizeof(a))) { long e = atol(a); if (e > 1700000000L) { g_rtcSetEpoch = (uint32_t)e; g_rtcSetReq = true; } }
    strcpy(body, "ok");
  }
  else if (pathIs(path, "/info")) appInfoJson(body, sizeof(body));
  else if (pathIs(path, "/trk")) track = true;
#if CAR_LINK == 1
  else if (pathIs(path, "/cmd")) {
    if (elmTermPending) strcpy(body, "busy");
    else {
      if (!qArg(path, "c", elmTermCmd, sizeof(elmTermCmd))) elmTermCmd[0] = 0;
      elmTermOut[0] = 0;
      elmTermPending = true;
      snprintf(body, sizeof(body), "%lu", (unsigned long)elmTermSeq);
    }
  }
  else if (pathIs(path, "/cmdr")) {
    char hdr[16]; snprintf(hdr, sizeof(hdr), "R%ld %lu\\n", id, (unsigned long)elmTermSeq);
    Serial.write((const uint8_t *)hdr, strlen(hdr));
    if (elmTermPending) Serial.write((const uint8_t *)"…", strlen("…"));
    else usbEsc(elmTermOut, strlen(elmTermOut), nullptr);
    Serial.write('\n');
    return;
  }
  else if (pathIs(path, "/cmdend")) { elmTermLastAt = 0; strcpy(body, "ok"); }
#endif
  else strcpy(body, "?");
  char hdr[16]; snprintf(hdr, sizeof(hdr), "R%ld ", id);
  Serial.write((const uint8_t *)hdr, strlen(hdr));
  if (track) trackDump(usbEsc, nullptr);
  else usbEsc(body, strlen(body), nullptr);
  Serial.write('\n');
}

// Поток данных для приложения. Wi-Fi: раз в секунду «K1 w=0» на UDP 4210, ответ — пакеты JSON.
// USB: те же строки по кабелю (приоритетнее: пока USB живой, по Wi-Fi не шлём).
WiFiUDP appUdp;
void appUdpTask(void *) {
  bool bound = false;
  IPAddress cliIp; uint16_t cliPort = 0;
  uint32_t udpSeenAt = 0, usbSeenAt = 0;
  char pkt[700];
  char ln[200]; int lnLen = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(APP_PERIOD_MS));

    // --- USB: входящие строки ---
    while (Serial.available() > 0) {
      int c = Serial.read();
      if (c < 0) break;
      if (c == '\r') continue;
      if (c != '\n') { if (lnLen < (int)sizeof(ln) - 1) ln[lnLen++] = (char)c; continue; }
      ln[lnLen] = 0;
      if (lnLen >= 2 && ln[0] == 'K' && ln[1] == '1') { appHello(ln); usbSeenAt = millis(); }
      else if (lnLen >= 2 && ln[0] == 'G' && ln[1] == '1') appGps(ln);
      else if (lnLen >= 3 && ln[0] == 'Q') { usbSeenAt = millis(); usbRequest(ln); }
      lnLen = 0;
    }
    bool usbLive = usbSeenAt != 0 && millis() - usbSeenAt < APP_TIMEOUT_MS;

    // --- Wi-Fi: входящие пакеты ---
    if (!wifiOn) { if (bound) { appUdp.stop(); bound = false; } }
    else if (!bound) bound = appUdp.begin(APP_UDP_PORT);
    if (bound) {
      for (int sz = appUdp.parsePacket(); sz > 0; sz = appUdp.parsePacket()) {
        char in[96];
        int k = appUdp.read((uint8_t *)in, sizeof(in) - 1);
        if (k < 2) continue;
        in[k] = 0;
        if (in[0] == 'K' && in[1] == '1') {
          cliIp = appUdp.remoteIP(); cliPort = appUdp.remotePort();
          appHello(in); udpSeenAt = millis();
          wifiLastActivity = millis();
        } else if (in[0] == 'G' && in[1] == '1') appGps(in);
      }
    }
    bool udpLive = bound && cliPort && udpSeenAt != 0 && millis() - udpSeenAt < APP_TIMEOUT_MS;
    if (!usbLive && !udpLive) continue;

    int n = appPacket(pkt, sizeof(pkt) - 1);
    if (n <= 0 || n >= (int)sizeof(pkt) - 1) continue;
    if (usbLive) {
      pkt[n] = '\n';
      Serial.write((const uint8_t *)pkt, n + 1);   // одной записью — отладочные строки не вклинятся
    } else {
      appUdp.beginPacket(cliIp, cliPort);
      appUdp.write((const uint8_t *)pkt, n);
      appUdp.endPacket();
    }
  }
}

void handleRotate() {
  extern Preferences prefsUi;
  wifiLastActivity = millis();
  g_rot = (g_rot + 1) & 3;
  prefsUi.putInt("rot", g_rot);
  server.send(200, "text/plain", String(g_rot));
}

void setTimeFromPhone() {
  wifiLastActivity = millis();
  long e = server.arg("e").toInt();
  if (e > 1700000000L) { g_rtcSetEpoch = (uint32_t)e; g_rtcSetReq = true; toast("Время с телефона"); }
  server.send(200, "text/plain", "ok");
}

void sendData() {
  wifiLastActivity = millis();
  float roll, pitch, hdg, odo;
  int hs;
  portENTER_CRITICAL(&mux);
  roll = g_roll; pitch = g_pitch; hdg = g_heading; hs = g_headingSrc; odo = g_odometer;
  portEXIT_CRITICAL(&mux);
  float baro = (g_baroAt && millis() - g_baroAt < 60000) ? g_baroKpa : 101.3f;
  bool mapOk = g_mapAt && millis() - g_mapAt < 3000;
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"r\":%.2f,\"p\":%.2f,\"h\":%.1f,\"hs\":%d,\"o\":%.1f,\"c\":%d,\"s\":%d,\"m\":%d,\"b\":%.2f,\"t\":%d,\"u\":%.2f,\"e\":%d,\"f\":%d,\"bn\":%d,\"ts\":%d}",
           roll, pitch, hdg, hs, odo, g_canOk ? 1 : 0, (int)g_speedKmh, mapOk ? 1 : 0,
           mapOk ? (g_mapKpa - baro) / 100.0f : 0.0f, (int)g_coolant, (float)g_volt,
           CAR_LINK == 1 ? (int)g_elmState : -1, (g_atfAt && millis() - g_atfAt < 30000) ? (int)g_atf : -999,
           !USE_BNO085 ? -1 : !g_bnoFound ? 0 : g_headingSrc == 2 ? 2 : 1, (int)g_tiltSrc);
  server.send(200, "application/json", buf);
}

void present(void (*scene)());
void otaScene() {
  int p = otaPercent < 0 ? 0 : otaPercent;
  arcAA(120, 120, 100, 8, 0, 2 * PI, IOS_TRACK);
  if (p > 0) arcAA(120, 120, 100, 8, -PI / 2, -PI / 2 + 2 * PI * p / 100.0f, IOS_BLUE);
  char b[8]; snprintf(b, sizeof(b), "%d%%", p);
  text(120, 136, b, F_BIG, IOS_WHITE);
  text(120, 84, "Обновление", F_SMALL, IOS_GRAY);
  text(120, 166, "не выключайте", F_TINY, IOS_GRAY2);
}
void drawOtaScreen() { present(otaScene); }

void handleUploadChunk() {
  HTTPUpload &up = server.upload();
  wifiLastActivity = millis();
  if (up.status == UPLOAD_FILE_START) {
    otaError = false; otaPercent = 0;
    if (!g_screenOn) lcdSetBrightness(BRIGHTNESS);   // прогресс обновления видно и при выключенном экране
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) otaError = true;
    drawOtaScreen();
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!otaError && Update.write(up.buf, up.currentSize) != up.currentSize) otaError = true;
    size_t total = server.header("Content-Length").toInt();
    if (total > 0) {
      int pct = (int)(100ULL * up.totalSize / total);
      if (pct != otaPercent) { otaPercent = pct; drawOtaScreen(); }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (!otaError && !Update.end(true)) otaError = true;
    otaPercent = otaError ? -1 : 100;
    drawOtaScreen();
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); otaError = true; otaPercent = -1;
  }
}

void handleUploadDone() {
  if (otaError || Update.hasError()) {
    server.send(500, "text/plain; charset=utf-8", String("не удалось записать прошивку: ") + Update.errorString());
    otaPercent = -1;
    return;
  }
  server.send(200, "text/plain; charset=utf-8", "OK");
  delay(500);
  ESP.restart();
}

void wifiStart() {
  if (wifiOn) return;                            // уже работает — не перезапускать (магнитола отвалится)
  WiFi.mode(CAR_LINK == 1 ? WIFI_AP_STA : WIFI_AP);
  // канал своей сети = канал адаптера, иначе при подключении к адаптеру сеть «переедет» и магнитола отвалится
  int ch = (CAR_LINK == 1 && WiFi.status() == WL_CONNECTED) ? WiFi.channel() : (elmChan ? elmChan : 1);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, ch);
  if (!serverStarted) {
    const char *hdrs[] = {"Content-Length"};
    server.collectHeaders(hdrs, 1);
    server.on("/", HTTP_GET, sendIndex);
    server.on("/a", HTTP_GET, sendData);
    server.on("/t", HTTP_GET, setTimeFromPhone);
    server.on("/rot", HTTP_GET, handleRotate);
#if CAR_LINK == 1
    server.on("/elmcfg", HTTP_GET, handleElmCfg);
    server.on("/cmd", HTTP_GET, handleTermCmd);
    server.on("/cmdr", HTTP_GET, handleTermResult);
    server.on("/cmdend", HTTP_GET, handleTermEnd);
#endif
    server.on("/scr", HTTP_GET, handleScreen);
    server.on("/ap", HTTP_GET, handleApAlways);
    server.on("/zero", HTTP_GET, handleZero);
    server.on("/trip", HTTP_GET, handleTrip);
    server.on("/trk", HTTP_GET, handleTrack);
    server.on("/info", HTTP_GET, handleInfo);
    server.on("/update", HTTP_POST, handleUploadDone, handleUploadChunk);
    server.onNotFound(sendIndex);
    serverStarted = true;
  }
  server.begin();
  wifiOn = true;
  wifiLastActivity = millis();
}

void wifiStop() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(CAR_LINK == 1 ? WIFI_STA : WIFI_OFF);   // связь с ELM327 не рвём
  wifiOn = false;
}

void wifiService() {
  if (!wifiOn) return;
  server.handleClient();
  if (otaPercent >= 0 && otaPercent < 100 && millis() - wifiLastActivity > 30000UL) {
    Update.abort(); otaPercent = -1;
  }
  if (WiFi.softAPgetStationNum() > 0) wifiLastActivity = millis();
  if (!g_apAlways && millis() - wifiLastActivity > WIFI_TIMEOUT_MIN * 60000UL) { wifiStop(); toast("Wi-Fi выключен"); }
}

// ===================== Основная программа (ядро 1) =====================
enum { PAGE_TILT = 0, PAGE_WHEELS, PAGE_HORIZON, PAGE_COMPASS, PAGE_PATH, PAGE_CLOCK,
#if EXTRA_PAGES
       PAGE_BOOST, PAGE_COOLANT, PAGE_RPM, PAGE_GMETER,
#endif
       NUM_PAGES };

int page = PAGE_TILT;
uint32_t pageChangedAt = 0;
bool pageSaved = true;
Preferences prefsUi;
float peakBoost = 0, peakCool = NAN, peakRpm = 0;

uint32_t pressStart = 0;
bool wasPressed = false, longDone = false, moved = false;
int16_t tStartX = 0, tStartY = 0, tLastX = 0, tLastY = 0;
uint32_t lastTapAt = 0;

bool fresh(uint32_t at, uint32_t ms) { return at != 0 && millis() - at < ms; }

void resetPagePeaks() {
  switch (page) {
    case PAGE_TILT: case PAGE_HORIZON: maxRoll = maxPitch = 0; break;
#if EXTRA_PAGES
    case PAGE_BOOST:   peakBoost = 0; break;
    case PAGE_COOLANT: peakCool = NAN; break;
    case PAGE_RPM:     peakRpm = 0; break;
    case PAGE_GMETER:  gmAcc = gmBrake = gmLeft = gmRight = 0; break;
#endif
    default: return;
  }
  toast("Максимумы сброшены");
}

bool longPressPage() { return page == PAGE_TILT || page == PAGE_HORIZON || page == PAGE_WHEELS || page == PAGE_PATH; }

void handleTouch() {
  bool pressed = g_touch;
  if (!g_screenOn) {                             // экран выключен из приложения: касание — включить
    if (pressed && !wasPressed) { screenSet(true); longDone = true; }
    wasPressed = pressed;
    return;
  }
  int16_t tx = g_tx, ty = g_ty;
  if (pressed && !wasPressed) {
    pressStart = millis(); longDone = false; moved = false;
    tStartX = tLastX = tx; tStartY = tLastY = ty;
  }
  if (pressed) {
    tLastX = tx; tLastY = ty;
    if (abs(tx - tStartX) > 20 || abs(ty - tStartY) > 20) moved = true;
  }
  uint32_t held = pressed ? millis() - pressStart : 0;
  if (pressed && !moved && !longDone && longPressPage() && held >= LONGPRESS_MS) {
    longDone = true;
    if (page == PAGE_TILT || page == PAGE_HORIZON || page == PAGE_WHEELS) { g_zeroRequest = true; maxRoll = maxPitch = 0; toast("Ноль установлен"); }
    else { g_tripReset = true; toast("Новая поездка"); }
  }
  if (!pressed && wasPressed && !longDone) {
    int dx = tLastX - tStartX, dy = tLastY - tStartY;
    if (moved && abs(dx) > 50 && abs(dx) > abs(dy)) {
      page = (page + (dx < 0 ? 1 : NUM_PAGES - 1)) % NUM_PAGES;
      pageChangedAt = millis(); pageSaved = false; lastTapAt = 0;
    } else if (!moved && millis() - pressStart < 800) {
      if (lastTapAt != 0 && pressStart - lastTapAt < 450) {
        if (wifiOn) { wifiStop(); toast("Wi-Fi выключен"); }
        else { wifiStart(); toast("Wi-Fi: " WIFI_SSID); }
        lastTapAt = 0;
      } else {
        lastTapAt = millis();                      // одиночное касание выполнится, если не будет второго
      }
    }
  }
  if (!pressed && lastTapAt != 0 && millis() - lastTapAt >= 450) {
    lastTapAt = 0;
    if (page == PAGE_CLOCK) { clockStyle ^= 1; prefsUi.putInt("clk", clockStyle); }
    else resetPagePeaks();
  }
  wasPressed = pressed;
  if (!pageSaved && millis() - pageChangedAt > 5000) { prefsUi.putInt("page", page); pageSaved = true; }
}

void setup() {
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxBufferSize(8192);          // поток для приложения по USB (v3.11)
  Serial.setRxBufferSize(1024);
  Serial.setTxTimeoutMs(20);             // магнитола не читает — не тормозить
#endif
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // ждём, пока компьютер откроет порт USB
  Serial.println();
  Serial.println("Krenometr " FW_VERSION " start");
  Serial.printf("PSRAM: %u bytes\n", (unsigned)ESP.getPsramSize());

  pinMode(PIN_LCD_EN, OUTPUT);
  digitalWrite(PIN_LCD_EN, HIGH);        // включить питание AMOLED
  delay(50);
  bool ok = panel->begin();
  Serial.printf("display begin: %d\n", ok);
  panel->setBrightness(BRIGHTNESS);
  // два буфера полос во внутренней памяти, доступной для DMA: пока один уходит на экран, рисуется другой
  for (int rows : {60, 40, 30, 20}) {
    bandBuf[0] = (uint16_t *)heap_caps_malloc((size_t)SW * rows * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    bandBuf[1] = (uint16_t *)heap_caps_malloc((size_t)SW * rows * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (bandBuf[0] && bandBuf[1]) { bandRows = rows; break; }
    free(bandBuf[0]); free(bandBuf[1]); bandBuf[0] = bandBuf[1] = nullptr;
  }
  if (!bandBuf[0]) { Serial.println("ERROR: no memory for band buffers"); for (;;) delay(1000); }
  FB = bandBuf[0];
  rotFB = (uint16_t *)heap_caps_malloc((size_t)SW * SW * 2, MALLOC_CAP_SPIRAM);   // кадр для поворота экрана
  Serial.printf("rotation frame: %d\n", rotFB != nullptr);
  Serial.printf("band buffers: %d rows, free internal %u\n", bandRows, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  bool fast = lcdFastInit();
  Serial.printf("fast LCD (DMA, %d MHz): %d\n", QSPI_HZ / 1000000, fast);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  for (uint8_t a = 1; a < 127; a++) {      // что видно на шине I2C
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("I2C device at 0x%02X\n", a);
  }
  prefs.begin("krenometr", false);
  prefsUi.begin("kren_ui", false);
  page = prefsUi.getInt("page", PAGE_TILT);
  clockStyle = prefsUi.getInt("clk", 0) & 1;
  g_rot = prefsUi.getInt("rot", DISPLAY_ROT) & 3;
  g_screenOn = prefsUi.getBool("scr", true);
  g_apAlways = prefsUi.getBool("ap", true);
  if (page < 0 || page >= NUM_PAGES) page = PAGE_TILT;

  trackLock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, NULL, 3, NULL, 0);
#if CAR_LINK == 2
  xTaskCreatePinnedToCore(canTask, "can", 4096, NULL, 2, NULL, 0);
#elif CAR_LINK == 1
  elmLoadCfg();                          // канал адаптера нужен своей сети ещё до запуска задачи ELM
  if (g_apAlways) wifiStart(); else WiFi.mode(WIFI_STA);
  xTaskCreatePinnedToCore(elmTask, "elm", 8192, NULL, 2, NULL, 0);
#else
  if (g_apAlways) wifiStart();
#endif
  xTaskCreatePinnedToCore(appUdpTask, "app", 4096, NULL, 1, NULL, 0);
}

// Состояние кадра: считается один раз, а рисуется по полосам
struct FrameState {
  float roll, pitch, lon, lat, hdg, px, py, odo, boost;
  int src;
  bool mapOk, coolOk, rpmOk;
  char sub[64];
} frm;

void updateFrame() {
  if (!g_imuOk && !g_calibrating) {
    static bool autoWifi = false;
    if (!autoWifi) { autoWifi = true; wifiStart(); }
    return;
  }
  if (g_calibrating) return;

  portENTER_CRITICAL(&mux);
  frm.roll = g_roll; frm.pitch = g_pitch; frm.lon = g_gLon; frm.lat = g_gLat;
  frm.hdg = g_heading; frm.src = g_headingSrc; frm.px = g_posX; frm.py = g_posY; frm.odo = g_odometer;
  portEXIT_CRITICAL(&mux);

  maxRoll = std::max(maxRoll, fabsf(frm.roll));
  maxPitch = std::max(maxPitch, fabsf(frm.pitch));
  gmeterUpdate(frm.lon, frm.lat);

  frm.mapOk = USE_CAN && fresh(g_mapAt, 3000);
  float baro = fresh(g_baroAt, 60000) ? (float)g_baroKpa : 101.3f;
  frm.boost = frm.mapOk ? (g_mapKpa - baro) / 100.0f : 0;
  if (frm.mapOk) peakBoost = std::max(peakBoost, frm.boost);
  frm.coolOk = USE_CAN && fresh(g_coolAt, 30000);   // ОЖ меняется медленно
  if (frm.coolOk) peakCool = isnan(peakCool) ? (float)g_coolant : std::max(peakCool, (float)g_coolant);
  frm.rpmOk = USE_CAN && fresh(g_rpmAt, 3000);
  if (frm.rpmOk) peakRpm = std::max(peakRpm, (float)g_rpm);

  handleTouch();

  frm.sub[0] = 0;
#if EXTRA_PAGES
  if (page == PAGE_BOOST && fresh(g_iatAt, 5000)) snprintf(frm.sub, sizeof(frm.sub), "впуск %d°C", (int)g_iat);
  if (page == PAGE_RPM && g_volt > 0) {
    char v[24]; fmtNum(v, sizeof(v), g_volt, 1, " В");
    snprintf(frm.sub, sizeof(frm.sub), "бортсеть %s", v);
  }
#endif
}

void renderScene() {
  if (!g_imuOk && !g_calibrating) {
    text(120, 112, "Датчик не найден", F_MED, IOS_RED);
    text(120, 136, "QMI8658 не отвечает", F_SMALL, IOS_GRAY);
    if (wifiOn) text(120, 164, "Wi-Fi: " WIFI_SSID " · 192.168.4.1", F_TINY, IOS_BLUE);
    return;
  }
  if (g_calibrating) {
    float a = millis() / 300.0f;
    arcAA(120, 120, 60, 6, a, a + 1.6f, IOS_BLUE);
    text(120, 116, "Не двигайте", F_MED, IOS_WHITE);
    text(120, 138, "калибровка", F_SMALL, IOS_GRAY);
    return;
  }

  switch (page) {
    case PAGE_TILT:    pageTilt(frm.roll, frm.pitch); break;
    case PAGE_HORIZON: pageHorizon(frm.roll, frm.pitch, frm.lat); break;
    case PAGE_WHEELS:  pageWheels(frm.roll, frm.pitch); break;
    case PAGE_COMPASS: pageCompass(frm.hdg, frm.src, g_magAccuracy); break;
    case PAGE_CLOCK:   pageClock(); break;
    case PAGE_PATH:    pagePath(frm.hdg, frm.src, frm.px, frm.py, frm.odo); break;
#if EXTRA_PAGES
    case PAGE_BOOST:   pageGauge(G_BOOST, frm.boost, frm.mapOk, peakBoost, frm.sub); break;
    case PAGE_COOLANT: pageGauge(G_COOLANT, g_coolant, frm.coolOk, peakCool, ""); break;
    case PAGE_RPM:     pageGauge(G_RPM, g_rpm, frm.rpmOk, peakRpm, frm.sub); break;
    case PAGE_GMETER:  pageGMeter(frm.lon, frm.lat); break;
#endif
  }

  uint32_t held = wasPressed ? millis() - pressStart : 0;
  if (longPressPage() && wasPressed && !moved && !longDone && held > 400)
    holdRing(std::min(1.0f, (held - 400) / float(LONGPRESS_MS - 400)));

  pageDots(page, NUM_PAGES);
  drawToast();
}

// Нарисовать сцену полосами и отправить каждую на экран
// Поворот экрана: кадр рисуется полосами как обычно, каждая полоса переписывается
// повёрнутой в полный кадр в PSRAM, затем кадр уходит на экран полосами.
static void rotateBandInto(const uint16_t *src, int y0, int rows) {
  const int W = SW;
  for (int lx = 0; lx < W; lx++) {
    const uint16_t *sp = src + lx;
    switch (g_rot) {
      case 1: { uint16_t *d = rotFB + lx * W + (W - 1 - y0);         // px = W-1-ly, py = lx
                for (int r = 0; r < rows; r++) *(d - r) = sp[r * W]; } break;
      case 2: { uint16_t *d = rotFB + (W - 1 - y0) * W + (W - 1 - lx); // px = W-1-lx, py = W-1-ly
                for (int r = 0; r < rows; r++) *(d - r * W) = sp[r * W]; } break;
      default: { uint16_t *d = rotFB + (W - 1 - lx) * W + y0;         // px = ly, py = W-1-lx
                for (int r = 0; r < rows; r++) d[r] = sp[r * W]; } break;
    }
  }
}

void presentRotated(void (*scene)()) {
  // 1) нарисовать и повернуть
  for (int y = 0; y < SW; y += bandRows) {
    bandY0 = y; bandY1 = std::min(SW, y + bandRows);
    int rows = bandY1 - bandY0;
    if (lcdInflightBuf == 0) lcdWait();
    FB = bandBuf[0];
    uint32_t t0 = micros();
    memset(FB, 0, (size_t)SW * rows * 2);
    scene();
    rotateBandInto(FB, bandY0, rows);
    tRenderUs += micros() - t0;
  }
  // 2) отправить полосами
  int bi = 0;
  for (int y = 0; y < SW; y += bandRows, bi ^= 1) {
    int rows = std::min(bandRows, SW - y);
    uint32_t t0 = micros();
    if (lcdInflightBuf == bi) lcdWait();
    memcpy(bandBuf[bi], rotFB + y * SW, (size_t)SW * rows * 2);
    if (lcdDev) lcdSendBand(bi, y, rows);
    else {
      for (int i = 0; i < SW * rows; i++) bandBuf[bi][i] = sw16(bandBuf[bi][i]);
      panel->draw16bitRGBBitmap(0, y, bandBuf[bi], SW, rows);
    }
    tSendUs += micros() - t0;
    pxSent += (uint32_t)SW * rows;
  }
}

void present(void (*scene)()) {
  if (g_rot && rotFB) { presentRotated(scene); return; }
  int bi = 0;
  for (int y = 0; y < SW; y += bandRows, bi ^= 1) {
    bandY0 = y;
    bandY1 = std::min(SW, y + bandRows);
    int rows = bandY1 - bandY0;
    uint32_t t0 = micros();
    if (lcdInflightBuf == bi) lcdWait();          // этот буфер ещё отправляется
    tSendUs += micros() - t0;
    FB = bandBuf[bi];
    t0 = micros();
    memset(FB, 0, (size_t)SW * rows * 2);
    scene();
    uint32_t t1 = micros();
    tRenderUs += t1 - t0;
    if (lcdDev) {
      lcdSendBand(bi, bandY0, rows);             // уходит в фоне, пока рисуется следующая полоса
    } else {                                     // запасной путь через библиотеку (она сама переставляет байты)
      for (int i = 0; i < SW * rows; i++) FB[i] = sw16(FB[i]);
      panel->draw16bitRGBBitmap(0, bandY0, FB, SW, rows);
    }
    tSendUs += micros() - t1;
    pxSent += (uint32_t)SW * rows;
  }
}

void blankScene() {}

void drawFrame() {
  updateFrame();
  present(renderScene);
}

void loop() {
  wifiService();
  if (otaPercent >= 0) { delay(2); return; }
  static uint32_t lastFrame = 0;
  if (millis() - lastFrame < 33) { delay(2); return; }
  lastFrame = millis();
  g_wantWheels = (g_screenOn && page == PAGE_WHEELS) || (appActive() && g_appWheels);
  static bool shownOn = true;
  if (g_screenOn != shownOn) {                   // экран выключают: чёрный кадр и яркость 0
    shownOn = g_screenOn;
    if (!shownOn) { present(blankScene); lcdSetBrightness(0); }
    else lcdSetBrightness(BRIGHTNESS);
  }
  if (shownOn) drawFrame();
  else updateFrame();                            // касания, максимумы — без рисования

  // раз в 2 секунды — состояние в Serial Monitor
  static uint32_t lastLog = 0, frames = 0;
  frames++;
  if (millis() - lastLog >= 2000) {
    Serial.printf("v" FW_VERSION " fps=%.1f draw=%ums send=%ums sent=%u%% imu=%d roll=%.1f pitch=%.1f wifi=%d\n",
                  frames * 1000.0f / (millis() - lastLog),
                  (unsigned)(tRenderUs / 1000 / (frames ? frames : 1)), (unsigned)(tSendUs / 1000 / (frames ? frames : 1)),
                  (unsigned)(pxSent * 100ULL / ((uint64_t)SW * SW * (frames ? frames : 1))),
                  (int)g_imuOk, (float)g_roll, (float)g_pitch, (int)wifiOn);
    tRenderUs = tSendUs = pxSent = 0;
    frames = 0;
    lastLog = millis();
  }
}
