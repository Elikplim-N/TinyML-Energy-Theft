/*
 * SMART-ENERGY THEFT — ESP32 + INA219 + Edge Impulse + OLED UI
 * UI phases: BOOT → LOADING (first EI window) → RUN or ALERT animation
 * Serial protocol (unchanged):
 *   Host->ESP32: "RELAY:ON\n" | "RELAY:OFF\n"
 *   ESP32->Host: "OK:RELAY:ON\n" | "OK:RELAY:OFF\n" | "ERR:UNKNOWN_CMD\n"
 * Telemetry (per EI window):
 *   DATA:V=%.3f,I=%.3f,P=%.3f,THEFT=%.2f,ALERT=%d
 */

// ==== Edge Impulse model ====
#include <TinyMLEtheftv1_inferencing.h>   //email to get acces this model I created

// ==== System / drivers ====
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include "esp_bt.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
#endif

// ====== PINS ======
static constexpr int RELAY_PIN         = 16;
static constexpr bool RELAY_ACTIVE_LOW = false;
static constexpr int STATUS_LED        = 2;

// ====== I2C / OLED ======
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
static uint8_t OLED_ADDR = 0x3C;      // will auto-detect 0x3C/0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ====== INA219 ======
Adafruit_INA219 ina(0x40);

// ====== EI timing / buffer ======
#ifndef EI_CLASSIFIER_INTERVAL_MS
  #define EI_CLASSIFIER_INTERVAL_MS (1000.0f / EI_CLASSIFIER_FREQUENCY)
#endif
static constexpr size_t kTotalLen  = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
static constexpr size_t kRawCount  = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
static_assert(kTotalLen % kRawCount == 0, "Model input length mismatch.");
static constexpr size_t kModelAxes = kTotalLen / kRawCount;
static float features[kTotalLen];

// ====== UI State / Metrics ======
enum UiPhase : uint8_t { UI_BOOT=0, UI_LOADING, UI_RUN, UI_ALERT };

struct Metrics {
  float V, I, P;       // dashboard
  float theft;         // 0..1
  bool  alert;         // alert state
  bool  relay;         // relay on/off
  UiPhase phase;       // current UI phase
  float progress;      // 0..1 (used in LOADING)
};
static Metrics g_last = {0,0,0,0,false,false, UI_BOOT, 0.0f};
static SemaphoreHandle_t g_last_mtx = nullptr;

static inline void set_metrics(const Metrics& m) {
  if (g_last_mtx && xSemaphoreTake(g_last_mtx, portMAX_DELAY) == pdTRUE) {
    g_last = m;
    xSemaphoreGive(g_last_mtx);
  }
}
static inline Metrics get_metrics() {
  Metrics m;
  if (g_last_mtx && xSemaphoreTake(g_last_mtx, portMAX_DELAY) == pdTRUE) {
    m = g_last;
    xSemaphoreGive(g_last_mtx);
  } else {
    m = g_last;
  }
  return m;
}

// ====== Theft smoothing / decision ======
static float theft_prob_ewma = 0.0f;
static constexpr float EWMA_ALPHA = 0.5f;
static constexpr float THEFT_THRESHOLD = 0.6f;
static constexpr int   VOTE_N = 3;
static int   vote_hist[VOTE_N] = {0};
static int   vote_hist_idx = 0;

// ====== Relay helpers ======
inline void relayWrite(bool on) {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? !on : on);
}
inline bool relayIsOn() {
  int v = digitalRead(RELAY_PIN);
  return RELAY_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// ====== Serial command parser ======
static void parseCommandByte(uint8_t c) {
  static char buf[32];
  static size_t len = 0;
  if (c == '\r') return;
  if (c != '\n') { if (len < sizeof(buf)-1) buf[len++] = (char)c; return; }
  buf[len] = '\0';

  if      (strcmp(buf, "RELAY:ON")  == 0) { relayWrite(true);  Serial.println("OK:RELAY:ON"); }
  else if (strcmp(buf, "RELAY:OFF") == 0) { relayWrite(false); Serial.println("OK:RELAY:OFF"); }
  else if (len)                           { Serial.println("ERR:UNKNOWN_CMD"); }

  len = 0;
}
static inline void handleSerialInput() { while (Serial.available()) parseCommandByte((uint8_t)Serial.read()); }

// ====== EI helpers ======
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}
static bool label_has(const char* L, const char* s) { return strstr(L, s) != nullptr; }

static float theft_metric_from_labels(const ei_impulse_result_t& res, bool relay_on) {
  // Robust to label changes (e.g., "theft", "tamper", "normal", or on/off pairs)
  float p_theft = 0.0f, p_normal = 0.0f, p_tamper = 0.0f;
  float p_on = 0.0f, p_off = 0.0f;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char* L = res.classification[i].label; float v = res.classification[i].value;
    if (label_has(L, "theft"))  p_theft += v;
    if (label_has(L, "tamper")) p_tamper += v;
    if (label_has(L, "normal")) p_normal += v;
    if (label_has(L, "on"))     p_on     += v;
    if (label_has(L, "off"))    p_off    += v;
  }
  if ((p_theft + p_tamper) > 0.0f) return p_theft + p_tamper;
  if (p_normal > 0.0f)             return 1.0f - p_normal;
  if (p_on > 0.0f || p_off > 0.0f) return relay_on ? p_on : p_off;
  return 1.0f - p_normal;
}

// ====== Tiny warning icon (24x24) ======
static const uint8_t PROGMEM kAlertTri_24x24[] = {
  0x00,0x00,0x80, 0x00,0x01,0xC0, 0x00,0x03,0xE0,
  0x00,0x07,0xF0, 0x00,0x0F,0xF8, 0x00,0x1F,0xFC,
  0x00,0x3E,0x7E, 0x00,0x7C,0x3F, 0x00,0xF8,0x1F,
  0x01,0xF0,0x0F, 0x03,0xE0,0x07, 0x07,0xC0,0x03,
  0x0F,0x80,0x01, 0x1F,0xFF,0xF8, 0x3F,0xFF,0xFC,
  0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE,
  0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE,
  0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE, 0x7F,0xFF,0xFE
};

// ====== Tasks ======
static void taskSerial(void *pv) {
  for (;;) { handleSerialInput(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

// Auto-detect OLED addr among {0x3C,0x3D}
static uint8_t detectOLED(uint8_t def=0x3C) {
  for (uint8_t a : {uint8_t(0x3C), uint8_t(0x3D)}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) return a;
  }
  return def;
}

static void taskML(void *pv) {
  const TickType_t period = pdMS_TO_TICKS((uint32_t)EI_CLASSIFIER_INTERVAL_MS);
  TickType_t tick = xTaskGetTickCount();
  bool first_window = true;

  for (;;) {
    size_t ix = 0;
    float sumV = 0.f, sumI = 0.f, sumP = 0.f;

    for (size_t n = 0; n < kRawCount; n++) {
      vTaskDelayUntil(&tick, period);

      float V = ina.getBusVoltage_V();         // V
      float I = ina.getCurrent_mA() * 0.001f;  // A
      float P = V * I;
      float R = relayIsOn() ? 1.0f : 0.0f;

      // Pack into model-expected axes
      if (ix + kModelAxes <= kTotalLen) {
        size_t base = ix;
        if (kModelAxes >= 1) features[base + 0] = V;
        if (kModelAxes >= 2) features[base + 1] = I;
        if (kModelAxes >= 3) features[base + 2] = P;
        if (kModelAxes >= 4) features[base + 3] = R;
        for (size_t k = 4; k < kModelAxes; k++) features[base + k] = 0.0f;
        ix += kModelAxes;
      }

      sumV += V; sumI += I; sumP += P;

      // Update loading progress during first window
      if (first_window) {
        Metrics mm = get_metrics();
        mm.phase = UI_LOADING;
        mm.progress = float(n + 1) / float(kRawCount);
        set_metrics(mm);
      }
    }

    // Run classifier
    signal_t signal; signal.total_length = kTotalLen; signal.get_data = &raw_feature_get_data;
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) { Serial.printf("ERR:Classifier(%d)\n", err); continue; }

    bool relay_on = relayIsOn();
    float theft_prob = theft_metric_from_labels(result, relay_on);

    // EWMA + majority vote
    theft_prob_ewma = EWMA_ALPHA * theft_prob + (1.0f - EWMA_ALPHA) * theft_prob_ewma;
    vote_hist[vote_hist_idx] = (theft_prob >= 0.5f) ? 1 : 0;
    vote_hist_idx = (vote_hist_idx + 1) % VOTE_N;
    int vote_sum = 0; for (int i = 0; i < VOTE_N; i++) vote_sum += vote_hist[i];
    bool vote_theft = (vote_sum >= ((VOTE_N + 1) / 2));
    bool alert = (theft_prob_ewma >= THEFT_THRESHOLD) || vote_theft;

    digitalWrite(STATUS_LED, alert ? HIGH : LOW);

    const float V_mean = sumV / kRawCount;
    const float I_mean = sumI / kRawCount;
    const float P_mean = sumP / kRawCount;

    // Telemetry for Flask
    Serial.printf("DATA:V=%.3f,I=%.3f,P=%.3f,THEFT=%.2f,ALERT=%d\n",
                  V_mean, I_mean, P_mean, theft_prob_ewma, alert ? 1 : 0);

    // Update metrics for UI
    Metrics m = get_metrics();
    m.V = V_mean; m.I = I_mean; m.P = P_mean;
    m.theft = theft_prob_ewma; m.alert = alert; m.relay = relay_on;
    m.progress = 1.0f;
    m.phase = alert ? UI_ALERT : UI_RUN;
    set_metrics(m);

    first_window = false;
  }
}

// ====== OLED RENDERING ======
static void drawBootFrame(uint16_t frame) {
  // Simple slide-in + dot spinner under logo (approx 2s total)
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int x = (int)(-80 + (frame * 6));     // slide from left
  if (x > 8) x = 8;
  display.setCursor(x, 10);
  display.print("SMART");
  display.setCursor(x, 32);
  display.print("EMETER");

  // spinner dots
  display.setTextSize(1);
  display.setCursor(8, 54);
  const char* dots[] = {".  ", ".. ", "...", "   "};
  display.print("starting");
  display.print(dots[(frame / 6) % 4]);

  display.display();
}

static void drawLoading(const Metrics& m, uint32_t t_ms) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Loading model window");

  // progress bar
  int w = (int)((SCREEN_WIDTH - 4) * (m.progress < 0 ? 0 : (m.progress > 1 ? 1 : m.progress)));
  display.drawRect(0, 18, SCREEN_WIDTH, 12, SSD1306_WHITE);
  display.fillRect(2, 20, w, 8, SSD1306_WHITE);

  // small spinner (rotating bar)
  int phase = (t_ms / 150) % 4;
  int cx = SCREEN_WIDTH - 12, cy = 6;
  display.drawCircle(cx, cy, 5, SSD1306_WHITE);
  if      (phase == 0) display.drawLine(cx, cy, cx, cy-4, SSD1306_WHITE);
  else if (phase == 1) display.drawLine(cx, cy, cx+4, cy, SSD1306_WHITE);
  else if (phase == 2) display.drawLine(cx, cy, cx, cy+4, SSD1306_WHITE);
  else                 display.drawLine(cx, cy, cx-4, cy, SSD1306_WHITE);

  display.setCursor(0, 40);
  display.printf("V:%5.1fV  I:%5.3fA\n", m.V, m.I);
  display.printf("P:%5.1fW  R:%s\n", m.P, m.relay ? "ON " : "OFF");

  display.display();
}

static void drawRun(const Metrics& m) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.printf("V:%6.1fV  I:%6.3fA\n", m.V, m.I);
  display.printf("P:%6.1fW  R:%s\n", m.P, m.relay ? "ON " : "OFF");
  display.printf("THEFT: %.2f\n", m.theft);

  // Theft bar
  int bw = (int)((SCREEN_WIDTH - 2) * (m.theft < 0 ? 0 : (m.theft > 1 ? 1 : m.theft)));
  display.drawRect(0, 40, SCREEN_WIDTH, 12, SSD1306_WHITE);
  display.fillRect(1, 41, bw, 10, SSD1306_WHITE);

  display.setCursor(0, 56);
  display.print("OK");

  display.display();
}

static void drawAlert(const Metrics& m, uint32_t t_ms) {
  // Blink/invert at ~3 Hz, moving hazard background + icon
  bool invert = ((t_ms / 160) % 2) == 0;
  display.clearDisplay();
  display.invertDisplay(invert);

  // diagonal-ish stripes
  for (int y=0; y<SCREEN_HEIGHT; y+=8) {
    int xoff = (t_ms / 40 + y) % 8;
    for (int x=-8; x<SCREEN_WIDTH; x+=16) {
      display.fillRect(x + xoff, y, 8, 8, SSD1306_WHITE);
    }
  }

  // center warning triangle + text box
  display.drawBitmap((SCREEN_WIDTH-24)/2, 4, kAlertTri_24x24, 24, 24, SSD1306_BLACK);
  display.fillRect(0, 36, SCREEN_WIDTH, 28, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(6, 40);
  display.print("THEFT DETECTED");
  display.setCursor(6, 52);
  display.printf("p=%.2f  P:%5.0fW", m.theft, m.P);

  display.invertDisplay(false); // reset for next frame
  display.display();
}

static void taskDisplay(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(100); // 10 FPS feels smooth without cost
  TickType_t tick = xTaskGetTickCount();
  uint16_t boot_frames = 0;

  for (;;) {
    uint32_t now = millis();
    Metrics m = get_metrics();

    if (m.phase == UI_BOOT && boot_frames < 20) {
      drawBootFrame(boot_frames++);
    } else if (m.phase == UI_LOADING) {
      drawLoading(m, now);
    } else if (m.phase == UI_ALERT) {
      drawAlert(m, now);
    } else {
      drawRun(m);
    }

    vTaskDelayUntil(&tick, period);
  }
}

void setup() {
  #if defined(ESP32)
    WiFi.mode(WIFI_OFF); btStop();
  #endif

  Serial.begin(115200); Serial.setTimeout(5); delay(120);

  pinMode(RELAY_PIN, OUTPUT); relayWrite(false);
  pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, LOW);

  // I2C bus
  Wire.begin(21,22);
  Wire.setClock(400000);

  // Mutex
  g_last_mtx = xSemaphoreCreateMutex();

  // INA219
  if (!ina.begin()) {
    Serial.println("ERR:INA219 not found @0x40");
  }
  // Choose calibration matching your shunt & range (R100: better resolution with 1A profile)
  ina.setCalibration_32V_1A();
  // If available, enable ADC averaging for smoother training/runtime values:
  // ina.setBusADC(INA219_BUS_ADC_12BIT_128S);
  // ina.setShuntADC(INA219_SHUNT_ADC_12BIT_128S);

  // OLED (auto-detect 0x3C/0x3D)
  OLED_ADDR = detectOLED(0x3C);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.printf("ERR:SSD1306 not found @0x%02X\n", OLED_ADDR);
  } else {
    display.clearDisplay();
    display.display();
  }

  // Seed UI in BOOT phase; ML will take over with LOADING → RUN/ALERT
  set_metrics({0,0,0,0,false,false, UI_BOOT, 0.0f});

  // Tasks (core 1): priorities — Serial(3) > ML(2) > Display(1)
  xTaskCreatePinnedToCore(taskSerial,  "Serial",  2048, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskML,      "ML",      8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskDisplay, "Display", 4096, nullptr, 1, nullptr, 1);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
