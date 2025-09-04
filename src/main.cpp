/*
 * ESP32 + INA219 + Active-HIGH Relay + Edge Impulse (TinyMLEtheftv1)
 * - Model expects 4 axes per sample: V, I, P, RELAY_ON(0/1)
 * - Uses EI_CLASSIFIER_FREQUENCY and EI_CLASSIFIER_RAW_SAMPLE_COUNT from your EI lib
 * - Smoothing: EWMA + majority vote
 *
 * Libraries required:
 *  - Adafruit INA219
 *  - Your EI Arduino library (TinyMLEtheftv1_inferencing.zip added via Sketch > Include Library > Add .ZIP Library)
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyMLEtheftv1_inferencing.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include "esp_bt.h"
#endif

// -------- Pins / Relay --------
const int  RELAY_PIN         = 16;    // IN pin on your relay module
const bool RELAY_ACTIVE_LOW  = false; // ACTIVE-HIGH relay (ON when pin is HIGH)
const int  LED_ALERT_PIN     = 2;     // onboard LED for alert (or change to any GPIO)
// const int  BUZZER_PIN     = 4;     // optional buzzer pin

// -------- INA219 --------
Adafruit_INA219 ina219;

// -------- Edge Impulse input buffer --------
// Interleaved per-sample layout with 4 axes: V,I,P,R  V,I,P,R  ...
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// -------- Smoothing / thresholds --------
float      theft_prob_ewma   = 0.0f;
const float EWMA_ALPHA       = 0.50f; // 0..1 (higher reacts faster)
const float THEFT_THRESHOLD  = 0.60f; // smoothed theft prob to alert

const int   VOTE_N           = 3;     // majority over last N windows
int         vote_hist_idx    = 0;
int         vote_hist[VOTE_N] = {0};

// -------- Helpers --------
void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}
bool relayIsOn() {
  int v = digitalRead(RELAY_PIN);
  return RELAY_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// EI signal getter
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

void setup() {
  // Free RAM (no Wi-Fi/Bluetooth needed for this demo)
  #if defined(ESP32)
    WiFi.mode(WIFI_OFF);
    btStop();
  #endif

  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);                 // force OFF (open) at boot for ACTIVE-HIGH
  pinMode(LED_ALERT_PIN, OUTPUT);
  digitalWrite(LED_ALERT_PIN, LOW);
  // pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // I2C @ 400 kHz for faster reads (keep wires short)
  Wire.begin(21, 22);                // SDA, SCL (ESP32 defaults)
  Wire.setClock(400000);

  if (!ina219.begin()) {
    Serial.println("ERROR: INA219 not found. Check wiring/address.");
    while (1) delay(500);
  }
  // Choose calibration for your expected current range
  ina219.setCalibration_32V_2A();    // good default (5V system, up to ~2A)

  // Sanity info from EI model
  Serial.printf("EI: freq=%.1f Hz  samples=%d  axes=%d  frame=%d\n",
    EI_CLASSIFIER_FREQUENCY,
    EI_CLASSIFIER_RAW_SAMPLE_COUNT,
    EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME,
    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 4) {
    Serial.println("WARNING: Model not built for 4 axes (V,I,P,Relay). Check impulse inputs or adjust fill order.");
  }
}

void loop() {
  // -------- Collect one full window at the model's frequency --------
  const uint32_t interval_ms = (uint32_t)roundf(1000.0f / EI_CLASSIFIER_FREQUENCY);
  uint32_t next_tick = millis();

  size_t ix = 0;
  for (size_t n = 0; n < EI_CLASSIFIER_RAW_SAMPLE_COUNT; n++) {
    // keep timing steady
    uint32_t now = millis();
    if (now < next_tick) delay(next_tick - now);
    next_tick += interval_ms;

    // Read sensors
    float V = ina219.getBusVoltage_V();            // volts
    float I = ina219.getCurrent_mA() / 1000.0f;    // amps
    float P = V * I;                                // watts
    float R = relayIsOn() ? 1.0f : 0.0f;           // relay state axis

    // Interleave exactly as model expects: V, I, P, R
    features[ix++] = V;
    features[ix++] = I;
    features[ix++] = P;
    features[ix++] = R;
  }

  // -------- Run classifier --------
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data     = &raw_feature_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("run_classifier failed (%d)\n", err);
    return;
  }

  // -------- Parse class probabilities (use macro count for this EI lib) --------
  float p_normal_off = 0, p_normal_on = 0, p_theft_off = 0, p_theft_on = 0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char* lab = result.classification[i].label;
    float val = result.classification[i].value;
    if      (strcmp(lab, "normal_off") == 0) p_normal_off = val;
    else if (strcmp(lab, "normal_on")  == 0) p_normal_on  = val;
    else if (strcmp(lab, "theft_off")  == 0) p_theft_off  = val;
    else if (strcmp(lab, "theft_on")   == 0) p_theft_on   = val;
  }
  float theft_prob = p_theft_off + p_theft_on;

  // -------- Smooth & decide --------
  theft_prob_ewma = EWMA_ALPHA * theft_prob + (1.0f - EWMA_ALPHA) * theft_prob_ewma;

  // Majority vote over last N windows
  vote_hist[vote_hist_idx] = (theft_prob >= 0.50f) ? 1 : 0;
  vote_hist_idx = (vote_hist_idx + 1) % VOTE_N;
  int votes = 0; for (int k = 0; k < VOTE_N; k++) votes += vote_hist[k];
  bool vote_theft = (votes >= (VOTE_N + 1) / 2);

  bool alert = (theft_prob_ewma >= THEFT_THRESHOLD) || vote_theft;

  // -------- Actuate --------
  digitalWrite(LED_ALERT_PIN, alert ? HIGH : LOW);
  // digitalWrite(BUZZER_PIN, alert ? HIGH : LOW);

  // -------- Debug --------
  Serial.printf("NOFF=%.2f NON=%.2f TOFF=%.2f TON=%.2f | theft=%.2f ewma=%.2f vote=%d/%d -> %s\n",
    p_normal_off, p_normal_on, p_theft_off, p_theft_on,
    theft_prob, theft_prob_ewma, votes, VOTE_N, alert ? "ALERT" : "ok");
}
