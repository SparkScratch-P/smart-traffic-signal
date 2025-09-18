/* SmartTraffic_Final_Fixed.ino
   - 2-lane ESP32 traffic controller
   - Non-blocking, ring-history, percentile normalization (1%/99%)
   - EWMA alpha = 0.2
   - Dynamic override (mean + dynamicK * stddev) with hysteresis (>=3)
   - Input sanitization, watchdog, non-blocking fallback, failsafe relay pin
*/
#define BLYNK_TEMPLATE_ID "TMPL382sW9WQQ"
#define BLYNK_TEMPLATE_NAME "PID Control Tuning"
/*****************************************************
 * SmartTraffic_Final_Fixed_WithMandatoryYellow.ino
 * Final full code — preserves all features and fixes,
 * and ensures BOTH lanes always pass through YELLOW (non-blocking).
 *
 * Features preserved:
 *  - Non-blocking state machine (millis)
 *  - GPIO control for 2 lanes (R/Y/G each)
 *  - Separate ML / Car ring-history buffers
 *  - Percentile normalization (1% / 99%)
 *  - EWMA smoothing (alpha = 0.2)
 *  - Weighted distribution (30% ML, 70% Cars)
 *  - Min / Max green enforcement
 *  - Safe fallback schedule when inputs stale
 *  - Dynamic override detection with hysteresis (>=3)
 *  - Input sanitization
 *  - Blynk integration, V1..V4 inputs, V5 status (L1L2 as two-digit number)
 *  - Watchdog (ESP-IDF v5.x API)
 *  - Hardware failsafe assumed (relay), code leaves FAILSAFE_PIN placeholder
 *
 * Replace SSID / PASS / auth with your credentials before flashing.
 *****************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "esp_task_wdt.h"

// ---------------- CONFIG - set these ----------------
char auth[] = "LpTIYa43VENKHRK0Y0aIJsPDLL5Uu1qq";
char ssid[] = "Prodyumna";
char pass[] = "pro12345";

// ---------- GPIO pin definitions ----------
const int LANE1_RED    = 14;
const int LANE1_YELLOW = 27;
const int LANE1_GREEN  = 26;
const int LANE2_RED    = 25;
const int LANE2_YELLOW = 33;
const int LANE2_GREEN  = 32;

// Optional failsafe relay pin (energize to allow normal operation)
const int FAILSAFE_PIN = 4;

// ---------- Parameters ----------
const int CYCLE_LENGTH_SEC   = 120;
const int YELLOW_SEC         = 3;
const int MIN_GREEN_SEC      = 10;
const int MAX_GREEN_SEC      = 90;
const float ML_WEIGHT       = 0.30f;
const float CAR_WEIGHT      = 0.70f;
const float EWMA_ALPHA      = 0.20f;
const int HIST_SIZE         = 100;
const int MIN_HIST_SAMPLES  = 8;
const int OVERRIDE_CONFIRM_N= 3;
const unsigned long INPUT_STALE_MS = 15000UL; // fallback if no inputs
const int WDT_TIMEOUT_SEC   = 10;

// ---------- Global data ----------
std::vector<int> mlHist1, mlHist2, carHist1, carHist2;
int mlRaw[2]  = {0,0};
int carRaw[2] = {0,0};
float mlEwma[2] = {0.0f, 0.0f};
float carEwma[2] = {0.0f, 0.0f};
unsigned long lastInputMs = 0UL;

// override hysteresis counter (per lane)
int overrideCnt[2] = {0,0};

// computed green durations (seconds)
int laneGreenSec[2] = {30, 30};

// ---------- State machine ----------
enum State { STATE_L1_GREEN, STATE_L1_YELLOW, STATE_L2_GREEN, STATE_L2_YELLOW };
State currentState = STATE_L1_GREEN;
unsigned long stateStartMs = 0UL;

// lane numeric states for Blynk reporting
// mapping: RED=1, GREEN=2, YELLOW=3
#define LS_RED   1
#define LS_GREEN 2
#define LS_YELLOW 3
int laneStateVal[2] = {LS_RED, LS_RED};
int prevLaneStateVal[2] = {0,0}; // to detect changes and send Blynk

// ---------- Utility functions ----------
void addToHist(std::vector<int> &hist, int val) {
  hist.push_back(val);
  if ((int)hist.size() > HIST_SIZE) hist.erase(hist.begin());
}

std::vector<int> getLastN(const std::vector<int> &hist, int n) {
  int c = (int)hist.size();
  if (n <= 0 || c == 0) return {};
  n = min(n, c);
  std::vector<int> out;
  out.reserve(n);
  int start = c - n;
  for (int i = 0; i < n; ++i) out.push_back(hist[start + i]);
  return out;
}

float percentileFromVec(const std::vector<int> &hist, float p) {
  if (hist.empty()) return 0.0f;
  std::vector<int> tmp = hist;
  std::sort(tmp.begin(), tmp.end());
  int idx = (int)floor(p * (tmp.size() - 1));
  idx = max(0, min((int)tmp.size()-1, idx));
  return (float)tmp[idx];
}

// normalize x to 0..100 using 1%/99% percentiles from history
int normalizeScore(float x, const std::vector<int> &hist) {
  if ((int)hist.size() < MIN_HIST_SAMPLES) return 50;
  float minV = percentileFromVec(hist, 0.01f);
  float maxV = percentileFromVec(hist, 0.99f);
  if (fabsf(maxV - minV) < 1e-6f) return 50;
  float xf = x;
  if (xf < minV) xf = minV;
  if (xf > maxV) xf = maxV;
  float n = (xf - minV) / (maxV - minV);
  return (int)roundf(n * 100.0f);
}

float ewmaUpdate(float prev, float x) {
  // initialize prev to x if prev is 0 and history empty might be misleading,
  // but we call update after pushing data; it's acceptable.
  return EWMA_ALPHA * x + (1.0f - EWMA_ALPHA) * prev;
}

// ---------- Blynk status update ----------
void updateBlynkLaneStatusIfChanged() {
  // combined status: L1*10 + L2 (e.g., 12)
  if (laneStateVal[0] != prevLaneStateVal[0] || laneStateVal[1] != prevLaneStateVal[1]) {
    int combined = laneStateVal[0] * 10 + laneStateVal[1];
    Blynk.virtualWrite(V5, combined);
    prevLaneStateVal[0] = laneStateVal[0];
    prevLaneStateVal[1] = laneStateVal[1];
  }
}

// ---------- Override detection (dynamic kFactor & hysteresis) ----------
bool overrideCandidate() {
  if ((int)carHist1.size() < MIN_HIST_SAMPLES || (int)carHist2.size() < MIN_HIST_SAMPLES) return false;
  int N = min((int)carHist1.size(), (int)carHist2.size());
  std::vector<float> diffs; diffs.reserve(N);
  for (int i = 0; i < N; ++i) {
    int idx1 = (int)carHist1.size() - N + i;
    int idx2 = (int)carHist2.size() - N + i;
    float d = fabsf((float)carHist1[idx1] - (float)carHist2[idx2]);
    diffs.push_back(d);
  }
  float mean = 0.0f;
  for (float v : diffs) mean += v;
  mean /= (float)N;
  float ss = 0.0f;
  for (float v : diffs) ss += (v - mean) * (v - mean);
  float stddev = sqrtf(ss / (float)N);

  float currAbs = fabsf(carEwma[0] - carEwma[1]);

  float denom = max(mean, 1.0f);
  float dynamicK = 1.5f + (stddev / denom); // adaptive K (>=1.5)
  // threshold using mean + dynamicK*stddev
  float threshold = mean + dynamicK * stddev;

  return currAbs > threshold;
}

// returns -1 if none, or lane index 0/1 when confirmed
int checkOverrideHysteresis() {
  if (!overrideCandidate()) {
    // decay counters
    if (overrideCnt[0] > 0) overrideCnt[0]--;
    if (overrideCnt[1] > 0) overrideCnt[1]--;
    return -1;
  }
  // candidate present; choose lane with higher EWMA car count
  if (carEwma[0] > carEwma[1]) {
    overrideCnt[0]++;
    overrideCnt[1] = 0;
    if (overrideCnt[0] >= OVERRIDE_CONFIRM_N) return 0;
  } else {
    overrideCnt[1]++;
    overrideCnt[0] = 0;
    if (overrideCnt[1] >= OVERRIDE_CONFIRM_N) return 1;
  }
  return -1;
}

// ---------- Compute lane distribution ----------
void computeLaneDistributionSec(int &g0_sec, int &g1_sec) {
  int nML0  = normalizeScore(mlEwma[0], mlHist1);
  int nML1  = normalizeScore(mlEwma[1], mlHist2);
  int nCar0 = normalizeScore(carEwma[0], carHist1);
  int nCar1 = normalizeScore(carEwma[1], carHist2);

  // apply EWMA smoothing on normalized values (we already updated raw ewma; this further smooths)
  mlEwma[0] = ewmaUpdate(mlEwma[0], (float)nML0);
  mlEwma[1] = ewmaUpdate(mlEwma[1], (float)nML1);
  carEwma[0] = ewmaUpdate(carEwma[0], (float)nCar0);
  carEwma[1] = ewmaUpdate(carEwma[1], (float)nCar1);

  float mlShare0 = 50.0f, carShare0 = 50.0f;
  if ((mlEwma[0] + mlEwma[1]) > 0.0f) mlShare0 = (mlEwma[0] * 100.0f) / (mlEwma[0] + mlEwma[1]);
  if ((carEwma[0] + carEwma[1]) > 0.0f) carShare0 = (carEwma[0] * 100.0f) / (carEwma[0] + carEwma[1]);

  float finalShare0 = ML_WEIGHT * mlShare0 + CAR_WEIGHT * carShare0;
  float finalShare1 = 100.0f - finalShare0;

  int available = CYCLE_LENGTH_SEC - 2 * YELLOW_SEC;
  if (available < 2 * MIN_GREEN_SEC) {
    g0_sec = max(MIN_GREEN_SEC, available / 2);
    g1_sec = max(MIN_GREEN_SEC, available - g0_sec);
    return;
  }

  g0_sec = constrain((int)roundf((finalShare0 / 100.0f) * available), MIN_GREEN_SEC, MAX_GREEN_SEC);
  g1_sec = constrain((int)roundf((finalShare1 / 100.0f) * available), MIN_GREEN_SEC, MAX_GREEN_SEC);

  // balance if exceed available
  if (g0_sec + g1_sec > available) {
    float scale = (float)available / (float)(g0_sec + g1_sec);
    g0_sec = max(MIN_GREEN_SEC, (int)floor(g0_sec * scale));
    g1_sec = max(MIN_GREEN_SEC, (int)floor(g1_sec * scale));
  }

  // apply override if confirmed
  int ov = checkOverrideHysteresis();
  if (ov >= 0) {
    if (ov == 0) { g0_sec = available - MIN_GREEN_SEC; g1_sec = MIN_GREEN_SEC; }
    else         { g1_sec = available - MIN_GREEN_SEC; g0_sec = MIN_GREEN_SEC; }
  }
}

// ---------- Hardware control helpers ----------
void setAllOff() {
  digitalWrite(LANE1_RED, LOW); digitalWrite(LANE1_YELLOW, LOW); digitalWrite(LANE1_GREEN, LOW);
  digitalWrite(LANE2_RED, LOW); digitalWrite(LANE2_YELLOW, LOW); digitalWrite(LANE2_GREEN, LOW);
}

void setLaneGreen(int lane) {
  if (lane == 0) {
    digitalWrite(LANE1_RED, LOW); digitalWrite(LANE1_YELLOW, LOW); digitalWrite(LANE1_GREEN, HIGH);
    digitalWrite(LANE2_RED, HIGH); digitalWrite(LANE2_YELLOW, LOW); digitalWrite(LANE2_GREEN, LOW);
    laneStateVal[0] = LS_GREEN; laneStateVal[1] = LS_RED;
  } else {
    digitalWrite(LANE2_RED, LOW); digitalWrite(LANE2_YELLOW, LOW); digitalWrite(LANE2_GREEN, HIGH);
    digitalWrite(LANE1_RED, HIGH); digitalWrite(LANE1_YELLOW, LOW); digitalWrite(LANE1_GREEN, LOW);
    laneStateVal[1] = LS_GREEN; laneStateVal[0] = LS_RED;
  }
}

void setLaneYellow(int lane) {
  if (lane == 0) {
    digitalWrite(LANE1_RED, LOW); digitalWrite(LANE1_YELLOW, HIGH); digitalWrite(LANE1_GREEN, LOW);
    digitalWrite(LANE2_RED, HIGH); digitalWrite(LANE2_YELLOW, LOW); digitalWrite(LANE2_GREEN, LOW);
    laneStateVal[0] = LS_YELLOW; laneStateVal[1] = LS_RED;
  } else {
    digitalWrite(LANE2_RED, LOW); digitalWrite(LANE2_YELLOW, HIGH); digitalWrite(LANE2_GREEN, LOW);
    digitalWrite(LANE1_RED, HIGH); digitalWrite(LANE1_YELLOW, LOW); digitalWrite(LANE1_GREEN, LOW);
    laneStateVal[1] = LS_YELLOW; laneStateVal[0] = LS_RED;
  }
}

// ---------- Blynk status update ----------
void updateBlynkLaneStatus() {
  int combined = laneStateVal[0] * 10 + laneStateVal[1];
  Blynk.virtualWrite(V5, combined);
}

// ---------- Blynk handlers (inputs sanitized) ----------
BLYNK_WRITE(V1) { int v = constrain(param.asInt(), 0, 1000000); mlRaw[0] = v; addToHist(mlHist1, v); mlEwma[0] = ewmaUpdate(mlEwma[0], (float)v); lastInputMs = millis(); }
BLYNK_WRITE(V2) { int v = constrain(param.asInt(), 0, 1000000); mlRaw[1] = v; addToHist(mlHist2, v); mlEwma[1] = ewmaUpdate(mlEwma[1], (float)v); lastInputMs = millis(); }
BLYNK_WRITE(V3) { int v = constrain(param.asInt(), 0, 1000000); carRaw[0] = v; addToHist(carHist1, v); carEwma[0] = ewmaUpdate(carEwma[0], (float)v); lastInputMs = millis(); }
BLYNK_WRITE(V4) { int v = constrain(param.asInt(), 0, 1000000); carRaw[1] = v; addToHist(carHist2, v); carEwma[1] = ewmaUpdate(carEwma[1], (float)v); lastInputMs = millis(); }

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // init pins and set LOW
  pinMode(LANE1_RED, OUTPUT); pinMode(LANE1_YELLOW, OUTPUT); pinMode(LANE1_GREEN, OUTPUT);
  pinMode(LANE2_RED, OUTPUT); pinMode(LANE2_YELLOW, OUTPUT); pinMode(LANE2_GREEN, OUTPUT);
  pinMode(FAILSAFE_PIN, OUTPUT);

  setAllOff();
  // energize failsafe relay so normal operation allowed (hardware should default to ALL-RED if relay de-energized)
  digitalWrite(FAILSAFE_PIN, HIGH);

  // start Blynk
  Blynk.begin(auth, ssid, pass);

  // watchdog config for ESP-IDF v5.x
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms = (uint32_t)WDT_TIMEOUT_SEC * 1000U,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_cfg);
  esp_task_wdt_add(NULL);

  // initial compute
  computeLaneDistributionSec(laneGreenSec[0], laneGreenSec[1]);
  // start with lane1 green, lane2 red
  currentState = STATE_L1_GREEN;
  stateStartMs = millis();
  setLaneGreen(0);
  updateBlynkLaneStatus(); // initial push
  Serial.println("SmartTraffic Final (with mandatory yellow) started.");
}

// ---------- Non-blocking state machine (mandatory Yellow for both lanes) ----------
void loop() {
  Blynk.run();
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Fallback mode: if inputs stale, use safe fixed split (non-blocking)
  if (lastInputMs == 0UL || (now - lastInputMs) > INPUT_STALE_MS) {
    int avail = CYCLE_LENGTH_SEC - 2 * YELLOW_SEC;
    laneGreenSec[0] = max(MIN_GREEN_SEC, avail / 2);
    laneGreenSec[1] = max(MIN_GREEN_SEC, avail - laneGreenSec[0]);
  } else {
    // normally recompute at end of each full cycle (we keep compute when entering L1_GREEN)
    // but recompute proactively if desired (commented to avoid too frequent recompute):
    // computeLaneDistributionSec(laneGreenSec[0], laneGreenSec[1]);
    ;
  }

  unsigned long elapsed = now - stateStartMs;

  switch (currentState) {
    case STATE_L1_GREEN:
      // ensure lane state variables and Blynk reflect green/red
      laneStateVal[0] = LS_GREEN; laneStateVal[1] = LS_RED;
      updateBlynkLaneStatusIfChanged();

      if (elapsed >= (unsigned long)laneGreenSec[0] * 1000UL) {
        // go to L1 yellow (mandatory)
        currentState = STATE_L1_YELLOW;
        stateStartMs = now;
        setLaneYellow(0);
        updateBlynkLaneStatus();
      }
      break;

    case STATE_L1_YELLOW:
      // during yellow: both lanes have lane1 yellow, lane2 red
      if (elapsed >= (unsigned long)YELLOW_SEC * 1000UL) {
        // after yellow -> lane2 green
        currentState = STATE_L2_GREEN;
        stateStartMs = now;
        setLaneGreen(1);
        updateBlynkLaneStatus();
      }
      break;

    case STATE_L2_GREEN:
      laneStateVal[1] = LS_GREEN; laneStateVal[0] = LS_RED;
      updateBlynkLaneStatusIfChanged();

      if (elapsed >= (unsigned long)laneGreenSec[1] * 1000UL) {
        // mandatory yellow for lane2
        currentState = STATE_L2_YELLOW;
        stateStartMs = now;
        setLaneYellow(1);
        updateBlynkLaneStatus();
      }
      break;

    case STATE_L2_YELLOW:
      if (elapsed >= (unsigned long)YELLOW_SEC * 1000UL) {
        // after yellow -> recompute distribution for next cycle and go to L1 green
        computeLaneDistributionSec(laneGreenSec[0], laneGreenSec[1]);
        currentState = STATE_L1_GREEN;
        stateStartMs = now;
        setLaneGreen(0);
        updateBlynkLaneStatus();
      }
      break;
  }

  // Periodically update EWMA even when inputs come via Blynk handlers (if your inputs arrive irregularly)
  // (Not strictly necessary here since we update EWMA when inputs arrive via handlers)
  // Small cooperative yield
  delay(5);
}
