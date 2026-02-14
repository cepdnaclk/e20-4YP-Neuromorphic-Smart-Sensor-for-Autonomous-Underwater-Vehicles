// =========================================
// UNDERWATER STEP3 (POOL-STABLE) — 1 SENSOR
// Baseline learned in OPEN water, danger when distance drops significantly.
// Output:
// time_ms,echo_us,valid,dist_cm,dist_f_cm,baseline_cm,thr_enter_cm,thr_exit_cm,danger,event
// event: +1 entered danger, -1 exited danger, 0 none
// =========================================

#define TRIG_PIN 5
#define ECHO_PIN 18

// ---- Timing ----
const int   SAMPLE_PERIOD_MS = 200;
const int   TRIG_HIGH_US     = 25;
const long  TIMEOUT_US       = 60000;

// ---- Echo validity window (underwater tends to be noisier) ----
const long  ECHO_MIN_US      = 250;
const long  ECHO_MAX_US      = 30000;

// ---- Median-of-N pings (reduces jitter) ----
const int   NPINGS           = 7;
const int   INTERPING_MS     = 35;

// ---- Water speed conversion ----
// JSN-SR04T uses round-trip time; one-way distance = (echo_us * cm/us)/2
// You were using 0.148 cm/us (approx speed of sound in water ~1480 m/s).
const float CM_PER_US_ONEWAY = 0.148f;

// Sensor spec range (clamp for sanity)
const float MIN_DIST_CM      = 20.0f;
const float MAX_DIST_CM      = 600.0f;

// ---- Filtering ----
const float DIST_EMA_A       = 0.25f;     // EMA alpha

// ---- Baseline calibration (OPEN water) ----
const int   BASELINE_SAMPLES = 25;        // ~25 * 200ms = 5s

// ---- Danger definition (relative to baseline) ----
// ENTER when dist_f <= baseline - MARGIN_CM (closer than "safe open water" by margin)
// EXIT when dist_f >= (baseline - MARGIN_CM) + HYST_CM
const float MARGIN_CM        = 60.0f;
const float HYST_CM          = 20.0f;

// ---- Confirmation window (debounce state transitions) ----
const int   CONF_N           = 3;         // window size
const int   ENTER_K          = 2;         // need >=2/3 below enter threshold
const int   EXIT_K           = 2;         // need >=2/3 above exit threshold

// ---- Outlier reject (anti-multipath) ----
// IMPORTANT: in your logs, 0.40 caused the filter to "freeze".
// Use 0.30 + "2-strike" logic to ignore one spike but accept a real obstacle.
const float DROP_REJECT_FRAC = 0.30f;

// ---- State ----
bool  dist_init   = false;
float dist_f_cm   = 0.0f;

bool  baseline_ready = false;
float baseline_cm = 0.0f;
int   base_n      = 0;
float base_buf[BASELINE_SAMPLES];

bool  in_danger   = false;

// Confirmation history (store filtered distances)
float hist[CONF_N];
int   hist_i = 0;
int   hist_count = 0;

// 2-strike drop handling
int drop_strikes = 0;

// ---------- Helpers ----------
float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

void sortArrayFloat(float *a, int n) {
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (a[j] < a[i]) { float t = a[i]; a[i] = a[j]; a[j] = t; }
}

void sortArrayLong(long *a, int n) {
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (a[j] < a[i]) { long t = a[i]; a[i] = a[j]; a[j] = t; }
}

long readEchoOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_HIGH_US);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
}

long readEchoMedian() {
  long vals[NPINGS];
  int k = 0;
  for (int i = 0; i < NPINGS; i++) {
    long e = readEchoOnce();
    if (e >= ECHO_MIN_US && e <= ECHO_MAX_US) vals[k++] = e;
    delay(INTERPING_MS);
  }
  if (k == 0) return 0;
  sortArrayLong(vals, k);
  return vals[k / 2];
}

float echoToDistCm(long echo_us) {
  return (echo_us * CM_PER_US_ONEWAY) / 2.0f;
}

void pushHist(float x) {
  hist[hist_i] = x;
  hist_i = (hist_i + 1) % CONF_N;
  if (hist_count < CONF_N) hist_count++;
}

int countBelow(float thr) {
  int c = 0;
  for (int i = 0; i < hist_count; i++) if (hist[i] <= thr) c++;
  return c;
}

int countAbove(float thr) {
  int c = 0;
  for (int i = 0; i < hist_count; i++) if (hist[i] >= thr) c++;
  return c;
}

float medianOfBuffer(float *buf, int n) {
  float tmp[BASELINE_SAMPLES];
  for (int i = 0; i < n; i++) tmp[i] = buf[i];
  sortArrayFloat(tmp, n);
  return tmp[n / 2];
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  // ESP32 supports INPUT_PULLDOWN; if your board doesn’t, switch to INPUT.
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  delay(500);

  Serial.println("UNDERWATER STEP3 (POOL-STABLE)");
  Serial.println("Format: time_ms,echo_us,valid,dist_cm,dist_f_cm,baseline_cm,thr_enter_cm,thr_exit_cm,danger,event");
  Serial.println("Calibration: point into OPEN water for ~5 seconds to learn baseline.");
}

void loop() {
  unsigned long t = millis();

  long echo_us = readEchoMedian();
  bool valid = (echo_us != 0);

  float dist_cm = 0.0f;
  float danger  = 0.0f;
  int   event   = 0;

  // thresholds (only meaningful after baseline ready)
  float thr_enter = 0.0f;
  float thr_exit  = 0.0f;

  if (valid) {
    dist_cm = clampf(echoToDistCm(echo_us), MIN_DIST_CM, MAX_DIST_CM);

    // --- Filtering with 2-strike drop handling ---
    if (!dist_init) {
      dist_f_cm = dist_cm;
      dist_init = true;
      drop_strikes = 0;
    } else {
      float reject_limit = dist_f_cm * (1.0f - DROP_REJECT_FRAC);

      if (dist_cm < reject_limit) {
        drop_strikes++;
        if (drop_strikes >= 2) {
          // repeated drop => accept (real obstacle)
          dist_f_cm = DIST_EMA_A * dist_cm + (1.0f - DIST_EMA_A) * dist_f_cm;
          drop_strikes = 2; // cap
        } else {
          // first strike => ignore (likely multipath)
          // keep dist_f_cm unchanged
        }
      } else {
        // normal reading => update + reset strikes
        drop_strikes = 0;
        dist_f_cm = DIST_EMA_A * dist_cm + (1.0f - DIST_EMA_A) * dist_f_cm;
      }
    }

    // --- Baseline learning (OPEN water) ---
    if (!baseline_ready) {
      // collect filtered distances
      base_buf[base_n++] = dist_f_cm;

      if (base_n >= BASELINE_SAMPLES) {
        baseline_cm = medianOfBuffer(base_buf, BASELINE_SAMPLES);
        baseline_ready = true;
        Serial.print("BASELINE learned (median): ");
        Serial.print(baseline_cm, 1);
        Serial.println(" cm");

        // reset danger state cleanly after calibration
        in_danger = false;
        hist_count = 0;
        hist_i = 0;
        drop_strikes = 0;
      }
    }

    // --- Danger detection (only after baseline is ready) ---
    if (baseline_ready) {
      thr_enter = baseline_cm - MARGIN_CM;
      thr_exit  = thr_enter + HYST_CM;

      // store history for confirmation-based transitions
      pushHist(dist_f_cm);

      if (!in_danger) {
        if (hist_count == CONF_N && countBelow(thr_enter) >= ENTER_K) {
          in_danger = true;
          event = +1;
        }
      } else {
        if (hist_count == CONF_N && countAbove(thr_exit) >= EXIT_K) {
          in_danger = false;
          event = -1;
        }
      }

      // --- danger scalar (0..1) ---
      // in danger: 1 near/under thr_enter, decreases as you move away toward thr_exit+SLOPE
      if (in_danger) {
        float far = thr_exit + 120.0f;  // slope width (tune)
        danger = (dist_f_cm <= thr_enter) ? 1.0f :
                 (dist_f_cm >= far) ? 0.0f :
                 1.0f - (dist_f_cm - thr_enter) / (far - thr_enter);
      } else {
        danger = 0.0f;
      }
    } else {
      // not calibrated yet => keep danger 0
      danger = 0.0f;
    }

  } else {
    // invalid echo: keep previous dist_f_cm and state; do not force danger
    // (If you want "invalid=stop", we can change this behavior.)
  }

  // ---------- Print ----------
  Serial.print(t); Serial.print(",");
  Serial.print(echo_us); Serial.print(",");
  Serial.print(valid ? 1 : 0); Serial.print(",");
  Serial.print(valid ? dist_cm : 0.0f, 1); Serial.print(",");
  Serial.print(dist_f_cm, 1); Serial.print(",");

  if (baseline_ready) {
    thr_enter = baseline_cm - MARGIN_CM;
    thr_exit  = thr_enter + HYST_CM;
    Serial.print(baseline_cm, 1); Serial.print(",");
    Serial.print(thr_enter, 1);   Serial.print(",");
    Serial.print(thr_exit, 1);    Serial.print(",");
  } else {
    Serial.print("0.0,0.0,0.0,");
  }

  Serial.print(danger, 3); Serial.print(",");
  Serial.println(event);

  delay(SAMPLE_PERIOD_MS);
}

