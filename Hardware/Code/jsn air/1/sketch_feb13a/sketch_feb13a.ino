#define TRIG_PIN 5
#define ECHO_PIN 18

// ===== AIR baseline learning settings =====
const int   SAMPLE_PERIOD_MS = 200;   // 5 Hz
const int   TRIG_HIGH_US     = 25;
const long  TIMEOUT_US       = 60000;

// Valid ranges for air
const long  ECHO_MIN_US_AIR  = 80;
const long  ECHO_MAX_US_AIR  = 40000;
const float DIST_MIN_CM_AIR  = 2.0;
const float DIST_MAX_CM_AIR  = 800.0;

// Baseline learning (safe background)
const int BASELINE_SAMPLES = 25;      // 25 samples * 200ms ≈ 5 seconds
float dist_buf[BASELINE_SAMPLES];
int   buf_n = 0;
bool  baseline_ready = false;
float D_base = 0.0f;

// Danger margins relative to baseline
const float MARGIN_WARN_CM = 30.0f;   // closer than (baseline - 30) => start danger
const float MARGIN_STOP_CM = 60.0f;   // closer than (baseline - 60) => full danger

// Smoothing
float danger_smoothed = 0.0f;
const float OUT_ALPHA = 0.35f;

// Stuck diagnostic
long last_echo = -1;
int  same_count = 0;
const long STUCK_EPS_US = 5;
const int  STUCK_LIMIT  = 12;

long readEchoUs() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_HIGH_US);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
}

float airDistanceCmFromUs(long echo_us) {
  return (echo_us * 0.0343f) / 2.0f;
}

float clamp01(float x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

// Simple insertion sort to compute median (small N)
float median(float *a, int n) {
  float tmp[BASELINE_SAMPLES];
  for (int i = 0; i < n; i++) tmp[i] = a[i];

  for (int i = 1; i < n; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }

  if (n % 2 == 1) return tmp[n / 2];
  return 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  delay(1000);

  Serial.println("AIR MODE: Learn SAFE baseline, then compute danger");
  Serial.println("Format: time_ms,echo_us,dist_cm,meas_valid,stuck,baseline_cm,danger");
  Serial.println("Calibration: keep sensor facing OPEN space for ~5 seconds...");
}

void loop() {
  unsigned long time_ms = millis();
  long echo_us = readEchoUs();

  // stuck diagnostic
  if (last_echo > 0 && labs(echo_us - last_echo) <= STUCK_EPS_US) same_count++;
  else same_count = 0;
  last_echo = echo_us;
  bool stuck = (same_count >= STUCK_LIMIT);

  // measurement validity
  bool meas_valid = (echo_us >= ECHO_MIN_US_AIR && echo_us <= ECHO_MAX_US_AIR);

  float dist_cm = -1.0f;
  if (meas_valid) {
    dist_cm = airDistanceCmFromUs(echo_us);
    if (dist_cm < DIST_MIN_CM_AIR || dist_cm > DIST_MAX_CM_AIR) meas_valid = false;
  }

  // 1) Baseline learning phase
  if (!baseline_ready) {
    if (meas_valid) {
      dist_buf[buf_n++] = dist_cm;
      if (buf_n >= BASELINE_SAMPLES) {
        D_base = median(dist_buf, BASELINE_SAMPLES);
        baseline_ready = true;
        Serial.print("SAFE baseline learned (median): ");
        Serial.print(D_base, 1);
        Serial.println(" cm");
      }
    }

    Serial.print(time_ms); Serial.print(",");
    Serial.print(echo_us); Serial.print(",");
    Serial.print(dist_cm, 1); Serial.print(",");
    Serial.print(meas_valid ? 1 : 0); Serial.print(",");
    Serial.print(stuck ? 1 : 0); Serial.print(",");
    Serial.print(0.0, 1); Serial.print(",");
    Serial.println(0.000, 3);

    delay(SAMPLE_PERIOD_MS);
    return;
  }

  // 2) Danger relative to baseline
  float danger = 0.0f;

  if (meas_valid) {
    float warn_dist = D_base - MARGIN_WARN_CM; // above this -> safe
    float stop_dist = D_base - MARGIN_STOP_CM; // below this -> danger=1

    // If baseline is small (e.g., wall already close), clamp
    if (warn_dist < 0) warn_dist = 0;
    if (stop_dist < 0) stop_dist = 0;

    if (dist_cm >= warn_dist) danger = 0.0f;
    else if (dist_cm <= stop_dist) danger = 1.0f;
    else {
      // linear ramp between warn and stop
      danger = (warn_dist - dist_cm) / (warn_dist - stop_dist);
    }
  } else {
    // invalid -> keep previous, decay slowly
    danger = danger_smoothed;
  }

  danger_smoothed = OUT_ALPHA * danger + (1.0f - OUT_ALPHA) * danger_smoothed;

  Serial.print(time_ms); Serial.print(",");
  Serial.print(echo_us); Serial.print(",");
  Serial.print(dist_cm, 1); Serial.print(",");
  Serial.print(meas_valid ? 1 : 0); Serial.print(",");
  Serial.print(stuck ? 1 : 0); Serial.print(",");
  Serial.print(D_base, 1); Serial.print(",");
  Serial.println(danger_smoothed, 3);

  delay(SAMPLE_PERIOD_MS);
}

