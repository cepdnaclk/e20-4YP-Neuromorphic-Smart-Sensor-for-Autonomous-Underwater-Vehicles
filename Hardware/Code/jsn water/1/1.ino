// =========================================
// UNDERWATER STEP3 (POOL-STABLE) — 1 SENSOR
// time_ms,echo_us,valid,dist_cm,dist_f_cm,safe_ref_cm,in_danger,danger,event
// event: +1 entered danger, -1 exited danger, 0 none
// =========================================

#define TRIG_PIN 5
#define ECHO_PIN 18

const int   SAMPLE_PERIOD_MS = 200;
const int   TRIG_HIGH_US     = 25;
const long  TIMEOUT_US       = 60000;

const long  ECHO_MIN_US      = 250;
const long  ECHO_MAX_US      = 30000;

const int   NPINGS           = 7;
const int   INTERPING_MS     = 35;

const float CM_PER_US_ONEWAY = 0.148f;  // underwater speed-of-sound approx
const float MIN_DIST_CM      = 20.0f;
const float MAX_DIST_CM      = 600.0f;

const float DIST_EMA_A       = 0.25f;

// ---- YOU choose these ----
const float SAFE_DIST_CM     = 120.0f;  // danger if <= this
const float HYST_CM          = 20.0f;   // must go above SAFE+HYST to exit

// ---- Confirmation to avoid false triggers ----
const int   CONF_N           = 3;       // window size
const int   ENTER_K          = 2;       // need >=2/3 below threshold to enter
const int   EXIT_K           = 2;       // need >=2/3 above (SAFE+HYST) to exit

// ---- Outlier reject (anti-multipath) ----
const float DROP_REJECT_FRAC = 0.40f;   // reject if dist drops by >40% suddenly

// ---- Optional reference (logging only) ----
const int   CAL_SAMPLES      = 30;
int cal_n = 0;
float safe_ref_cm = 0.0f;

bool  dist_init = false;
float dist_f_cm = 0.0f;

bool  in_danger = false;

// history of filtered distances
float hist[CONF_N];
int   hist_i = 0;
int   hist_count = 0;

float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

void sortArray(long *a, int n) {
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
  sortArray(vals, k);
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

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  delay(500);

  Serial.println("UNDERWATER STEP3: time_ms,echo_us,valid,dist_cm,dist_f_cm,safe_ref_cm,in_danger,danger,event");
  Serial.println("Point into OPEN water for a few seconds, then test with a flat obstacle.");
}

void loop() {
  unsigned long t = millis();

  long echo_us = readEchoMedian();
  bool valid = (echo_us != 0);

  float dist_cm = 0.0f;
  int event = 0;
  float danger = 0.0f;

  if (valid) {
    dist_cm = clampf(echoToDistCm(echo_us), MIN_DIST_CM, MAX_DIST_CM);

    if (!dist_init) {
      dist_f_cm = dist_cm;
      dist_init = true;
    } else {
      // anti-multipath: reject sudden huge drop
      if (dist_cm < dist_f_cm * (1.0f - DROP_REJECT_FRAC)) {
        // ignore this reading (keep previous dist_f)
      } else {
        dist_f_cm = DIST_EMA_A * dist_cm + (1.0f - DIST_EMA_A) * dist_f_cm;
      }
    }

    // log-only safe reference
    if (cal_n < CAL_SAMPLES) {
      safe_ref_cm = (cal_n == 0) ? dist_f_cm : (0.85f * safe_ref_cm + 0.15f * dist_f_cm);
      cal_n++;
    }

    // confirmation window for state transitions
    pushHist(dist_f_cm);

    if (!in_danger) {
      if (hist_count == CONF_N && countBelow(SAFE_DIST_CM) >= ENTER_K) {
        in_danger = true;
        event = +1; // entered danger (one-time pulse)
      }
    } else {
      if (hist_count == CONF_N && countAbove(SAFE_DIST_CM + HYST_CM) >= EXIT_K) {
        in_danger = false;
        event = -1; // exited danger (one-time pulse)
      }
    }

    // danger value for logging/steering
    if (in_danger) {
      // 1 at/under SAFE, then decreases as you go away
      float far = SAFE_DIST_CM + 100.0f; // tune slope width
      danger = (dist_f_cm <= SAFE_DIST_CM) ? 1.0f :
               (dist_f_cm >= far) ? 0.0f :
               1.0f - (dist_f_cm - SAFE_DIST_CM) / (far - SAFE_DIST_CM);
    } else {
      danger = 0.0f; // IMPORTANT: open water should show 0
    }
  } else {
    // invalid echo: keep state, do not force danger
  }

  Serial.print(t); Serial.print(",");
  Serial.print(echo_us); Serial.print(",");
  Serial.print(valid ? 1 : 0); Serial.print(",");
  Serial.print(valid ? dist_cm : 0.0f, 1); Serial.print(",");
  Serial.print(dist_f_cm, 1); Serial.print(",");
  Serial.print((cal_n >= CAL_SAMPLES) ? safe_ref_cm : 0.0f, 1); Serial.print(",");
  Serial.print(in_danger ? 1 : 0); Serial.print(",");
  Serial.print(danger, 3); Serial.print(",");
  Serial.println(event);

  delay(SAMPLE_PERIOD_MS);
}


