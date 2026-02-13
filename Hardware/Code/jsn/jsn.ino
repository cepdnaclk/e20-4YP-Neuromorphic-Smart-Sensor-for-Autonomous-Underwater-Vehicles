#define TRIG_PIN 5
#define ECHO_PIN 18

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  delay(1000);
}

void loop() {
  long duration;
  float distance_cm;

  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(25);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 60000);

  if (duration == 0) {
    Serial.println("No echo detected");
  } else {
    // UNDERWATER calculation
    distance_cm = (duration * 0.148) / 2;

    Serial.print("Echo time (us): ");
    Serial.print(duration);
    Serial.print(" | Underwater distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  }

  delay(1000);
}
