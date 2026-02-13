// 28BYJ-48 + ULN2003 HALF-STEP (Anticlockwise) - ESP32

#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// Half-step sequence (IN1, IN2, IN3, IN4)
const int halfStepSeq[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

int stepIndex = 7;          // start from end for reverse direction
int stepDelayUs = 2500;     // increase for more torque

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);
  Serial.println("Half-step anticlockwise...");
}

void halfStepCCW() {
  digitalWrite(IN1, halfStepSeq[stepIndex][0]);
  digitalWrite(IN2, halfStepSeq[stepIndex][1]);
  digitalWrite(IN3, halfStepSeq[stepIndex][2]);
  digitalWrite(IN4, halfStepSeq[stepIndex][3]);

  stepIndex--;
  if (stepIndex < 0) stepIndex = 7;

  delayMicroseconds(stepDelayUs);
}

void loop() {
  halfStepCCW();   // runs forever anticlockwise
}
