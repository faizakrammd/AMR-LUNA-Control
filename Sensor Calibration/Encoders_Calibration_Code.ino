#include <Arduino.h>

// ── Pin definitions (match your main code) ──────────────────────────────────
#define ENC_A1  34
#define ENC_A2  35
#define ENC_B1  39
#define ENC_B2  36

// ── Encoder state ────────────────────────────────────────────────────────────
volatile long countA = 0;
volatile long countB = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

void IRAM_ATTR encoderA_ISR() {
  int a = digitalRead(ENC_A1);
  int b = digitalRead(ENC_A2);
  int encoded = (a << 1) | b;
  int sum = (lastA << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) countA++;
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  countA--;
  lastA = encoded;
}

void IRAM_ATTR encoderB_ISR() {
  int a = digitalRead(ENC_B1);
  int b = digitalRead(ENC_B2);
  int encoded = (a << 1) | b;
  int sum = (lastB << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) countB++;
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  countB--;
  lastB = encoded;
}

// ── Config ───────────────────────────────────────────────────────────────────
// Set this to the exact distance you will push the robot (millimetres)
const float KNOWN_DISTANCE_MM = 300.0f;

// Number of calibration runs to average
const int NUM_RUNS = 3;

// ── Helpers ──────────────────────────────────────────────────────────────────
void waitForEnter() {
  Serial.println("  → Press ENTER when ready...");
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
    }
    delay(10);
  }
  // Flush any leftover bytes
  while (Serial.available()) Serial.read();
}

void resetCounts() {
  noInterrupts();
  countA = 0;
  countB = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(ENC_A1, INPUT);  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT);  pinMode(ENC_B2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  Serial.println("\n====================================");
  Serial.println("  ENCODER CALIBRATION — PUSH METHOD");
  Serial.println("====================================");
  Serial.printf("  Calibration distance : %.1f mm\n", KNOWN_DISTANCE_MM);
  Serial.printf("  Number of runs       : %d\n\n", NUM_RUNS);

  float sumTicksPerMmA = 0;
  float sumTicksPerMmB = 0;

  for (int run = 1; run <= NUM_RUNS; run++) {
    Serial.printf("── Run %d of %d ─────────────────────\n", run, NUM_RUNS);
    Serial.println("  Place robot at START mark.");
    waitForEnter();

    resetCounts();
    Serial.printf("  Encoders zeroed. Push robot EXACTLY %.0f mm forward.\n",
                  KNOWN_DISTANCE_MM);
    waitForEnter();

    noInterrupts();
    long snapA = countA;
    long snapB = countB;
    interrupts();

    float tpmA = fabsf((float)snapA) / KNOWN_DISTANCE_MM;
    float tpmB = fabsf((float)snapB) / KNOWN_DISTANCE_MM;
    sumTicksPerMmA += tpmA;
    sumTicksPerMmB += tpmB;

    Serial.printf("  Ticks  →  A: %ld   B: %ld\n", snapA, snapB);
    Serial.printf("  Ticks/mm → A: %.4f   B: %.4f\n\n", tpmA, tpmB);
  }

  float avgA = sumTicksPerMmA / NUM_RUNS;
  float avgB = sumTicksPerMmB / NUM_RUNS;
  float avgMmPerTickA = 1.0f / avgA;
  float avgMmPerTickB = 1.0f / avgB;

  Serial.println("====================================");
  Serial.println("  CALIBRATION RESULTS (averaged)");
  Serial.println("====================================");
  Serial.printf("  Motor A  →  TICKS_PER_MM = %.4f   MM_PER_TICK = %.6f\n",
                avgA, avgMmPerTickA);
  Serial.printf("  Motor B  →  TICKS_PER_MM = %.4f   MM_PER_TICK = %.6f\n",
                avgB, avgMmPerTickB);
  Serial.println("\n  Copy these into your main code:");
  Serial.printf("    #define TICKS_PER_MM_A  %.4f\n", avgA);
  Serial.printf("    #define TICKS_PER_MM_B  %.4f\n", avgB);
  Serial.println("====================================");
}

void loop() {
  // Nothing — calibration is done in setup()
}