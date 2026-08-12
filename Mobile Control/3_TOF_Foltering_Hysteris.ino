#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 3
// ToF Filtering + Hysteresis / Anti-Chatter
//
// Raw ToF
//    ↓
// Moving Average
//    ↓
// Hysteresis
//    ↓
// MOVE / STOP
// =====================================================

// -------------------- ToF --------------------

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

// Hysteresis thresholds
#define STOP_THRESHOLD 200     // mm
#define MOVE_THRESHOLD 250     // mm


// -------------------- Motors --------------------

#define PWMA 26
#define AIN1 14
#define AIN2 27

#define PWMB 32
#define BIN1 12
#define BIN2 33

#define STBY 25

#define MOTOR_SPEED 80


// -------------------- Moving Average --------------------

#define FILTER_SIZE 10

int distanceBuffer[FILTER_SIZE];

int bufferIndex = 0;
bool bufferFull = false;

float filteredDistance = 0;


// -------------------- Robot State --------------------

bool obstacleDetected = false;


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ---------------- Motor Setup ----------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);


  // ---------------- ToF Setup ----------------

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("======================================");
  Serial.println("LUNA - EXPERIMENT 3");
  Serial.println("ToF Hysteresis / Anti-Chatter");
  Serial.println("======================================");

  if (!lox.begin())
  {
    Serial.println("Failed to detect VL53L0X");

    setMotor(0, 0);

    while (1)
    {
      delay(100);
    }
  }

  Serial.println("VL53L0X detected");


  // ---------------- Initialize Filter ----------------

  for (int i = 0; i < FILTER_SIZE; i++)
  {
    distanceBuffer[i] = 0;
  }

  Serial.println("Moving average initialized.");
  Serial.println();

  Serial.println("STOP threshold : 200 mm");
  Serial.println("MOVE threshold : 250 mm");

  Serial.println();
  Serial.println("Raw(mm),Filtered(mm),State");

  // Start moving
  setMotor(MOTOR_SPEED, MOTOR_SPEED);
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;

  // ---------------- Read ToF ----------------

  lox.rangingTest(&measure, false);


  // ---------------- Check Measurement ----------------

  if (measure.RangeStatus != 4)
  {
    int rawDistance = measure.RangeMilliMeter;


    // =================================================
    // MOVING AVERAGE FILTER
    // =================================================

    distanceBuffer[bufferIndex] = rawDistance;

    bufferIndex++;

    if (bufferIndex >= FILTER_SIZE)
    {
      bufferIndex = 0;
      bufferFull = true;
    }


    int samples;

    if (bufferFull)
      samples = FILTER_SIZE;
    else
      samples = bufferIndex;


    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
      sum += distanceBuffer[i];
    }

    filteredDistance = (float)sum / samples;


    // =================================================
    // HYSTERESIS LOGIC
    // =================================================

    if (!obstacleDetected)
    {
      // Robot is currently moving

      if (filteredDistance <= STOP_THRESHOLD)
      {
        obstacleDetected = true;

        setMotor(0, 0);

        Serial.println(">>> OBSTACLE DETECTED - STOP <<<");
      }
      else
      {
        setMotor(MOTOR_SPEED, MOTOR_SPEED);
      }
    }

    else
    {
      // Robot is currently stopped

      if (filteredDistance >= MOVE_THRESHOLD)
      {
        obstacleDetected = false;

        setMotor(MOTOR_SPEED, MOTOR_SPEED);

        Serial.println(">>> PATH CLEAR - MOVE <<<");
      }
      else
      {
        setMotor(0, 0);
      }
    }


    // =================================================
    // SERIAL OUTPUT
    // =================================================

    Serial.print(rawDistance);
    Serial.print(",");
    Serial.print(filteredDistance, 1);
    Serial.print(",");

    if (obstacleDetected)
      Serial.println("STOP");
    else
      Serial.println("MOVE");
  }

  else
  {
    Serial.println("ToF: OUT OF RANGE");

    // Safety stop
    setMotor(0, 0);
  }


  delay(50);
}


// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(int left, int right)
{
  // ---------------- Left Motor ----------------

  if (left >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    left = -left;
  }


  // ---------------- Right Motor ----------------

  if (right >= 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    right = -right;
  }


  // ---------------- PWM Limit ----------------

  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);


  // ---------------- Apply PWM ----------------

  ledcWrite(PWMA, left);
  ledcWrite(PWMB, right);
}
