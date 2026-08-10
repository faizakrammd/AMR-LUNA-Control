#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 2
// ToF Filtering
// Raw Distance → Moving Average → Threshold → Motor Stop
// =====================================================

// -------------------- ToF --------------------

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define OBSTACLE_THRESHOLD 200   // mm

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

// -------------------- State --------------------

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
  Serial.println("LUNA - EXPERIMENT 2");
  Serial.println("ToF Moving Average Filtering");
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
  Serial.println("Initializing filter...");


  // ---------------- Initialize Filter ----------------

  for (int i = 0; i < FILTER_SIZE; i++)
  {
    distanceBuffer[i] = 0;
  }

  Serial.println("LUNA starting...");
  Serial.println();
  Serial.println("Raw(mm),Filtered(mm)");
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;

  // ---------------- Read ToF ----------------

  lox.rangingTest(&measure, false);


  // ---------------- Valid Measurement ----------------

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


    // Calculate average

    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
      sum += distanceBuffer[i];
    }

    filteredDistance = (float)sum / samples;


    // =================================================
    // SERIAL OUTPUT
    // =================================================

    Serial.print(rawDistance);
    Serial.print(",");
    Serial.println(filteredDistance, 1);


    // =================================================
    // OBSTACLE DETECTION USING FILTERED VALUE
    // =================================================

    if (filteredDistance <= OBSTACLE_THRESHOLD)
    {
      setMotor(0, 0);

      if (!obstacleDetected)
      {
        Serial.println(">>> OBSTACLE DETECTED <<<");
        Serial.println(">>> LUNA STOPPED <<<");

        obstacleDetected = true;
      }
    }

    else
    {
      if (obstacleDetected)
      {
        Serial.println(">>> PATH CLEAR <<<");
        Serial.println(">>> LUNA MOVING <<<");

        obstacleDetected = false;
      }

      setMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }

  else
  {
    Serial.println("Out of range");

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


  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);


  ledcWrite(PWMA, left);
  ledcWrite(PWMB, right);
}
