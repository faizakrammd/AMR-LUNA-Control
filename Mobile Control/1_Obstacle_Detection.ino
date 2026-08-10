#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 1
// ToF → Distance Measurement → Threshold → Motor Stop
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

#define MOTOR_SPEED 120


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
  Serial.println("==============================");
  Serial.println("LUNA - EXPERIMENT 1");
  Serial.println("ToF Obstacle Detection");
  Serial.println("==============================");

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
  Serial.println("LUNA starting...");
  Serial.println();
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
    int distance = measure.RangeMilliMeter;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");


    // =================================================
    // OBSTACLE DETECTION
    // =================================================

    if (distance <= OBSTACLE_THRESHOLD)
    {
      // ---------------- OBSTACLE ----------------

      setMotor(0, 0);

      if (!obstacleDetected)
      {
        Serial.println();
        Serial.println(">>> OBSTACLE DETECTED <<<");
        Serial.println(">>> LUNA STOPPED <<<");
        Serial.println();

        obstacleDetected = true;
      }
    }

    else
    {
      // ---------------- PATH CLEAR ----------------

      if (obstacleDetected)
      {
        Serial.println();
        Serial.println(">>> PATH CLEAR <<<");
        Serial.println(">>> LUNA MOVING <<<");
        Serial.println();

        obstacleDetected = false;
      }

      setMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }

  else
  {
    Serial.println("Out of range");

    // Safety: stop if ToF measurement is invalid
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
