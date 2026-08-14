#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 5
// WALL FOLLOWING - BASIC BANG-BANG CONTROL
// =====================================================

// -------------------- ToF --------------------

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define TARGET_DISTANCE 200   // mm
#define DEAD_BAND 20          // mm


// -------------------- Motors --------------------

#define PWMA 26
#define AIN1 14
#define AIN2 27

#define PWMB 32
#define BIN1 12
#define BIN2 33

#define STBY 25


// -------------------- Speeds --------------------

#define FORWARD_SPEED 80
#define CORRECTION_SPEED 60


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // ToF
  Serial.println();
  Serial.println("======================================");
  Serial.println("LUNA - EXPERIMENT 5");
  Serial.println("WALL FOLLOWING");
  Serial.println("======================================");

  if (!lox.begin())
  {
    Serial.println("VL53L0X NOT DETECTED");

    setMotor(0, 0);

    while (1)
    {
      delay(100);
    }
  }

  Serial.println("VL53L0X detected");
  Serial.println();

  Serial.print("Target distance: ");
  Serial.print(TARGET_DISTANCE);
  Serial.println(" mm");

  Serial.print("Dead band: +/- ");
  Serial.print(DEAD_BAND);
  Serial.println(" mm");

  Serial.println();
  Serial.println("Distance(mm),Error(mm),Action");

  // Start moving
  setMotor(FORWARD_SPEED, FORWARD_SPEED);
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  // Valid measurement
  if (measure.RangeStatus != 4)
  {
    int distance = measure.RangeMilliMeter;

    // Error relative to desired wall distance
    int error = distance - TARGET_DISTANCE;

    Serial.print(distance);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");


    // =================================================
    // TOO FAR FROM WALL
    // =================================================

    if (error > DEAD_BAND)
    {
      Serial.println("TURN TOWARD WALL");

      setMotor(
        FORWARD_SPEED + CORRECTION_SPEED,
        FORWARD_SPEED - CORRECTION_SPEED
      );
    }


    // =================================================
    // TOO CLOSE TO WALL
    // =================================================

    else if (error < -DEAD_BAND)
    {
      Serial.println("TURN AWAY FROM WALL");

      setMotor(
        FORWARD_SPEED - CORRECTION_SPEED,
        FORWARD_SPEED + CORRECTION_SPEED
      );
    }


    // =================================================
    // CORRECT DISTANCE
    // =================================================

    else
    {
      Serial.println("STRAIGHT");

      setMotor(
        FORWARD_SPEED,
        FORWARD_SPEED
      );
    }
  }


  // =================================================
  // INVALID MEASUREMENT
  // =================================================

  else
  {
    Serial.println("INVALID - STOP");

    setMotor(0, 0);
  }

  delay(50);
}


// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(int left, int right)
{
  // Left motor
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


  // Right motor
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


  // Limit PWM
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);


  // Apply PWM
  ledcWrite(PWMA, left);
  ledcWrite(PWMB, right);
}
