#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 5B
// PID WALL FOLLOWING
//
// ToF → Distance → Error → PID → Motor Correction
// =====================================================

// -------------------- ToF --------------------

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define TARGET_DISTANCE 200.0f   // mm


// -------------------- Motors --------------------

#define PWMA 26
#define AIN1 14
#define AIN2 27

#define PWMB 32
#define BIN1 12
#define BIN2 33

#define STBY 25


// -------------------- Motor Speed --------------------

#define BASE_SPEED 80
#define MAX_SPEED 150


// -------------------- PID --------------------
//
// Start with P only.
// Then increase KD and finally KI.
//
//

float Kp = 0.8f;
float Ki = 0.0f;
float Kd = 0.15f;


// -------------------- PID Variables --------------------

float error = 0;
float previousError = 0;

float integral = 0;

unsigned long previousTime = 0;

#define INTEGRAL_LIMIT 200.0f


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ---------------- Motor Pins ----------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);


  // ---------------- I2C ----------------

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);


  // ---------------- ToF ----------------

  Serial.println();
  Serial.println("======================================");
  Serial.println("LUNA - EXPERIMENT 5B");
  Serial.println("PID WALL FOLLOWING");
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

  Serial.print("Kp: ");
  Serial.println(Kp);

  Serial.print("Ki: ");
  Serial.println(Ki);

  Serial.print("Kd: ");
  Serial.println(Kd);

  Serial.println();

  Serial.println("Distance,Error,PID_Output,Left_PWM,Right_PWM");


  previousTime = micros();

  // Start moving
  setMotor(BASE_SPEED, BASE_SPEED);
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);


  // ===================================================
  // VALID TOF MEASUREMENT
  // ===================================================

  if (measure.RangeStatus != 4)
  {
    float distance = measure.RangeMilliMeter;


    // =================================================
    // ERROR
    // =================================================

    error = distance - TARGET_DISTANCE;


    // =================================================
    // TIME
    // =================================================

    unsigned long currentTime = micros();

    float dt =
      (currentTime - previousTime) * 1e-6f;

    previousTime = currentTime;


    if (dt <= 0.0f || dt > 0.5f)
    {
      dt = 0.05f;
    }


    // =================================================
    // PROPORTIONAL
    // =================================================

    float P = Kp * error;


    // =================================================
    // INTEGRAL
    // =================================================

    integral += error * dt;

    integral = constrain(
      integral,
      -INTEGRAL_LIMIT,
      INTEGRAL_LIMIT
    );

    float I = Ki * integral;


    // =================================================
    // DERIVATIVE
    // =================================================

    float derivative = 0;

    if (dt > 0.001f)
    {
      derivative =
        (error - previousError) / dt;
    }

    float D = Kd * derivative;


    // =================================================
    // PID OUTPUT
    // =================================================

    float output = P + I + D;


    previousError = error;


    // =================================================
    // MOTOR COMMAND
    // =================================================

    int leftSpeed =
      BASE_SPEED + output;

    int rightSpeed =
      BASE_SPEED - output;


    leftSpeed =
      constrain(leftSpeed, 0, MAX_SPEED);

    rightSpeed =
      constrain(rightSpeed, 0, MAX_SPEED);


    // =================================================
    // APPLY MOTOR SPEED
    // =================================================

    setMotor(leftSpeed, rightSpeed);


    // =================================================
    // SERIAL DATA
    // =================================================

    Serial.print(distance);
    Serial.print(",");

    Serial.print(error);
    Serial.print(",");

    Serial.print(output);
    Serial.print(",");

    Serial.print(leftSpeed);
    Serial.print(",");

    Serial.println(rightSpeed);
  }


  // ===================================================
  // INVALID TOF
  // ===================================================

  else
  {
    Serial.println("INVALID TOF - STOP");

    setMotor(0, 0);

    integral = 0;
    previousError = 0;
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
