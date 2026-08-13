#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 4
// Reactive Obstacle Avoidance
//
// START
//   ↓
// CHECK ToF
//   ↓
// CLEAR → MOVE FORWARD
//   ↓
// OBSTACLE < THRESHOLD
//   ↓
// STOP
//   ↓
// ROTATE 90° USING IMU
//   ↓
// MOVE FORWARD
//   ↓
// REPEAT
// =====================================================


// =====================================================
// TOF
// =====================================================

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define OBSTACLE_THRESHOLD 200   // mm


// =====================================================
// MPU6050
// =====================================================

#define MPU_ADDR 0x68

#define GYRO_SCALE 131.0f
#define CALIB_SAMPLES 2000
#define CALIB_DELAY_MS 2
#define GYRO_DEADBAND 0.5f

float gyroZ = 0;
float gyroZ_off = 0;

float yaw = 0;
float dt = 0;

unsigned long prevTime;


// =====================================================
// MOTOR PINS
// =====================================================

#define PWMA 26
#define AIN1 14
#define AIN2 27

#define PWMB 32
#define BIN1 12
#define BIN2 33

#define STBY 25


// =====================================================
// MOTOR SPEED
// =====================================================

#define MOTOR_SPEED 80
#define ROTATE_SPEED 70


// =====================================================
// ROTATION
// =====================================================

#define ROTATION_TARGET 90.0f
#define ROTATION_TOLERANCE 2.0f

float rotationStartYaw = 0;


// =====================================================
// ROBOT STATES
// =====================================================

enum RobotState
{
  MOVE_FORWARD,
  ROTATE_90
};

RobotState state = MOVE_FORWARD;


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ---------------- Motor pins ----------------

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


  // =================================================
  // TOF INITIALIZATION
  // =================================================

  Serial.println();
  Serial.println("====================================");
  Serial.println("LUNA - EXPERIMENT 4");
  Serial.println("Reactive Obstacle Avoidance");
  Serial.println("====================================");

  Serial.println("Initializing VL53L0X...");

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


  // =================================================
  // MPU INITIALIZATION
  // =================================================

  Serial.println("Initializing MPU6050...");

  initMPU();

  delay(500);


  // =================================================
  // GYRO CALIBRATION
  // =================================================

  Serial.println("Calibrating gyro...");
  Serial.println("Keep LUNA stationary.");

  calibrateGyro();

  Serial.println("Gyro calibration complete.");


  // =================================================
  // INITIAL YAW
  // =================================================

  prevTime = micros();

  yaw = 0;

  updateIMU();

  yaw = 0;


  Serial.println();
  Serial.println("====================================");
  Serial.println("LUNA READY");
  Serial.print("Obstacle threshold: ");
  Serial.print(OBSTACLE_THRESHOLD);
  Serial.println(" mm");
  Serial.println("====================================");
  Serial.println();


  // Start moving
  setMotor(MOTOR_SPEED, MOTOR_SPEED);
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Always update IMU
  updateIMU();


  // ===================================================
  // STATE 1 - MOVE FORWARD
  // ===================================================

  if (state == MOVE_FORWARD)
  {
    VL53L0X_RangingMeasurementData_t measure;

    lox.rangingTest(&measure, false);


    // -------------------------------------------------
    // Valid ToF measurement
    // -------------------------------------------------

    if (measure.RangeStatus != 4)
    {
      int distance = measure.RangeMilliMeter;

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" mm | Yaw: ");
      Serial.println(yaw, 2);


      // =================================================
      // OBSTACLE DETECTED
      // =================================================

      if (distance < OBSTACLE_THRESHOLD)
      {
        Serial.println();
        Serial.println(">>> OBSTACLE DETECTED <<<");

        // STOP
        setMotor(0, 0);

        delay(300);


        // Save current yaw
        rotationStartYaw = yaw;

        Serial.print("Rotation start yaw: ");
        Serial.println(rotationStartYaw);


        // Change state
        state = ROTATE_90;

        Serial.println(">>> ROTATING 90 DEGREES <<<");
        Serial.println();
      }


      // =================================================
      // NO OBSTACLE
      // =================================================

      else
      {
        // Keep moving forward
        setMotor(MOTOR_SPEED, MOTOR_SPEED);
      }
    }


    // -------------------------------------------------
    // Invalid ToF
    // -------------------------------------------------

    else
    {
      Serial.println("ToF invalid - STOPPING");

      setMotor(0, 0);
    }
  }


  // ===================================================
  // STATE 2 - ROTATE 90°
  // ===================================================

  else if (state == ROTATE_90)
  {
    float rotation =
      fabs(wrap180(yaw - rotationStartYaw));


    Serial.print("Rotation: ");
    Serial.print(rotation, 2);
    Serial.println(" deg");


    // -------------------------------------------------
    // Continue rotating
    // -------------------------------------------------

    if (rotation < (ROTATION_TARGET - ROTATION_TOLERANCE))
    {
      setMotor(ROTATE_SPEED, -ROTATE_SPEED);
    }


    // -------------------------------------------------
    // 90° reached
    // -------------------------------------------------

    else
    {
      setMotor(0, 0);

      Serial.println();
      Serial.println(">>> 90 DEGREE ROTATION COMPLETE <<<");

      delay(300);


      // Return to forward motion
      state = MOVE_FORWARD;

      Serial.println(">>> MOVING FORWARD <<<");
      Serial.println();

      setMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }


  delay(20);
}


// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(int left, int right)
{
  // ---------------- Left motor ----------------

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


  // ---------------- Right motor ----------------

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


// =====================================================
// MPU6050 INITIALIZATION
// =====================================================

void initMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission(true);

  delay(100);


  // Digital low-pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);


  // Gyro ±250 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}


// =====================================================
// GYRO CALIBRATION
// =====================================================

void calibrateGyro()
{
  float sum = 0;


  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    readGyro();

    sum += gyroZ;

    delay(CALIB_DELAY_MS);
  }


  gyroZ_off = sum / CALIB_SAMPLES;


  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_off, 4);
}


// =====================================================
// READ MPU6050 GYRO
// =====================================================

void readGyro()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(
    (uint8_t)MPU_ADDR,
    (uint8_t)6,
    (uint8_t)true
  );


  int16_t gx =
    (int16_t)(Wire.read() << 8 | Wire.read());

  int16_t gy =
    (int16_t)(Wire.read() << 8 | Wire.read());

  int16_t gz =
    (int16_t)(Wire.read() << 8 | Wire.read());


  gyroZ = gz / GYRO_SCALE;
}


// =====================================================
// UPDATE IMU
// =====================================================

void updateIMU()
{
  unsigned long now = micros();


  dt = (now - prevTime) * 1e-6f;

  prevTime = now;


  if (dt <= 0.0f || dt > 0.5f)
  {
    dt = 0.005f;
  }


  readGyro();


  gyroZ -= gyroZ_off;


  if (fabs(gyroZ) < GYRO_DEADBAND)
  {
    gyroZ = 0;
  }


  yaw += gyroZ * dt;


  yaw = wrap180(yaw);
}


// =====================================================
// WRAP ANGLE TO -180 ... +180
// =====================================================

float wrap180(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }


  while (angle < -180.0f)
  {
    angle += 360.0f;
  }


  return angle;
}
