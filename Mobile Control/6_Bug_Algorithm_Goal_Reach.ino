#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 6A
// GOAL-DIRECTED BUG-STYLE NAVIGATION
//
// Position + Goal
//       ↓
// Desired Heading
//       ↓
// Yaw Error
//       ↓
// Heading Control
//       ↓
// Motor Differential
//
// Obstacle detected:
// STOP → 90° TURN → MOVE AROUND → RETURN TO GOAL
// =====================================================


// =====================================================
// TOF
// =====================================================

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define OBSTACLE_THRESHOLD 200


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
// ENCODERS
// =====================================================

#define ENC1_A 34
#define ENC1_B 35

#define ENC2_A 36
#define ENC2_B 39

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

long previousLeftCount = 0;
long previousRightCount = 0;


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
// MOTOR PARAMETERS
// =====================================================

#define BASE_SPEED 80
#define ROTATE_SPEED 70

#define MAX_SPEED 150
#define MIN_SPEED 0


// =====================================================
// GOAL
// =====================================================

// Coordinates in millimetres

float robotX = 0.0f;
float robotY = 0.0f;

float goalX = 1000.0f;
float goalY = 500.0f;


// =====================================================
// ENCODER CALIBRATION
// =====================================================
//
// IMPORTANT:
// Replace this after your encoder distance calibration.
//
// Example:
// 1000 counts = 100 mm
// MM_PER_COUNT = 0.10
//

#define MM_PER_COUNT 0.10f


// =====================================================
// HEADING CONTROL
// =====================================================

// Simple proportional heading controller

#define HEADING_KP 1.5f

#define HEADING_TOLERANCE 4.0f


// =====================================================
// GOAL
// =====================================================

#define GOAL_TOLERANCE 100.0f


// =====================================================
// OBSTACLE AVOIDANCE
// =====================================================

#define AVOIDANCE_TURN 90.0f

#define AVOIDANCE_DISTANCE 300.0f

float avoidanceStartYaw = 0.0f;

float avoidanceStartX = 0.0f;
float avoidanceStartY = 0.0f;


// =====================================================
// ROBOT STATES
// =====================================================

enum RobotState
{
  GO_TO_GOAL,
  ROTATE_AROUND_OBSTACLE,
  MOVE_AROUND_OBSTACLE,
  RETURN_TO_GOAL,
  GOAL_REACHED
};

RobotState state = GO_TO_GOAL;


// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

void updateIMU();
void updateOdometry();

void initMPU();
void calibrateGyro();
void readGyro();

void setMotor(int left, int right);

float wrap180(float angle);

float distanceToGoal();
float calculateGoalHeading();

void moveTowardHeading(float targetHeading);


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);


  // =================================================
  // MOTOR SETUP
  // =================================================

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);


  // =================================================
  // ENCODER SETUP
  // =================================================

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC1_A),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC2_A),
    rightEncoderISR,
    RISING
  );


  // =================================================
  // I2C
  // =================================================

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);


  // =================================================
  // TOF
  // =================================================

  Serial.println();
  Serial.println("======================================");
  Serial.println("LUNA - EXPERIMENT 6A");
  Serial.println("GOAL-DIRECTED BUG NAVIGATION");
  Serial.println("======================================");

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
  // MPU6050
  // =================================================

  Serial.println("Initializing MPU6050...");

  initMPU();

  delay(500);


  // =================================================
  // GYRO CALIBRATION
  // =================================================

  Serial.println();
  Serial.println("Keep LUNA stationary.");
  Serial.println("Calibrating gyro...");

  calibrateGyro();

  Serial.println("Gyro calibration complete.");


  // =================================================
  // INITIAL YAW
  // =================================================

  prevTime = micros();

  yaw = 0.0f;

  updateIMU();

  yaw = 0.0f;


  // =================================================
  // INITIAL ENCODER VALUES
  // =================================================

  noInterrupts();

  previousLeftCount = leftEncoderCount;
  previousRightCount = rightEncoderCount;

  interrupts();


  // =================================================
  // START
  // =================================================

  Serial.println();
  Serial.println("======================================");
  Serial.println("START POSITION");
  Serial.println("X = 0 mm");
  Serial.println("Y = 0 mm");
  Serial.println();
  Serial.print("GOAL X = ");
  Serial.print(goalX);
  Serial.println(" mm");

  Serial.print("GOAL Y = ");
  Serial.print(goalY);
  Serial.println(" mm");

  Serial.println("======================================");
  Serial.println();


  state = GO_TO_GOAL;
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  updateIMU();

  updateOdometry();


  // ===================================================
  // GO TO GOAL
  // ===================================================

  if (state == GO_TO_GOAL)
  {
    float goalDistance = distanceToGoal();

    float goalHeading = calculateGoalHeading();

    float headingError =
      wrap180(goalHeading - yaw);


    // -----------------------------------------------
    // PRINT DATA
    // -----------------------------------------------

    Serial.print("X: ");
    Serial.print(robotX, 1);

    Serial.print(" | Y: ");
    Serial.print(robotY, 1);

    Serial.print(" | Yaw: ");
    Serial.print(yaw, 1);

    Serial.print(" | GoalDist: ");
    Serial.print(goalDistance, 1);

    Serial.print(" | GoalHeading: ");
    Serial.print(goalHeading, 1);

    Serial.print(" | HeadingError: ");
    Serial.println(headingError, 1);


    // -----------------------------------------------
    // GOAL REACHED
    // -----------------------------------------------

    if (goalDistance <= GOAL_TOLERANCE)
    {
      setMotor(0, 0);

      state = GOAL_REACHED;

      Serial.println();
      Serial.println("======================================");
      Serial.println(">>> GOAL REACHED <<<");
      Serial.println("======================================");

      return;
    }


    // -----------------------------------------------
    // TOF
    // -----------------------------------------------

    VL53L0X_RangingMeasurementData_t measure;

    lox.rangingTest(&measure, false);


    if (measure.RangeStatus != 4)
    {
      int distance = measure.RangeMilliMeter;

      Serial.print("ToF: ");
      Serial.print(distance);
      Serial.println(" mm");


      // ---------------------------------------------
      // OBSTACLE
      // ---------------------------------------------

      if (distance < OBSTACLE_THRESHOLD)
      {
        Serial.println();
        Serial.println(">>> OBSTACLE DETECTED <<<");
        Serial.println(">>> STOPPING <<<");

        setMotor(0, 0);

        delay(300);


        // Save current position
        avoidanceStartX = robotX;
        avoidanceStartY = robotY;


        // Save current yaw
        avoidanceStartYaw = yaw;


        state = ROTATE_AROUND_OBSTACLE;

        Serial.println(">>> ROTATING 90 DEGREES <<<");
      }


      // ---------------------------------------------
      // CLEAR
      // ---------------------------------------------

      else
      {
        moveTowardHeading(goalHeading);
      }
    }


    // -----------------------------------------------
    // INVALID TOF
    // -----------------------------------------------

    else
    {
      Serial.println("ToF INVALID - STOP");

      setMotor(0, 0);
    }
  }


  // ===================================================
  // ROTATE AROUND OBSTACLE
  // ===================================================

  else if (state == ROTATE_AROUND_OBSTACLE)
  {
    float rotation =
      fabs(
        wrap180(
          yaw - avoidanceStartYaw
        )
      );


    Serial.print("Avoidance rotation: ");
    Serial.print(rotation, 1);
    Serial.println(" deg");


    if (rotation < (AVOIDANCE_TURN - 2.0f))
    {
      setMotor(
        ROTATE_SPEED,
        -ROTATE_SPEED
      );
    }

    else
    {
      setMotor(0, 0);

      delay(300);

      Serial.println(">>> 90 DEGREE TURN COMPLETE <<<");
      Serial.println(">>> MOVING AROUND OBSTACLE <<<");


      avoidanceStartX = robotX;
      avoidanceStartY = robotY;

      state = MOVE_AROUND_OBSTACLE;

      setMotor(
        BASE_SPEED,
        BASE_SPEED
      );
    }
  }


  // ===================================================
  // MOVE AROUND OBSTACLE
  // ===================================================

  else if (state == MOVE_AROUND_OBSTACLE)
  {
    float dx =
      robotX - avoidanceStartX;

    float dy =
      robotY - avoidanceStartY;


    float distanceMoved =
      sqrtf(
        dx * dx +
        dy * dy
      );


    Serial.print("Avoidance distance: ");
    Serial.print(distanceMoved, 1);
    Serial.println(" mm");


    // -----------------------------------------------
    // MOVE AROUND OBSTACLE
    // -----------------------------------------------

    if (distanceMoved < AVOIDANCE_DISTANCE)
    {
      setMotor(
        BASE_SPEED,
        BASE_SPEED
      );
    }

    else
    {
      setMotor(0, 0);

      delay(300);

      Serial.println(">>> OBSTACLE CLEARANCE DISTANCE COMPLETE <<<");
      Serial.println(">>> RETURNING TO GOAL HEADING <<<");

      state = RETURN_TO_GOAL;
    }
  }


  // ===================================================
  // RETURN TO GOAL
  // ===================================================

  else if (state == RETURN_TO_GOAL)
  {
    float goalHeading =
      calculateGoalHeading();

    float headingError =
      wrap180(
        goalHeading - yaw
      );


    Serial.print("Goal Heading: ");
    Serial.print(goalHeading, 1);

    Serial.print(" | Current Yaw: ");
    Serial.print(yaw, 1);

    Serial.print(" | Error: ");
    Serial.println(headingError, 1);


    // -----------------------------------------------
    // HEADING CORRECT
    // -----------------------------------------------

    if (fabs(headingError) <= HEADING_TOLERANCE)
    {
      Serial.println(">>> GOAL HEADING ACQUIRED <<<");

      state = GO_TO_GOAL;

      setMotor(
        BASE_SPEED,
        BASE_SPEED
      );
    }


    // -----------------------------------------------
    // TURN TOWARD GOAL
    // -----------------------------------------------

    else
    {
      setMotor(
        -ROTATE_SPEED,
        ROTATE_SPEED
      );
    }
  }


  // ===================================================
  // GOAL REACHED
  // ===================================================

  else if (state == GOAL_REACHED)
  {
    setMotor(0, 0);
  }


  delay(20);
}


// =====================================================
// MOVE TOWARD HEADING
// =====================================================

void moveTowardHeading(float targetHeading)
{
  float error =
    wrap180(
      targetHeading - yaw
    );


  float correction =
    HEADING_KP * error;


  correction =
    constrain(
      correction,
      -60.0f,
      60.0f
    );


  int leftSpeed =
    BASE_SPEED + correction;

  int rightSpeed =
    BASE_SPEED - correction;


  leftSpeed =
    constrain(
      leftSpeed,
      MIN_SPEED,
      MAX_SPEED
    );

  rightSpeed =
    constrain(
      rightSpeed,
      MIN_SPEED,
      MAX_SPEED
    );


  setMotor(
    leftSpeed,
    rightSpeed
  );
}


// =====================================================
// DISTANCE TO GOAL
// =====================================================

float distanceToGoal()
{
  float dx =
    goalX - robotX;

  float dy =
    goalY - robotY;

  return sqrtf(
    dx * dx +
    dy * dy
  );
}


// =====================================================
// CALCULATE GOAL HEADING
// =====================================================

float calculateGoalHeading()
{
  float dx =
    goalX - robotX;

  float dy =
    goalY - robotY;


  return atan2f(
    dy,
    dx
  ) * RAD_TO_DEG;
}


// =====================================================
// LEFT ENCODER ISR
// =====================================================

void IRAM_ATTR leftEncoderISR()
{
  if (
    digitalRead(AIN1) == HIGH &&
    digitalRead(AIN2) == LOW
  )
  {
    leftEncoderCount++;
  }
  else
  {
    leftEncoderCount--;
  }
}


// =====================================================
// RIGHT ENCODER ISR
// =====================================================

void IRAM_ATTR rightEncoderISR()
{
  if (
    digitalRead(BIN1) == HIGH &&
    digitalRead(BIN2) == LOW
  )
  {
    rightEncoderCount++;
  }
  else
  {
    rightEncoderCount--;
  }
}


// =====================================================
// ODOMETRY
// =====================================================

void updateOdometry()
{
  long leftCount;
  long rightCount;


  noInterrupts();

  leftCount =
    leftEncoderCount;

  rightCount =
    rightEncoderCount;

  interrupts();


  long deltaLeft =
    leftCount -
    previousLeftCount;

  long deltaRight =
    rightCount -
    previousRightCount;


  previousLeftCount =
    leftCount;

  previousRightCount =
    rightCount;


  float leftDistance =
    deltaLeft *
    MM_PER_COUNT;

  float rightDistance =
    deltaRight *
    MM_PER_COUNT;


  float distance =
    (
      leftDistance +
      rightDistance
    ) / 2.0f;


  float heading =
    yaw *
    DEG_TO_RAD;


  robotX +=
    distance *
    cosf(heading);

  robotY +=
    distance *
    sinf(heading);
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


  left =
    constrain(
      left,
      0,
      255
    );

  right =
    constrain(
      right,
      0,
      255
    );


  ledcWrite(
    PWMA,
    left
  );

  ledcWrite(
    PWMB,
    right
  );
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


  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);


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


  for (
    int i = 0;
    i < CALIB_SAMPLES;
    i++
  )
  {
    readGyro();

    sum += gyroZ;

    delay(CALIB_DELAY_MS);
  }


  gyroZ_off =
    sum /
    CALIB_SAMPLES;


  Serial.print("Gyro Z Offset: ");
  Serial.println(
    gyroZ_off,
    4
  );
}


// =====================================================
// READ GYRO
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
    (int16_t)(
      Wire.read() << 8 |
      Wire.read()
    );


  int16_t gy =
    (int16_t)(
      Wire.read() << 8 |
      Wire.read()
    );


  int16_t gz =
    (int16_t)(
      Wire.read() << 8 |
      Wire.read()
    );


  gyroZ =
    gz /
    GYRO_SCALE;
}


// =====================================================
// UPDATE IMU
// =====================================================

void updateIMU()
{
  unsigned long now =
    micros();


  dt =
    (now - prevTime) *
    1e-6f;


  prevTime =
    now;


  if (
    dt <= 0.0f ||
    dt > 0.5f
  )
  {
    dt = 0.005f;
  }


  readGyro();


  gyroZ -=
    gyroZ_off;


  if (
    fabs(gyroZ) <
    GYRO_DEADBAND
  )
  {
    gyroZ = 0;
  }


  yaw +=
    gyroZ *
    dt;


  yaw =
    wrap180(yaw);
}


// =====================================================
// ANGLE WRAP
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
