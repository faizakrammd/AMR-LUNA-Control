#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// LUNA - EXPERIMENT 6B
// BUG2 ALGORITHM
//
// SINGLE RIGHT-SIDE ToF
//
// Move straight
//      ↓
// Right-side obstacle detected
//      ↓
// Record hit point
//      ↓
// EXACT 5B PID WALL FOLLOWING
//      ↓
// M-line + closer to goal
//      ↓
// Leave wall
//      ↓
// Move straight
//      ↓
// Goal
// =====================================================


// =====================================================
// ToF
// =====================================================

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define SDA_PIN 21
#define SCL_PIN 22

#define TARGET_DISTANCE 200.0f
#define OBSTACLE_THRESHOLD 300.0f


// =====================================================
// Motors
// =====================================================

#define PWMA 26
#define AIN1 14
#define AIN2 27

#define PWMB 32
#define BIN1 12
#define BIN2 33

#define STBY 25


// =====================================================
// Motor Speed
// =====================================================

#define BASE_SPEED 80
#define MAX_SPEED 150


// =====================================================
// EXACT 5B PID
// =====================================================

float Kp = 0.8f;
float Ki = 0.0f;
float Kd = 0.15f;


// =====================================================
// PID Variables
// =====================================================

float error = 0;
float previousError = 0;

float integral = 0;

unsigned long previousTime = 0;

#define INTEGRAL_LIMIT 200.0f


// =====================================================
// MPU6050
// =====================================================

#define MPU_ADDR 0x68

#define GYRO_SCALE 131.0f
#define CALIB_SAMPLES 2000
#define CALIB_DELAY_MS 2
#define GYRO_DEADBAND 0.5f

float gyroZ = 0.0f;
float gyroZOffset = 0.0f;

float yaw = 0.0f;

unsigned long imuPreviousTime;


// =====================================================
// ENCODERS
// =====================================================

#define ENC1_A 34
#define ENC2_A 36

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

long previousLeftCount = 0;
long previousRightCount = 0;


// =====================================================
// ENCODER CALIBRATION
// =====================================================
//
// CHANGE THIS ONLY IF YOU HAVE YOUR CALIBRATED VALUE.
//
// Example:
// 1000 encoder counts = 100 mm
// therefore:
// MM_PER_COUNT = 0.1
//

#define MM_PER_COUNT 0.10f


// =====================================================
// GOAL
// =====================================================

float startX = 0.0f;
float startY = 0.0f;

float goalX = 1000.0f;
float goalY = 500.0f;


// =====================================================
// INITIAL HEADING
// =====================================================
//
// The robot does NOT turn toward this angle.
//
// It starts physically aligned with the goal direction.
//
// This angle is used ONLY for odometry coordinates.
//
// atan2(500,1000) = 26.565 degrees
//

float initialHeadingDeg;


// =====================================================
// Robot Position
// =====================================================

float robotX = 0.0f;
float robotY = 0.0f;


// =====================================================
// M-LINE
// =====================================================

float lineA;
float lineB;
float lineC;


// =====================================================
// BUG2 HIT POINT
// =====================================================

float hitPointX = 0.0f;
float hitPointY = 0.0f;

float hitGoalDistance = 0.0f;


// =====================================================
// WALL FOLLOW TRAVEL
// =====================================================

float wallTravel = 0.0f;

float previousWallX = 0.0f;
float previousWallY = 0.0f;


// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState
{
  GO_TO_GOAL,
  WALL_FOLLOW,
  GOAL_REACHED
};

RobotState state = GO_TO_GOAL;


// =====================================================
// BUG2 FLAG
// =====================================================

bool obstacleHandled = false;


// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void setMotor(int left, int right);

float readToF();

void initMPU();
void calibrateGyro();
void readGyro();
void updateIMU();

void updateOdometry();

void moveStraight();
void wallFollow();

float distanceToGoal();
float distanceFromMLine();

bool onMLine();

float wrap180(float angle);

void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);


  // ===================================================
  // MOTOR PINS
  // ===================================================

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);


  // ===================================================
  // ENCODERS
  // ===================================================

  pinMode(ENC1_A, INPUT);
  pinMode(ENC2_A, INPUT);

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


  // ===================================================
  // I2C
  // ===================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);


  // ===================================================
  // ToF
  // ===================================================

  Serial.println();
  Serial.println("======================================");
  Serial.println("LUNA - EXPERIMENT 6B");
  Serial.println("BUG2");
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


  // ===================================================
  // MPU
  // ===================================================

  initMPU();

  delay(500);


  // ===================================================
  // GYRO CALIBRATION
  // ===================================================

  Serial.println();
  Serial.println("Keep LUNA stationary.");
  Serial.println("Calibrating gyro...");

  calibrateGyro();

  Serial.println("Gyro calibration complete");


  // ===================================================
  // INITIAL IMU
  // ===================================================

  imuPreviousTime = micros();

  yaw = 0.0f;


  // ===================================================
  // ENCODER INITIALIZATION
  // ===================================================

  noInterrupts();

  previousLeftCount =
    leftEncoderCount;

  previousRightCount =
    rightEncoderCount;

  interrupts();


  // ===================================================
  // INITIAL POSITION
  // ===================================================

  robotX = startX;
  robotY = startY;


  // ===================================================
  // INITIAL HEADING
  // ===================================================
  //
  // IMPORTANT:
  // This does NOT command the robot to rotate.
  //
  // It only tells odometry what direction the
  // robot's physical starting direction represents.
  //

  initialHeadingDeg =
    atan2(
      goalY - startY,
      goalX - startX
    ) *
    RAD_TO_DEG;


  // ===================================================
  // M-LINE
  // ===================================================

  lineA =
    startY - goalY;

  lineB =
    goalX - startX;

  lineC =
    startX * goalY -
    goalX * startY;


  // ===================================================
  // START
  // ===================================================

  Serial.println();

  Serial.print("START: ");
  Serial.print(startX);
  Serial.print(", ");
  Serial.println(startY);

  Serial.print("GOAL: ");
  Serial.print(goalX);
  Serial.print(", ");
  Serial.println(goalY);

  Serial.print("Initial heading: ");
  Serial.println(initialHeadingDeg);

  Serial.println();

  Serial.println("ToF = RIGHT SIDE");

  Serial.println("Starting straight movement...");

  Serial.println();

  previousTime = micros();

  setMotor(
    BASE_SPEED,
    BASE_SPEED
  );
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

  if (
    state ==
    GO_TO_GOAL
  )
  {
    float distance =
      readToF();


    // =================================================
    // CHECK RIGHT-SIDE OBSTACLE
    // =================================================

    if (
      !obstacleHandled &&
      distance > 0 &&
      distance <
      OBSTACLE_THRESHOLD
    )
    {
      // ===============================================
      // STOP
      // ===============================================

      setMotor(
        0,
        0
      );

      delay(150);


      // ===============================================
      // ACTUAL HIT POINT
      // ===============================================

      hitPointX =
        robotX;

      hitPointY =
        robotY;

      hitGoalDistance =
        distanceToGoal();


      Serial.println();
      Serial.println(
        "======================================"
      );

      Serial.println(
        "OBSTACLE DETECTED"
      );

      Serial.print(
        "Hit X = "
      );

      Serial.println(
        hitPointX,
        2
      );

      Serial.print(
        "Hit Y = "
      );

      Serial.println(
        hitPointY,
        2
      );

      Serial.print(
        "Distance to goal = "
      );

      Serial.println(
        hitGoalDistance,
        2
      );

      Serial.println(
        "Starting wall following..."
      );

      Serial.println(
        "======================================"
      );


      // ===============================================
      // RESET WALL FOLLOW PID
      // ===============================================

      error = 0;
      previousError = 0;
      integral = 0;

      previousTime = micros();


      // ===============================================
      // WALL TRAVEL
      // ===============================================

      wallTravel = 0.0f;

      previousWallX =
        robotX;

      previousWallY =
        robotY;


      state =
        WALL_FOLLOW;

      return;
    }


    // =================================================
    // GOAL CHECK
    // =================================================

    float goalDistance =
      distanceToGoal();


    if (
      goalDistance <
      80.0f
    )
    {
      setMotor(
        0,
        0
      );

      state =
        GOAL_REACHED;


      Serial.println();
      Serial.println(
        "======================================"
      );

      Serial.println(
        "GOAL REACHED"
      );

      Serial.println(
        "======================================"
      );

      return;
    }


    // =================================================
    // MOVE STRAIGHT
    // =================================================

    moveStraight();
  }


  // ===================================================
  // WALL FOLLOW
  // ===================================================

  else if (
    state ==
    WALL_FOLLOW
  )
  {
    wallFollow();
  }


  // ===================================================
  // GOAL REACHED
  // ===================================================

  else if (
    state ==
    GOAL_REACHED
  )
  {
    setMotor(
      0,
      0
    );
  }


  delay(20);
}


// =====================================================
// MOVE STRAIGHT
// =====================================================
//
// NO heading correction.
// NO goal-heading PID.
// BOTH motors receive exactly the same command.
//

void moveStraight()
{
  setMotor(
    BASE_SPEED,
    BASE_SPEED
  );


  Serial.print(
    "STRAIGHT | X="
  );

  Serial.print(
    robotX,
    1
  );

  Serial.print(
    " | Y="
  );

  Serial.print(
    robotY,
    1
  );

  Serial.print(
    " | ToF="
  );

  Serial.println(
    readToF()
  );
}


// =====================================================
// WALL FOLLOW
// =====================================================
//
// THIS IS YOUR WORKING 5B CODE.
// The PID equations and motor relationship are
// intentionally preserved.
//

void wallFollow()
{
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(
    &measure,
    false
  );


  // ===================================================
  // VALID TOF MEASUREMENT
  // ===================================================

  if (
    measure.RangeStatus != 4
  )
  {
    float distance =
      measure.RangeMilliMeter;


    // =================================================
    // ERROR
    // =================================================

    error =
      distance -
      TARGET_DISTANCE;


    // =================================================
    // TIME
    // =================================================

    unsigned long currentTime =
      micros();

    float dt =
      (
        currentTime -
        previousTime
      ) *
      1e-6f;

    previousTime =
      currentTime;


    if (
      dt <= 0.0f ||
      dt > 0.5f
    )
    {
      dt =
        0.05f;
    }


    // =================================================
    // PROPORTIONAL
    // =================================================

    float P =
      Kp *
      error;


    // =================================================
    // INTEGRAL
    // =================================================

    integral +=
      error *
      dt;

    integral =
      constrain(
        integral,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
      );

    float I =
      Ki *
      integral;


    // =================================================
    // DERIVATIVE
    // =================================================

    float derivative =
      0;

    if (
      dt > 0.001f
    )
    {
      derivative =
        (
          error -
          previousError
        ) /
        dt;
    }

    float D =
      Kd *
      derivative;


    // =================================================
    // PID OUTPUT
    // =================================================

    float output =
      P +
      I +
      D;


    previousError =
      error;


    // =================================================
    // EXACT 5B MOTOR COMMAND
    // =================================================

    int leftSpeed =
      BASE_SPEED +
      output;

    int rightSpeed =
      BASE_SPEED -
      output;


    leftSpeed =
      constrain(
        leftSpeed,
        0,
        MAX_SPEED
      );

    rightSpeed =
      constrain(
        rightSpeed,
        0,
        MAX_SPEED
      );


    // =================================================
    // APPLY MOTOR SPEED
    // =================================================

    setMotor(
      leftSpeed,
      rightSpeed
    );


    // =================================================
    // UPDATE WALL TRAVEL
    // =================================================

    float dx =
      robotX -
      previousWallX;

    float dy =
      robotY -
      previousWallY;

    float moved =
      sqrt(
        dx * dx +
        dy * dy
      );

    wallTravel +=
      moved;

    previousWallX =
      robotX;

    previousWallY =
      robotY;


    // =================================================
    // BUG2 M-LINE TEST
    // =================================================

    float goalDistance =
      distanceToGoal();

    bool mLine =
      onMLine();

    bool closerToGoal =
      goalDistance <
      (
        hitGoalDistance -
        100.0f
      );


    // =================================================
    // LEAVE WALL
    // =================================================
    //
    // Don't leave at the exact hit point.
    //
    // Must:
    // 1. travel along obstacle
    // 2. reach M-line
    // 3. be closer to goal
    //

    if (
      wallTravel >
      200.0f
    )
    {
      if (
        mLine &&
        closerToGoal
      )
      {
        Serial.println();
        Serial.println(
          "======================================"
        );

        Serial.println(
          "M-LINE FOUND"
        );

        Serial.println(
          "CLOSER TO GOAL"
        );

        Serial.println(
          "LEAVING WALL"
        );

        Serial.println(
          "======================================"
        );


        obstacleHandled =
          true;


        setMotor(
          0,
          0
        );

        delay(200);


        state =
          GO_TO_GOAL;

        return;
      }
    }


    // =================================================
    // SERIAL DATA
    // =================================================

    Serial.print(
      "WALL | "
    );

    Serial.print(
      distance
    );

    Serial.print(
      ","
    );

    Serial.print(
      error
    );

    Serial.print(
      ","
    );

    Serial.print(
      output
    );

    Serial.print(
      ","
    );

    Serial.print(
      leftSpeed
    );

    Serial.print(
      ","
    );

    Serial.print(
      rightSpeed
    );

    Serial.print(
      " | X="
    );

    Serial.print(
      robotX,
      1
    );

    Serial.print(
      " Y="
    );

    Serial.print(
      robotY,
      1
    );

    Serial.print(
      " MLine="
    );

    Serial.println(
      distanceFromMLine(),
      1
    );
  }


  // ===================================================
  // INVALID TOF
  // ===================================================

  else
  {
    Serial.println(
      "INVALID TOF - STOP"
    );

    setMotor(
      0,
      0
    );

    integral = 0;

    previousError = 0;
  }


  delay(50);
}


// =====================================================
// ToF READING
// =====================================================

float readToF()
{
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(
    &measure,
    false
  );


  if (
    measure.RangeStatus == 4
  )
  {
    return -1.0f;
  }


  return measure.RangeMilliMeter;
}


// =====================================================
// DISTANCE TO GOAL
// =====================================================

float distanceToGoal()
{
  float dx =
    goalX -
    robotX;

  float dy =
    goalY -
    robotY;


  return sqrt(
    dx * dx +
    dy * dy
  );
}


// =====================================================
// DISTANCE FROM M-LINE
// =====================================================

float distanceFromMLine()
{
  float numerator =
    fabs(
      lineA * robotX +
      lineB * robotY +
      lineC
    );


  float denominator =
    sqrt(
      lineA * lineA +
      lineB * lineB
    );


  if (
    denominator <
    0.0001f
  )
  {
    return 0.0f;
  }


  return (
    numerator /
    denominator
  );
}


// =====================================================
// M-LINE CHECK
// =====================================================

bool onMLine()
{
  return (
    distanceFromMLine() <
    50.0f
  );
}


// =====================================================
// ENCODER ISR
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
    ) /
    2.0f;


  // =================================================
  // GLOBAL HEADING
  //
  // Initial direction is the direction of the M-line.
  // Yaw is the change from that direction.
  // =================================================

  float heading =
    (
      initialHeadingDeg +
      yaw
    ) *
    DEG_TO_RAD;


  robotX +=
    distance *
    cos(
      heading
    );

  robotY +=
    distance *
    sin(
      heading
    );
}


// =====================================================
// MOTOR CONTROL
// =====================================================
//
// EXACT SAME MOTOR CONTROL AS YOUR WORKING 5B.
//

void setMotor(
  int left,
  int right
)
{
  // ---------------- Left Motor ----------------

  if (
    left >= 0
  )
  {
    digitalWrite(
      AIN1,
      HIGH
    );

    digitalWrite(
      AIN2,
      LOW
    );
  }
  else
  {
    digitalWrite(
      AIN1,
      LOW
    );

    digitalWrite(
      AIN2,
      HIGH
    );

    left =
      -left;
  }


  // ---------------- Right Motor ----------------

  if (
    right >= 0
  )
  {
    digitalWrite(
      BIN1,
      HIGH
    );

    digitalWrite(
      BIN2,
      LOW
    );
  }
  else
  {
    digitalWrite(
      BIN1,
      LOW
    );

    digitalWrite(
      BIN2,
      HIGH
    );

    right =
      -right;
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
// MPU INITIALIZATION
// =====================================================

void initMPU()
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x6B);
  Wire.write(0x01);

  Wire.endTransmission(
    true
  );

  delay(100);


  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x1A);
  Wire.write(0x03);

  Wire.endTransmission(
    true
  );


  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x1B);
  Wire.write(0x00);

  Wire.endTransmission(
    true
  );

  delay(100);
}


// =====================================================
// GYRO CALIBRATION
// =====================================================

void calibrateGyro()
{
  float sum =
    0.0f;


  for (
    int i = 0;
    i < CALIB_SAMPLES;
    i++
  )
  {
    readGyro();

    sum +=
      gyroZ;

    delay(
      CALIB_DELAY_MS
    );
  }


  gyroZOffset =
    sum /
    CALIB_SAMPLES;


  Serial.print(
    "Gyro Z Offset = "
  );

  Serial.println(
    gyroZOffset,
    4
  );
}


// =====================================================
// READ GYRO
// =====================================================

void readGyro()
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x43);

  Wire.endTransmission(
    false
  );


  Wire.requestFrom(
    (uint8_t)MPU_ADDR,
    (uint8_t)6,
    (uint8_t)true
  );


  Wire.read();
  Wire.read();

  Wire.read();
  Wire.read();


  int16_t gz =
    (
      Wire.read() << 8
    ) |
    Wire.read();


  gyroZ =
    gz /
    GYRO_SCALE;
}


// =====================================================
// UPDATE IMU
// =====================================================

void updateIMU()
{
  unsigned long currentTime =
    micros();


  float dt =
    (
      currentTime -
      imuPreviousTime
    ) *
    1e-6f;


  imuPreviousTime =
    currentTime;


  if (
    dt <= 0.0f ||
    dt > 0.5f
  )
  {
    dt =
      0.005f;
  }


  readGyro();


  gyroZ -=
    gyroZOffset;


  if (
    fabs(
      gyroZ
    ) <
    GYRO_DEADBAND
  )
  {
    gyroZ =
      0.0f;
  }


  yaw +=
    gyroZ *
    dt;


  yaw =
    wrap180(
      yaw
    );
}


// =====================================================
// ANGLE WRAP
// =====================================================

float wrap180(
  float angle
)
{
  while (
    angle >
    180.0f
  )
  {
    angle -=
      360.0f;
  }


  while (
    angle <
    -180.0f
  )
  {
    angle +=
      360.0f;
  }


  return angle;
}
