#include <Wire.h>

#define MPU_ADDR 0x68

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

#define CALIB_SAMPLES 3000
#define CALIB_DELAY_MS 2

#define ALPHA 0.96f
#define GYRO_DEADBAND 0.5f

// -------------------- Motor Pins --------------------

#define PWMA 26
#define AIN1 27
#define AIN2 14

#define PWMB 32
#define BIN1 33
#define BIN2 12

#define STBY 25

// -------------------- Rotation --------------------

#define TARGET_ANGLE 90.0f

#define ROTATE_SPEED 70

#define ANGLE_TOLERANCE 2.0f

// -------------------- IMU Variables --------------------

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off = 0;
float accY_off = 0;
float accZ_off = 0;

float gyroX_off = 0;
float gyroY_off = 0;
float gyroZ_off = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

float accRoll;
float accPitch;

float dt;

unsigned long prevTime;

bool finished = false;

// -------------------- Function Prototypes --------------------

void initMPU();
void calibrateMPU();
void readMPU();
void applyOffsets();
void updateIMU();

void setMotor(int left, int right);

float wrap180(float angle);

// -------------------- Setup --------------------

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================");
    Serial.println("ESP32 RESTARTED");
    Serial.println("========================");
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);

    ledcAttach(PWMA, 20000, 8);
    ledcAttach(PWMB, 20000, 8);

    Wire.begin(21,22);
    Wire.setClock(400000);

    initMPU();

    delay(2000);

    Serial.println("Calibrating...");

    calibrateMPU();

    prevTime = micros();

    updateIMU();

    yaw = 0;

    Serial.println("Starting Rotation...");
}

// -------------------- Loop --------------------

void loop()
{
    if(finished)
    {
        setMotor(0,0);
        return;
    }

    updateIMU();

    Serial.print("Yaw : ");
    Serial.println(yaw);

    if(yaw < TARGET_ANGLE - ANGLE_TOLERANCE)
    {
        setMotor(ROTATE_SPEED,-ROTATE_SPEED);
    }
    else
    {
        setMotor(0,0);

        finished = true;

        Serial.println("Reached Target");
    }

    delay(10);
}// -------------------- Motor Control --------------------

void setMotor(int left, int right)
{
    if(left >= 0)
    {
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
    }
    else
    {
        digitalWrite(AIN1,LOW);
        digitalWrite(AIN2,HIGH);
        left = -left;
    }

    if(right >= 0)
    {
        digitalWrite(BIN1,HIGH);
        digitalWrite(BIN2,LOW);
    }
    else
    {
        digitalWrite(BIN1,LOW);
        digitalWrite(BIN2,HIGH);
        right = -right;
    }

    left  = constrain(left,0,255);
    right = constrain(right,0,255);

    ledcWrite(PWMA,left);
    ledcWrite(PWMB,right);
}

// -------------------- IMU Update --------------------

void updateIMU()
{
    unsigned long now = micros();

    dt = (now - prevTime) * 1e-6f;

    prevTime = now;

    if(dt <= 0 || dt > 0.5)
        dt = 0.005;

    readMPU();

    applyOffsets();

    accRoll =
        atan2(accY,accZ) *
        RAD_TO_DEG;

    accPitch =
        atan2(
            -accX,
            sqrt(accY*accY + accZ*accZ)
        ) *
        RAD_TO_DEG;

    roll =
        ALPHA *
        (roll + gyroX*dt)
        +
        (1-ALPHA) *
        accRoll;

    pitch =
        ALPHA *
        (pitch + gyroY*dt)
        +
        (1-ALPHA) *
        accPitch;

    float gz = gyroZ;

    if(fabs(gz) < GYRO_DEADBAND)
        gz = 0;

    yaw += gz * dt;

    yaw = wrap180(yaw);
}

// -------------------- MPU Init --------------------

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

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    delay(200);
}// -------------------- Calibration --------------------

void calibrateMPU()
{
    float sAX = 0;
    float sAY = 0;
    float sAZ = 0;

    float sGX = 0;
    float sGY = 0;
    float sGZ = 0;

    for(int i=0;i<CALIB_SAMPLES;i++)
    {
        readMPU();

        sAX += accX;
        sAY += accY;
        sAZ += accZ;

        sGX += gyroX;
        sGY += gyroY;
        sGZ += gyroZ;

        delay(CALIB_DELAY_MS);
    }

    accX_off = sAX / CALIB_SAMPLES;
    accY_off = sAY / CALIB_SAMPLES;
    accZ_off = (sAZ / CALIB_SAMPLES) - 1.0f;

    gyroX_off = sGX / CALIB_SAMPLES;
    gyroY_off = sGY / CALIB_SAMPLES;
    gyroZ_off = sGZ / CALIB_SAMPLES;
}

// -------------------- Read MPU --------------------

void readMPU()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(uint8_t)true);

    accX =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        ACCEL_SCALE;

    accY =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        ACCEL_SCALE;

    accZ =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        ACCEL_SCALE;

    Wire.read();
    Wire.read();

    gyroX =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        GYRO_SCALE;

    gyroY =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        GYRO_SCALE;

    gyroZ =
        (int16_t)(Wire.read()<<8 | Wire.read()) /
        GYRO_SCALE;
}

// -------------------- Apply Offsets --------------------

void applyOffsets()
{
    accX -= accX_off;
    accY -= accY_off;
    accZ -= accZ_off;

    gyroX -= gyroX_off;
    gyroY -= gyroY_off;
    gyroZ -= gyroZ_off;
}

// -------------------- Wrap Angle --------------------

float wrap180(float angle)
{
    while(angle > 180.0f)
        angle -= 360.0f;

    while(angle < -180.0f)
        angle += 360.0f;

    return angle;
}
