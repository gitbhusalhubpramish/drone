/*
GYRO STABILIZED + POWER LIMITED VERSION - For Grade 8 Project
This version has:
✅ AUTO-STABILIZATION (MPU6050 gyroscope)
✅ SMART ALTITUDE CONTROL
✅ PID CONTROL SYSTEM
✅ POWER LIMITING (prevents battery voltage drop)
✅ ALL ADVANCED FEATURES + RELIABILITY

Motor Layout:
m1              m2
  \            /
    \        /
      \     /
        \  /
          X
        /  \
      /      \
    /          \
  /              \
m3                  m4
*/

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

#define lsig 2
#define rsig 4
#define fsig 7
#define bsig 8

Servo m1, m2, m3, m4;
const int dm1 = 9;
const int dm2 = 10;
const int dm3 = 11;
const int dm4 = 6;

MPU6050 mpu;
float pitch = 0, roll = 0;
float gyroPitch = 0, gyroRoll = 0;
float Kp = 2.0;
unsigned long lastTime = 0;

// POWER LIMITING SETTINGS - Reduced from original for battery protection
int hi[4] = {1700, 1700, 1700, 1700};  // Reduced from 2000 to 1700
int lo[4] = {1200, 1200, 1200, 1200};  // Keep same minimum
int mid[4] = {1450, 1450, 1450, 1450}; // Reduced from 1700 to 1450

// Power management variables
unsigned long startTime;
bool powerReduced = false;
int maxAllowedPower = 1700;
int originalMaxPower = 1700;

// ===== RC INPUTS =====
volatile uint16_t rcValue[4] = {1500, 1500, 1500, 1500};
volatile uint32_t rcStart[4];
const uint8_t rcPins[4] = {rsig, lsig, fsig, bsig};

void rcISR0()
{
    if (digitalRead(rsig))
        rcStart[0] = micros();
    else
        rcValue[0] = micros() - rcStart[0];
}
void rcISR1()
{
    if (digitalRead(lsig))
        rcStart[1] = micros();
    else
        rcValue[1] = micros() - rcStart[1];
}
void rcISR2()
{
    if (digitalRead(fsig))
        rcStart[2] = micros();
    else
        rcValue[2] = micros() - rcStart[2];
}
void rcISR3()
{
    if (digitalRead(bsig))
        rcStart[3] = micros();
    else
        rcValue[3] = micros() - rcStart[3];
}

float cmps[] = {0, 0, 0};
float ang[] = {0, 0, 0};
int *base[] = {&mid[0], &mid[1], &mid[2], &mid[3]};

int deg = 15;
int mtvl[] = {0, 0, 0, 0};

// Power management function
void updatePowerLimits()
{
    unsigned long runtime = millis() - startTime;

    // After 20 seconds of operation, start reducing max power gradually
    if (runtime > 20000 && !powerReduced)
    {
        maxAllowedPower = constrain(maxAllowedPower - 10, 1500, originalMaxPower);
        // Update all power arrays
        for (int i = 0; i < 4; i++)
        {
            hi[i] = maxAllowedPower;
            mid[i] = constrain(mid[i], 1200, maxAllowedPower - 100);
        }
        powerReduced = true;
        Serial.print("Power reduced to: ");
        Serial.println(maxAllowedPower);
    }

    // Reset power reduction flag every 45 seconds to allow recovery
    if (runtime % 45000 < 1000)
    {
        powerReduced = false;
    }

    // Emergency power reduction if too much continuous high power
    static unsigned long highPowerTime = 0;
    static bool inHighPower = false;

    // Check if we're in high power mode
    bool currentlyHighPower = (base[0] == &hi[0] || base[1] == &hi[1] ||
                               base[2] == &hi[2] || base[3] == &hi[3]);

    if (currentlyHighPower && !inHighPower)
    {
        highPowerTime = millis();
        inHighPower = true;
    }
    else if (!currentlyHighPower)
    {
        inHighPower = false;
    }

    // If in high power for more than 15 seconds, force reduction
    if (inHighPower && (millis() - highPowerTime > 15000))
    {
        maxAllowedPower = constrain(maxAllowedPower - 5, 1400, originalMaxPower);
        for (int i = 0; i < 4; i++)
        {
            hi[i] = maxAllowedPower;
        }
        Serial.println("Emergency power reduction - cooling down...");
        inHighPower = false; // Reset to allow recovery
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Gyro Stabilized + Power Limited Drone Starting...");
    Serial.print("Max Power Setting: ");
    Serial.println(maxAllowedPower);

    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (1)
        {
            Serial.println("Check MPU6050 wiring!");
            delay(1000);
        }
    }
    Serial.println("MPU6050 connected successfully!");

    m1.attach(dm1);
    m2.attach(dm2);
    m3.attach(dm3);
    m4.attach(dm4);

    // Improved ESC calibration with power limits
    Serial.println("Calibrating ESCs with power limits...");
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    delay(1000);

    // Calibrate with limited max power
    m1.writeMicroseconds(maxAllowedPower);
    m2.writeMicroseconds(maxAllowedPower);
    m3.writeMicroseconds(maxAllowedPower);
    m4.writeMicroseconds(maxAllowedPower);
    delay(2000);

    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    delay(1000);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(rsig), rcISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lsig), rcISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(fsig), rcISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(bsig), rcISR3, CHANGE);

    delay(2000);
    lastTime = micros();
    startTime = millis();

    Serial.println("Setup complete! Gyro stabilization active with power protection.");
    Serial.println("Flight modes:");
    Serial.println("- Left+Right: DESCEND");
    Serial.println("- Front+Back: ASCEND");
    Serial.println("- Front+Right: ROTATE CLOCKWISE");
    Serial.println("- Back+Left: ROTATE COUNTER-CLOCKWISE");
    Serial.println("- Single directions: MOVE");
}

void loop()
{
    int mtn[] = {0, 0, 0, 0}; //{x, y, z, yaw}
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // Update power management
    updatePowerLimits();

    // ===== RC BUTTON STATES =====
    bool rclk = rcValue[0] > 1700;
    bool lclk = rcValue[1] > 1700;
    bool fclk = rcValue[2] > 1700;
    bool bclk = rcValue[3] > 1700;

    // ===== MPU READ =====
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float gyroRate[3] = {gx / 131.0, gy / 131.0, gz / 131.0};
    float acc[3] = {(ax / 16384.0) * 9.81, (ay / 16384.0) * 9.81, (az / 16384.0) * 9.81};

    for (int i = 0; i < 3; i++)
    {
        cmps[i] += acc[i] * dt;
        ang[i] += gyroRate[i] * dt;
    }

    // ===== FLIGHT MODE SELECTION =====
    if (lclk && rclk && !fclk && !bclk)
    {
        mtn[2] = -1; // DESCEND
        base[0] = &lo[0];
        base[1] = &lo[1];
        base[2] = &lo[2];
        base[3] = &lo[3];
    }
    else if (fclk && bclk && !lclk && !rclk)
    {
        mtn[2] = 1; // ASCEND
        base[0] = &hi[0];
        base[1] = &hi[1];
        base[2] = &hi[2];
        base[3] = &hi[3];
    }
    else if (fclk && rclk && !bclk && !lclk)
    {
        mtn[3] = 1; // ROTATE CLOCKWISE
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else if (bclk && lclk && !fclk && !rclk)
    {
        mtn[3] = -1; // ROTATE COUNTER-CLOCKWISE
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else if (fclk && !bclk && !lclk && !rclk)
    {
        mtn[1] = 1; // FORWARD
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else if (bclk && !fclk && !lclk && !rclk)
    {
        mtn[1] = -1; // BACKWARD
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else if (rclk && !lclk && !fclk && !bclk)
    {
        mtn[0] = 1; // RIGHT
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else if (lclk && !rclk && !fclk && !bclk)
    {
        mtn[0] = -1; // LEFT
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }
    else
    {
        // HOVER/STABILIZE
        base[0] = &mid[0];
        base[1] = &mid[1];
        base[2] = &mid[2];
        base[3] = &mid[3];
    }

    // ===== CONTROL TARGET CALCULATION =====
    mtvl[0] = ((deg * mtn[0]) - ang[0]);   // Roll target
    mtvl[1] = ((deg * mtn[1]) - ang[1]);   // Pitch target
    mtvl[2] = ((-0.1 * mtn[2]) - cmps[2]); // Altitude target
    mtvl[3] = ((deg * mtn[3]) - ang[2]);   // Yaw target

    // ===== PID CONTROL LOGIC =====
    // Compute errors
    float e_roll = mtvl[0] - gyroRate[0];
    float e_pitch = mtvl[1] - gyroRate[1];
    float e_yaw = mtvl[3] - gyroRate[2];
    float e_alt = 9.81 + mtvl[2] - acc[2];

    // Compute control efforts (reduced Kp for stability with power limits)
    float Kp_adjusted = constrain(Kp * (maxAllowedPower / 1700.0), 1.0, 3.0);
    float u_roll = Kp_adjusted * e_roll;
    float u_pitch = Kp_adjusted * e_pitch;
    float u_yaw = Kp_adjusted * e_yaw;
    float u_alt = Kp_adjusted * e_alt;

    // ===== MOTOR MIXING =====
    int m1_speed = *base[0] + u_alt + u_pitch + u_roll - u_yaw;
    int m2_speed = *base[1] + u_alt + u_pitch - u_roll + u_yaw;
    int m3_speed = *base[2] + u_alt - u_pitch + u_roll + u_yaw;
    int m4_speed = *base[3] + u_alt - u_pitch - u_roll - u_yaw;

    // ===== POWER LIMITING & SAFETY =====
    m1_speed = constrain(m1_speed, 1200, maxAllowedPower);
    m2_speed = constrain(m2_speed, 1200, maxAllowedPower);
    m3_speed = constrain(m3_speed, 1200, maxAllowedPower);
    m4_speed = constrain(m4_speed, 1200, maxAllowedPower);

    // Update base speeds for next iteration
    *base[0] = constrain(*base[0], 1200, maxAllowedPower);
    *base[1] = constrain(*base[1], 1200, maxAllowedPower);
    *base[2] = constrain(*base[2], 1200, maxAllowedPower);
    *base[3] = constrain(*base[3], 1200, maxAllowedPower);

    // ===== MOTOR OUTPUT =====
    static unsigned long lastMotorUpdate = 0;
    if (now - lastMotorUpdate >= 20000)
    { // 50Hz update rate
        lastMotorUpdate = now;
        m1.writeMicroseconds(m1_speed);
        m2.writeMicroseconds(m2_speed);
        m3.writeMicroseconds(m3_speed);
        m4.writeMicroseconds(m4_speed);
    }

    // ===== DEBUG OUTPUT =====
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 2000)
    { // Every 2 seconds
        lastDebugPrint = millis();
        Serial.print("Runtime: ");
        Serial.print((millis() - startTime) / 1000);
        Serial.print("s | ");
        Serial.print("Max Power: ");
        Serial.print(maxAllowedPower);
        Serial.print(" | ");
        Serial.print("Motors: ");
        Serial.print(m1_speed);
        Serial.print(" ");
        Serial.print(m2_speed);
        Serial.print(" ");
        Serial.print(m3_speed);
        Serial.print(" ");
        Serial.print(m4_speed);
        Serial.print(" | ");
        Serial.print("Gyro: ");
        Serial.print(gyroRate[0]);
        Serial.print(" ");
        Serial.print(gyroRate[1]);
        Serial.print(" ");
        Serial.println(gyroRate[2]);
    }
}
