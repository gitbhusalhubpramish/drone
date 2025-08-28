# 📘 Quadcopter Drone Project – ICT Club of NJBS

## 🎯 Executive Summary

This project is an initiative by the **ICT Club of NJBS** to design, build, and test a **quadcopter drone** using **Arduino Uno, ESCs, BLDC motors, an RC receiver, and an MPU6050 IMU sensor**.

The drone demonstrates how school-level students can explore **STEM (Science, Technology, Engineering, Mathematics)** by integrating **electronics, programming, and aerodynamics**. The project not only focuses on building a functional drone but also aims to **inspire creativity, teamwork, and problem-solving** among ICT club members.

---

## 👥 Team & Credits

### **Supervision & Guidance**

- **Rajendra Pantha** – Project Supervisor & Guide

### **Leadership**

- **Pramish Bhusal** – Project Coordinator

### **Hardware Assembly**

- **Sangam Kunwar** – Hardware Assembler
- **Abishek Dhakal** – Hardware Assembler

### **Support & Documentation**

- **Sugam Bhusal** – Support & Documentation
- **Madhu Kunwar** – Support & Documentation

### **Documentation Assistants**

- **Puspa Bhattrai** – Documentation Assistant
- **Sonakshi Ranpal** – Documentation Assistant
- **Nischal Dhakal** – Documentation Assistant

### **Support Assistants**

- **Kishore Poudel** – Support Assistant
- **Sagar Ale Magar** – Support Assistant

![Group photo of ICT club of NJBS.jpg](Group_photo_of_ICT_club_of_NJBS.jpg)

---

## 🎯 Project Overview

### **Objective**

The primary goal of this project is to build a **functional quadcopter drone** controlled via an **RC transmitter and receiver**, using an **Arduino Uno** as the flight control unit.

### **Scope**

- To design and assemble the drone frame and hardware components
- To integrate sensors and electronics for balance and control
- To develop Arduino code for motor control and sensor data processing
- To perform testing, troubleshooting, and safe flight trials

### **Expected Outcomes**

- A working quadcopter capable of stable manual flight
- Detailed documentation for future ICT club batches
- A foundation for future projects (autonomous drones, GPS navigation, camera integration)

---

## 📦 Bill of Materials (BOM)

| **Component** | **Quantity** | **Description / Notes** | **Approx. Cost (NPR)** |
| --- | --- | --- | --- |
| Arduino Uno R3 | 1 | Main flight controller | 1,500 |
| ESC 30A | 4 | Electronic speed controllers | 4,000 (set of 4) |
| BLDC Motors (1000KV) | 4 | Propulsion system | 5,000 (set of 4) |
| Propellers (10x4.5) | 4 | CW and CCW pair | 1,000 |
| RC Transmitter & Rx | 1 set | 4-channel RC system | 7,000 |
| MPU6050 IMU | 1 | Gyroscope & accelerometer sensor | 500 |
| LiPo Battery (35C 2200mAh) | 1 | Power supply | 3,500 |

**Total Estimated Cost: ~28,000–30,000 NPR**

---

## 🔧 Hardware Design

### **Frame Configuration**

- **Frame:** 450mm quadcopter frame (X configuration)

### **Motor Layout:**

- **M1** – Front Left (CW)
- **M2** – Front Right (CCW)
- **M3** – Back Left (CCW)
- **M4** – Back Right (CW)

### **Electronics Placement:**

- Arduino + MPU6050 mounted centrally for balance
- ESCs fixed on arms for cooling
- Battery mounted at the bottom with strap
- Receiver fixed on top frame

---

## ⚡ Electronics & Schematics

### **Wiring Diagram**

<aside>
🔌

**Connection Overview:**

- RC Receiver → Arduino Analog Pins (A1–A4)
- Arduino Digital Pins (3, 5, 6, 9) → ESC signal inputs
- ESCs → BLDC Motors
- LiPo Battery → Power Distribution Board → ESCs + Arduino (via 5V BEC)
- MPU6050 → Arduino I2C (SDA/SCL on A4/A5)
</aside>

---

## 💻 Software

### **Overview**

- Reads RC receiver PWM signals
- Controls ESCs using Arduino Servo library
- Reads IMU data (acceleration, gyro) from MPU6050
- Logs outputs for debugging via Serial Monitor

### **Example Arduino Code (Safe Test)**

```cpp
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// ===== DRONE MOTOR LAYOUT =====
// m1 - Front Left  (CW)
// m2 - Front Right (CCW)
// m3 - Back Left   (CCW)
// m4 - Back Right  (CW)

#define lsig 2
#define rsig 4
#define fsig 7
#define bsig 8

Servo m1, m2, m3, m4;
const int dm1 = 9;
const int dm2 = 10;
const int dm3 = 11;
const int dm4 = 6;

// ===== MPU VARIABLES =====
MPU6050 mpu;
float pitch = 0, roll = 0;
float gyroPitch = 0, gyroRoll = 0;
float Kp = 2.0;
unsigned long lastTime = 0;

// ===== ACCEL Z-SMOOTHING =====
float accZ = 0;
float accZ_filt = 0;
float accZ_offset = 0;
float accZ_gain = 1.0;

// ===== ALTITUDE CONTROL =====
float targetAccZ = 0;
float accKp = 100.0;
float accKi = 10.0;
float accZ_integral = 0;

// ===== RC INPUTS =====
volatile uint16_t rcValue[4] = {1500, 1500, 1500, 1500};
volatile uint32_t rcStart[4];
const uint8_t rcPins[4] = {rsig, lsig, fsig, bsig};

// ===== SPEEDS =====
int hi = 2000, lo = 1000, mid = 1500;

// ===== INTERRUPT HANDLER =====
void rcISR0() { if(digitalRead(rsig)) rcStart[0] = micros(); else rcValue[0] = micros() - rcStart[0]; }
void rcISR1() { if(digitalRead(lsig)) rcStart[1] = micros(); else rcValue[1] = micros() - rcStart[1]; }
void rcISR2() { if(digitalRead(fsig)) rcStart[2] = micros(); else rcValue[2] = micros() - rcStart[2]; }
void rcISR3() { if(digitalRead(bsig)) rcStart[3] = micros(); else rcValue[3] = micros() - rcStart[3]; }

// ===== THRUST SMOOTHING =====
int currentThrust = 0;
int thrustStep = 2;

// ===== DRIFT COMPENSATION =====
float driftGain = 0.3; // auto motor balance gain

void setup() {
    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    if(!mpu.testConnection()) while(1);

    m1.attach(dm1);
    m2.attach(dm2);
    m3.attach(dm3);
    m4.attach(dm4);

    // Arm ESCs
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);
    delay(2000);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(rsig), rcISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lsig), rcISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(fsig), rcISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(bsig), rcISR3, CHANGE);

    lastTime = micros();

    // ===== ACCEL Z CALIBRATION =====
    float azsum = 0;
    int n = 400;
    for(int i=0;i<n;i++){
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        azsum += (float)az;
        delay(5);
    }
    accZ_offset = azsum / n;

    // ===== Tiny startup lift =====
    int startupThrust = 1200;
    unsigned long startTime = micros();
    while(micros() - startTime < 200000){ // ~0.2s
        m1.writeMicroseconds(startupThrust);
        m2.writeMicroseconds(startupThrust);
        m3.writeMicroseconds(startupThrust);
        m4.writeMicroseconds(startupThrust);
        delay(20);
    }
}

void loop(){
    unsigned long now = micros();
    float dt = (now - lastTime)/1000000.0;
    lastTime = now;

    // ===== RC BUTTON STATES =====
    bool rclk = rcValue[0] > 1700;
    bool lclk = rcValue[1] > 1700;
    bool fclk = rcValue[2] > 1700;
    bool bclk = rcValue[3] > 1700;

    // ===== MPU READ =====
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accPitch = atan2(ax, sqrt(ay*ay + az*az))*180/PI;
    float accRoll  = atan2(ay, sqrt(ax*ax + az*az))*180/PI;

    float gyroRateX = gx/131.0;
    float gyroRateY = gy/131.0;

    gyroPitch += gyroRateY*dt;
    gyroRoll  -= gyroRateX*dt;

    pitch = 0.98*gyroPitch + 0.02*accPitch;
    roll  = 0.98*gyroRoll  + 0.02*accRoll;

    // ===== ACCEL Z CONTROL =====
    accZ = ((float)az - accZ_offset) * accZ_gain;
    accZ_filt = 0.9*accZ_filt + 0.1*accZ;

    // ===== MOVEMENT PARAMETERS =====
    float desiredAngle = 15.0;
    int yawAmount = 150;
    int maxThrustUp = 300;
    int maxThrustDn = -300;

    float targetPitch = 0;
    float targetRoll  = 0;
    int targetThrust = 0;
    int yaw = 0;

    // ===== RC COMMANDS =====
    if(lclk && rclk && !fclk && !bclk) targetThrust = maxThrustDn;
    else if(fclk && bclk && !lclk && !rclk) targetThrust = maxThrustUp;
    else if(fclk && rclk && !bclk && !lclk) yaw = yawAmount;
    else if(bclk && lclk && !fclk && !rclk) yaw = -yawAmount;
    else if(fclk && !bclk && !lclk && !rclk) targetPitch = -desiredAngle;
    else if(bclk && !fclk && !lclk && !rclk) targetPitch = desiredAngle;
    else if(rclk && !lclk && !fclk && !bclk) targetRoll = desiredAngle;
    else if(lclk && !rclk && !fclk && !bclk) targetRoll = -desiredAngle;

    // ===== THRUST RAMPING =====
    if(currentThrust < targetThrust) currentThrust += thrustStep;
    else if(currentThrust > targetThrust) currentThrust -= thrustStep;

    // ===== VERTICAL ACCEL PID =====
    float accZ_error = 0;
    float accZ_out = 0;
    if(targetThrust != 0){
        accZ_error = (targetThrust>0?1.0:-1.0) - accZ_filt/16384.0;
        accZ_integral += accZ_error*dt;
        accZ_out = accKp*accZ_error + accKi*accZ_integral;
        accZ_out = constrain(accZ_out,maxThrustDn,maxThrustUp);
    } else accZ_integral = 0;

    int thrust = currentThrust + (int)accZ_out;

    // ===== BASE SPEEDS =====
    int base1 = mid + thrust;
    int base2 = mid + thrust;
    int base3 = mid + thrust;
    int base4 = mid + thrust;

    // Yaw mixing
    base1 += yaw; base4 += yaw;
    base2 -= yaw; base3 -= yaw;

    // ===== CORRECTIONS =====
    int pitchCorrection = Kp*(pitch - targetPitch);
    int rollCorrection  = Kp*(roll - targetRoll);

    int sp1 = constrain(base1 - pitchCorrection + rollCorrection,1000,2000);
    int sp2 = constrain(base2 - pitchCorrection - rollCorrection,1000,2000);
    int sp3 = constrain(base3 + pitchCorrection + rollCorrection,1000,2000);
    int sp4 = constrain(base4 + pitchCorrection - rollCorrection,1000,2000);

    // ===== IN-FLIGHT AUTO MOTOR BALANCE =====
    float pitchDrift = pitch - targetPitch;
    float rollDrift  = roll - targetRoll;

    sp1 -= pitchDrift*driftGain - rollDrift*driftGain;
    sp2 -= pitchDrift*driftGain + rollDrift*driftGain;
    sp3 += pitchDrift*driftGain - rollDrift*driftGain;
    sp4 += pitchDrift*driftGain + rollDrift*driftGain;

    // ===== SEND TO MOTORS =====
    static unsigned long lastMotorUpdate = 0;
    if(now - lastMotorUpdate >= 20000){
        lastMotorUpdate = now;
        m1.writeMicroseconds(sp1);
        m2.writeMicroseconds(sp2);
        m3.writeMicroseconds(sp3);
        m4.writeMicroseconds(sp4);
    }
}
```

---

## 🧪 Testing & Safety

### **ESC Calibration Process**

1. Power RC transmitter
2. Set throttle to max
3. Connect ESC power → wait for beeps
4. Lower throttle → ESC calibrated

### **Safety Checklist**

<aside>
⚠️

**Critical Safety Measures:**

- Test without propellers first
- Ensure common ground between ESCs and Arduino
- Secure all wiring with zip ties
- Keep a fire extinguisher nearby when testing batteries
</aside>

---

## 🔧 Troubleshooting

| **Issue** | **Possible Cause** | **Solution** |
| --- | --- | --- |
| Two motors not spinning | Incorrect wiring / ESC not armed | Check wiring, re-calibrate ESCs |
| Drone unstable in flight | Wrong propeller orientation | Check CW/CCW propellers |
| No RC signal detected | Wrong receiver wiring | Verify VCC, GND, signal pins |

---

## 📊 Results & Observations

- **Successful bench testing** with motor control from RC input
- **MPU6050 provides stable sensor data**
- **Flight test:** This image captures the **first testing phase of our quadcopter drone**. The drone is placed on the floor and powered on with all four BLDC motors spinning. A team member is seen holding the frame lightly to **prevent uncontrolled lift-off during initial stability testing**.

![testing photo of drone.png](testing_photo_of_drone.png)

---

---

## 🙏 Acknowledgements

We express our sincere gratitude to Nawa Jyoti English Baording School and **ICT Club of NJBS** for providing resources and guidance. Special thanks to **Rajendra Pantha Sir** for supervision and encouragement.

---

## 📋 Appendix

- Full Arduino Code
- Component datasheets
- Flight logs (if available)

---

<aside>
🏫

**Project Details:**

- **Institution:** Nawa Jyoti English Boarding School, Tilottama-8, Charnumber
- **Club:** ICT Club of NJBS
- **Date:** August 2025
- **Motto:** *"Innovation through Collaboration"*
</aside>
