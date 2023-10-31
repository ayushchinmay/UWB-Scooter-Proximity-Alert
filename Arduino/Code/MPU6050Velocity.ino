/**
 *  Project: MPU6050 Velocity Measurement
 *
 *  Description: Uses the MPU6050 9-DoF Inertial Measurement Unit to calculate linear velocity, and heading
 *
 *  Author          : Ayush Chinmay
 *  Date Created    : 19 Oct 2023
 *  Date Modified   : 25 Oct 2023
 *
 *  NOTES:
 *      - Keep the velocity
 *
 *  CIRCUIT:
 *    - |  ESP32   |  MPU6050  |
 *    - |----------|-----------|
 *    - |    3V3   |    VCC    |
 *    - |    GND   |    GND    |
 *    - |  GPIO22  |    SCL    |
 *    - |  GPIO21  |    SDA    |
 *
 *  CHANGELOG:
 *      [25 Oct 2023]
 *            [-] Initialize and Calibrate the MPU6050
 *            [-] Obtain Data from MPU6050
 *        [-] Calcualte Velcity by integrating Acceleration over time.
 *          velocity = prev_velocity + (lin_accel - drift_error) * delta_time
 *        [-] Digital Motion Processor outputs enabled for calibration
 *
 *  TODO:
 *        
 **/

/** ==========[ LIBRARIES ]========== **/
#include <MPU6050_6Axis_MotionApps612.h>
#include <I2Cdev.h>
#include <Wire.h>

/** ==========[ DEFINES ]========== **/
#define ADC_BITS    32768
#define INTERRUPT_PIN 2
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

/** ==========[ OBJECTS ]========== **/
MPU6050 mpu;

/** ==========[ CONSTANTS ]========== **/
const float FS_ACCEL = 2;           // Accel Range: ±2, ±4, ±8, ±16  [g]
const float FS_GYRO = 250;          // Gyro Range: ±250, ±500, ±1000, ±2000  [dps]
float LSB_ACCEL, LSB_GYRO;      // LSB Division Factor [LSB/g] | [LSB/dps] | [LSB/uT]

/** ==========[ VARIABLES ]========== **/
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// velocity calculation
VectorFloat velocity;
double vel_drift[3];

/** ==========[ SETUP ]========== **/
void setup() {
    Serial.begin(115200);
    Wire.begin();

    initMPU();
}

/** ==========[ LOOP ]========== **/
void loop() {
    readMPU();
    delay(100);
}

/** ==========[ DMP DATA READY ]========== **
 *  INTERRUPT DETECTION ROUTINE
 */
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/** ==========[ INITIALIZE MPU6050 ]========== **
 *  Initializes and sets up the MPU6050 IMU
 */
void initMPU() {
    Serial.println("[INFO] INITIALIZING MPU6050...");
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // Set Full Scale Range and calculate LSB division factor
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    LSB_ACCEL = ADC_BITS/FS_ACCEL;
    LSB_GYRO = ADC_BITS/FS_GYRO;

    Serial.print("\t- ADDRESS     : 0x"); Serial.println(mpu.getDeviceID(), HEX);
    Serial.print("\t- ACCEL RANGE : ±"); Serial.print((int)FS_ACCEL); Serial.println("\t[g]");
    Serial.print("\t- GYRO RANGE  : ±"); Serial.print((int)FS_GYRO); Serial.println("\t[dps]");
    Serial.print("\t- ACCEL LSB   : "); Serial.print(LSB_ACCEL); Serial.println("\t[LSB/g]");
    Serial.print("\t- GYRO LSB    : "); Serial.print(LSB_GYRO); Serial.println("\t[LSB/dps]");

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        Serial.print("[INFO] CALIBRATING ACCEL/GYRO...");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("COMPLETE!");
        // mpu.PrintActiveOffsets();

        // turn on the DMP, now that it's ready
        Serial.println(F("[INFO] Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("\t- Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("\t-  DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("[ERROR] DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    Serial.println("[INFO] INITIALIZATION COMPLETE\n");
}

/** ==========[ READ MPU6050 ]========== **
 *  Read MPU Raw values and calculate the real world values
 */
void readMPU() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        // Calculate Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Calculate real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        calcVelocity();

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/** ==========[ CALCULATE INSTANT VELOCITY ]========== **
 *  Calculate the instantaneous velocity by integratig acceleration
 *  V = U + A*dT
 */
unsigned long timer = 0;
double dT;
void calcVelocity() {
    dT = (millis() - timer)/1000.0;
    timer = millis();

    velocity.x += (9.8*aaReal.x/LSB_ACCEL - vel_drift[0])*dT;
    velocity.y += (9.8*aaReal.y/LSB_ACCEL - vel_drift[1])*dT;
    velocity.z += (9.8*aaReal.z/LSB_ACCEL - vel_drift[2])*dT;

    vel_drift[0] = 0.99*vel_drift[0] + 0.02*(9.8*aaReal.x/LSB_ACCEL);
    vel_drift[1] = 0.99*vel_drift[1] + 0.02*(9.8*aaReal.y/LSB_ACCEL);
    vel_drift[2] = 0.98*vel_drift[2] + 0.02*(9.8*aaReal.z/LSB_ACCEL);

    printMPU();
}

/** ==========[ PRINT MPU6050 ]========== **
 *  Print the MPU data
 */
void printMPU() {
    Serial.print("ACC:\t"); Serial.print(9.8*aaReal.x/LSB_ACCEL); Serial.print(" \t ");
    Serial.print(9.8*aaReal.y/LSB_ACCEL); Serial.print(" \t "); Serial.print(9.8*aaReal.z/LSB_ACCEL); Serial.print("\t|\t");

    Serial.print("YPR:\t"); Serial.print(ypr[0] * 180/M_PI); Serial.print(" \t ");
    Serial.print(ypr[1] * 180/M_PI); Serial.print(" \t "); Serial.print(ypr[2] * 180/M_PI); Serial.print("\t|\t");

    Serial.print("VEL:\t"); Serial.print(velocity.x); Serial.print(" \t ");
    Serial.print(velocity.y); Serial.print(" \t "); Serial.print(velocity.z);
    Serial.print("\t("); Serial.print(dT); Serial.println(")\t|\t");
}
// void printMPU() {
//     Serial.print("ACCEL:\t"); Serial.print(accel[0]); Serial.print(" \t ");
//     Serial.print(accel[1]); Serial.print(" \t "); Serial.print(accel[2]); Serial.print("\t|\t");
//     Serial.print("GYRO:\t"); Serial.print(gyro[0]); Serial.print(" \t ");
//     Serial.print(gyro[1]); Serial.print(" \t "); Serial.print(gyro[2]); Serial.print("\t|\t");
//     Serial.print("VEL:\t"); Serial.print(vel[0]); Serial.print(" \t ");
//     Serial.print(vel[1]); Serial.print(" \t "); Serial.print(vel[2]); Serial.println("\t|\t");
// }