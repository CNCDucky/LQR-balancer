#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

#include "functions.h"
#include "EigenKalman.h"
#include "EigenLQR.h"
#include "printLinalg.h"

#define SCL_PIN 2
#define SDA_PIN 3

#define INP_VOLTAGE_SENSE 10
#define CURRENT_SEN_M1 5
#define CURRENT_SEN_M2 4
#define CURRENT_SEN_M1_OFFSET 1917.3f
#define CURRENT_SEN_M2_OFFSET 1925.4f

// Motor PWM
#define PWM1_M1 13
#define PWM2_M1 14
#define PWM1_M2 15
#define PWM2_M2 16

// Servo PWM
#define SERVO1 9
#define SERVO2 8
#define SERVO3 7
#define SERVO4 6

LPfilter voltageFilter(0.99, 0);
LPfilter currentM1filter(0.95, 0);
LPfilter currentM2filter(0.95, 0);

int freq = 200;
int resolution = 10; // Bits
int channel1_M1 = 1; int channel2_M1 = 2;
int channel1_M2 = 3; int channel2_M2 = 4;

PwmMotor Motor1(PWM1_M1, PWM2_M1, channel1_M1, channel2_M1, freq, resolution, false); // Right
PwmMotor Motor2(PWM1_M2, PWM2_M2, channel1_M2, channel2_M2, freq, resolution, true);  // Left
//

Adafruit_MPU6050 mpu;
EigenKalmanFilter Kalman;
EigenLQR LQR;
AS5600 encoderR(Wire, 0x36);

int n = 4;
int m = 2;
int p = 4;


void setup(){
  /////////////////////////////////////////////////////////

  Serial.begin(115200);
  Wire.begin();

  pinMode(INP_VOLTAGE_SENSE, INPUT);
  pinMode(CURRENT_SEN_M1, INPUT);
  pinMode(CURRENT_SEN_M2, INPUT);

  pinMode(PWM1_M1, OUTPUT); pinMode(PWM2_M1, OUTPUT);
  pinMode(PWM1_M2, OUTPUT); pinMode(PWM2_M2, OUTPUT);

  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  Motor1.motorInit();
  Motor2.motorInit();

  /////////////////////////////////////////////////////////

  Kalman.A << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

  Kalman.B << 0, 0,
              0, 0,
              0, 0,
              0, 0;

  Kalman.C = MatrixXf::Identity(p, p);

  Kalman.Q << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

  Kalman.R << 0.01, 0, 0, 0,
              0, 0.01, 0, 0,
              0, 0, 0.01, 0,
              0, 0, 0, 0.01;

  // Weight Matrices
  LQR.Q << 100, 0, 0, 0,
           0, 0.1, 0, 0,
           0, 0, 100, 0,
           0, 0, 0, 10;

  LQR.R = 0.001 * MatrixXf::Identity(m, m);
  LQR.x_ref << 0, 0, 0, 0; 

  LQR.init(Kalman.A_d, Kalman.B_d); // TOOD: Make one step Gain Calulation

}

void loop() {
  float vSense = analogRead(INP_VOLTAGE_SENSE);
  float voltage = voltageFilter.update(1.0875*20.13*vSense/4096);

  float sensitivity = 0.090; // V/A
  float iSense_M1 = analogRead(CURRENT_SEN_M1);
  float current_M1 = currentM1filter.update(-3.3*(iSense_M1 - CURRENT_SEN_M1_OFFSET)/(4096*sensitivity));

  float iSense_M2 = analogRead(CURRENT_SEN_M2);
  float current_M2 = currentM2filter.update(3.3*(iSense_M2 - CURRENT_SEN_M2_OFFSET)/(4096*sensitivity));

  Serial.print(voltage, 1); Serial.print(" "); Serial.print(current_M1); Serial.print(" "); Serial.println(current_M2);

  Motor1.motorWrite(1, true);
  Motor2.motorWrite(1, true);
}