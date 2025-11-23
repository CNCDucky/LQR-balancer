#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

#include "functions.h"
#include "StateSpaceModel.h"
#include "printLinalg.h"

#define SCL_PIN 2
#define SDA_PIN 3

#define SCL2_PIN 37
#define SDA2_PIN 38

#define INP_VOLTAGE_SENSE 10
#define CURRENT_SEN_M1 5
#define CURRENT_SEN_M2 4
#define CURRENT_SEN_M1_OFFSET 1977.6f
#define CURRENT_SEN_M2_OFFSET 2218.6f

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
LPfilter currentM1filter(0.97, 0);
LPfilter currentM2filter(0.97, 0);

int freq = 200;
int resolution = 10; // Bits
int channel1_M1 = 1; int channel2_M1 = 2;
int channel1_M2 = 3; int channel2_M2 = 4;

PwmMotor Motor1(PWM1_M1, PWM2_M1, channel1_M1, channel2_M1, freq, resolution, false); // Left
PwmMotor Motor2(PWM1_M2, PWM2_M2, channel1_M2, channel2_M2, freq, resolution, true);  // Right

void setup(){
  /////////////////////////////////////////////////////////
  delay(2000);
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire2.begin(SDA2_PIN, SCL2_PIN);

  Wire.setClock(400000);
  Wire2.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  pinMode(INP_VOLTAGE_SENSE, INPUT);
  pinMode(CURRENT_SEN_M1, INPUT);
  pinMode(CURRENT_SEN_M2, INPUT);

  pinMode(PWM1_M1, OUTPUT); pinMode(PWM2_M1, OUTPUT);
  pinMode(PWM1_M2, OUTPUT); pinMode(PWM2_M2, OUTPUT);

  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  // TODO: MEASURE MOTOR RESISTANCE!!!
  Motor1.calculateParams(8, 8000*PI/30, 0.7, 3, 4);
  Motor2.calculateParams(8, 8000*PI/30, 0.7, 3, 4);

  Motor1.setRegulatorParams(15, 0, 0, 0.03, 0, 0);
  Motor2.setRegulatorParams(15, 0, 0, 0.03, 0, 0);

  Motor1.motorInit();
  Motor2.motorInit();

  /////////////////////////////////////////////////////////

  Model.Ad << 1, -0.000122881, 0.0100099, -4.08979e-07,
              0, 1.02053, -0.00198484, 0.0100683,
              0, -0.024668, 1.00199, -0.000122881,
              0, 4.12202, -0.398451, 1.02053;

  Model.Bd << 0.0183257, 0.0183257,
              -1.72868, -1.72868,
              3.67338, 3.67338,
              -347.156, -347.156;

  Model.Q << 0.05, 0, 0, 0,
             0, 0.05, 0, 0,
             0, 0, 0.05, 0,
             0, 0, 0, 0.05;
  
  Model.R << 0.05, 0, 0, 0,
             0, 0.05, 0, 0,
             0, 0, 0.05, 0,
             0, 0, 0, 0.05;

  // LQR
  Model.x_ref << 0, 0, 0, 0;
  Model.K_lqr << -0.00111299, -0.045214, -0.00313631, -0.00399954,
                 -0.001113, -0.045214, -0.00313631, -0.00399954;

}

void loop() {
  float vSense = analogRead(INP_VOLTAGE_SENSE);
  float voltage = voltageFilter.update(1.0875*20.13*vSense/4096);

  float sensitivity = 0.090; // V/A
  float iSense_M1 = analogRead(CURRENT_SEN_M1);
  float current_M1 = currentM1filter.update(-3.3*(iSense_M1 - CURRENT_SEN_M1_OFFSET)/(4096*sensitivity));

  float iSense_M2 = analogRead(CURRENT_SEN_M2);
  float current_M2 = currentM2filter.update(3.3*(iSense_M2 - CURRENT_SEN_M2_OFFSET)/(4096*sensitivity));

  // Serial.print(voltage, 1); Serial.print(" "); Serial.print(current_M1); Serial.print(" "); Serial.println(current_M2);

  ////////////////////////////////////////////////////////

  unsigned static long lastTime = millis();
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= Model.Ts*1000) {

      float dt = (currentTime - lastTime)/1000.0f;
      lastTime = currentTime;

      static VectorXf y_meas = VectorXf::Zero(Model.n);

      // Read sensors
      VectorXf imu = readMPU(dt);       // angle and angular velocity (rad, rad/s)
      VectorXf enc = readEncoders(dt);  // position and velocity (m, m/s)
      y_meas << enc(0), imu(0), enc(1), imu(1);

      // Estimate current states
      VectorXf x_est = Model.kalmanFilter(y_meas);
      // Serial.print(y_meas(0)); Serial.print(" "); Serial.print(y_meas(1));
      // Serial.print(" "); Serial.print(y_meas(2)); Serial.print(" "); Serial.println(y_meas(3));

      // LQR
      VectorXf x_dev = x_est - Model.x_ref;
      VectorXf tau_ref = - Model.K_lqr*y_meas;

      // Save current input for next iteration
      Model.u_prev = tau_ref;

      if (abs(imu(0)) >= 1) {
        Motor1.motorDisable();
        Motor2.motorDisable();
      }
      else {
        Motor1.motorWriteTorque(tau_ref(0), voltage, current_M1, encoderL.angVel, dt);
        Motor2.motorWriteTorque(tau_ref(1), voltage, current_M2, encoderR.angVel, dt);
      }
    }
}