#include <Arduino.h>
#include <Wire.h>

#define SCL_PIN 2
#define SDA_PIN 3

#define INP_VOLTAGE_SENSE 10
#define CURRENT_SEN_M1 5
#define CURRENT_SEN_M2 4

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

void setup() {

  Serial.begin(115200);
  Wire.begin();

  pinMode(INP_VOLTAGE_SENSE, INPUT);
  pinMode(CURRENT_SEN_M1, INPUT);
  pinMode(CURRENT_SEN_M2, INPUT);

  pinMode(PWM1_M1, OUTPUT);
  pinMode(PWM2_M1, OUTPUT);
  pinMode(PWM1_M2, OUTPUT);
  pinMode(PWM2_M2, OUTPUT);

  pinMode(SERVO1, OUTPUT);
  pinMode(SERVO2, OUTPUT);
  pinMode(SERVO3, OUTPUT);
  pinMode(SERVO4, OUTPUT);

  Serial.println("Started");
  delay(2000);
}

void loop() {
  float current_m1 = analogRead(CURRENT_SEN_M1);
  Serial.println(current_m1);
}