#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "driver/ledc.h"

#include "EigenKalman.h"
#include "EigenLQR.h"
#include "printLinalg.h"

// Declare function prototypes here:
int readAngle(TwoWire &i2c, uint8_t addr, long &cumulativeAngle, int &lastAngle, long &initialAngle);
void MotorControl(float voltage);
void CalibrateIMU();
VectorXf ReadIMU();
void CompIMU();
VectorXf CalcX();

Adafruit_MPU6050 mpu;
EigenKalmanFilter Balancer;
EigenLQR LQR;

#define SDA_1 21
#define SCL_1 22
#define SDA_2 14
#define SCL_2 27
#define AS5600_ADDR 0x36
#define ANGLE_REG_LSB 0x0C  // Low byte of angle
#define ANGLE_REG_MSB 0x0D  // High byte of angle

TwoWire I2C_2 = TwoWire(1);  // Second I2C channel (custom pins)

int n = 4;          // Number of states
int m = 2;          // Number of inputs
int p = 4;          // Number of outputs
float Ts = 0.01;    // Sampling time

// System parameters
float Kt = 0.01, Rr = 3.5, Rl = 4.5, M = 0.34, L = 0.04, r = 0.055/2, g = 9.82;

// Encoder readings
long cumulativeAngleR = 0;
long initialAngleR = 0;
long cumulativeAngleL = 0;
long initialAngleL = 0;
int lastAngleR = 0;
int lastAngleL = 0;

// Motor control
#define AIN1 18     // Direction pin 1
#define AIN2 23     // Direction pin 2
#define BIN1 33     // Direction pin 1
#define BIN2 32     // Direction pin 2
#define STBY 19     // Standby pin, on when high

// PWM settings
#define PWM_FREQ 20000
#define PWM_RESOLUTION 10
#define PWM_CHANNEL_A1 0
#define PWM_CHANNEL_A2 1
#define PWM_CHANNEL_B1 2
#define PWM_CHANNEL_B2 3

void setup() {
    Serial.begin(115200);
    Serial.println("Balancing Robot Initialization...");

    Wire.begin();
    I2C_2.begin(SDA_2, SCL_2);

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }

    initialAngleR = readAngle(Wire, AS5600_ADDR, cumulativeAngleR, lastAngleR, initialAngleR);
    initialAngleL = readAngle(I2C_2, AS5600_ADDR, cumulativeAngleL, lastAngleL, initialAngleL);

    Balancer.A << 0, 1, 0, 0,
                  g / L, 0, 0, 0,
                  0, 0, 0, 1,
                 -g, 0, 0, 0;

    Balancer.B << 0, 0,
                 -Kt / (Rr*M*L*L), -Kt / (Rl*M*L*L),
                 0, 0,
                 Kt / (M*Rr*r),  Kt / (M*Rl*r);

    Balancer.C = MatrixXf::Identity(p, p);

    // Weight Matrices
    LQR.Q << 100, 0, 0, 0,
             0, 0.01, 0, 0,
             0, 0, 100, 0,
             0, 0, 0, 1;

    LQR.R = 0.01 * MatrixXf::Identity(m, m);
    LQR.x_ref << 0, 0, 0, 0; 

    Balancer.discretize_state_matricies(); // creates A_d and B_d
    LQR.init(Balancer.A_d, Balancer.B_d);

    Balancer.Q << 0.02, 0, 0, 0,
                  0, 0.02, 0, 0,
                  0, 0, 10, 0,
                  0, 0, 0, 10;

    Balancer.R << 0.01, 0, 0, 0,
                  0, 0.01, 0, 0,
                  0, 0, 0.01, 0,
                  0, 0, 0, 0.01;

    // Motor control
    // Initialize motor control pins as outputs
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    // Initialize standby pin (to enable the motor driver)
    digitalWrite(STBY, HIGH);

    // Configure LEDC PWM channels
    ledcSetup(PWM_CHANNEL_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B2, PWM_FREQ, PWM_RESOLUTION);
    
    // Attach the channels to the GPIOs
    ledcAttachPin(AIN1, PWM_CHANNEL_A1);
    ledcAttachPin(AIN2, PWM_CHANNEL_A2);
    ledcAttachPin(BIN1, PWM_CHANNEL_B1);
    ledcAttachPin(BIN2, PWM_CHANNEL_B2);

    // Print Matrices
    printMatrix("A_d", Balancer.A_d);
    printMatrix("B_d", Balancer.B_d);
    printMatrix("C", Balancer.C);
    printMatrix("Q", Balancer.Q);
    printMatrix("R", Balancer.R);
    printMatrix("Q LQR", LQR.Q);
    printMatrix("R LQR", LQR.R);
    printMatrix("Riccati solution (P)", LQR.P);
    printMatrix("LQR Gain", LQR.L);

    Serial.println("System Initialized.");
    Serial.println();

    // CalibrateIMU(); // blocks the rest of the program
}

void loop() {

    unsigned static long lastTime;
    unsigned long currentTime = millis();
    static float pos = 0;

    if (currentTime - lastTime >= Ts * 1000) {  // Total loop time ~6ms icl serial print
        lastTime = currentTime;

        static VectorXf y = VectorXf::Zero(Balancer.n);

        // Read sensors
        VectorXf IMU = ReadIMU();    // angle and angular velocity (rad, rad/s)
        VectorXf x = CalcX();        // position and velocity (m, m/s)

        y << IMU(0), IMU(1), x(0), x(1);

        // Estimate current states
        Balancer.kalman_filter(y);

        LQR.x_ref << 0, 0, 0, 0; 

        Serial.print(Balancer.x(2)); Serial.print(" "); Serial.println(Balancer.x(3));

        // LQR
        VectorXf x_dev = Balancer.x - LQR.x_ref;
        VectorXf u = -LQR.L*x_dev;

        // Save current input for next iteration
        Balancer.u_prev = u;

        // If falling over:
        if (abs(IMU(2)) >= 1){ // rad
            u = VectorXf::Zero(m);;
        }

        MotorControl(u(0));
    }
}

VectorXf ReadIMU(){
    static VectorXf imu_vals = VectorXf::Zero(3);
    sensors_event_t a, G, temp;
    mpu.getEvent(&a, &G, &temp);

    static double gyro_ang = 0;

    float x_offset = 0.350, z_offset = 0.387, gyro_offset = -0.035;

    float gravity_x = -a.acceleration.x + x_offset;
    float gravity_z = -a.acceleration.z - z_offset;
    float trig_ang = atan2(gravity_x, gravity_z);

    // Complementary filter
    float gamma = 0.99;
    float ang_vel = -(G.gyro.y - gyro_offset);
    gyro_ang += Ts*ang_vel;
    gyro_ang = gamma*gyro_ang + (1 - gamma)*trig_ang;

    imu_vals << gyro_ang, ang_vel, trig_ang;
    return imu_vals;
}

int readAngle(TwoWire &i2c, uint8_t addr, long &cumulativeAngle, int &lastAngle, long &initialAngle) {
    uint8_t lowByte, highByte;

    // Request 2 bytes of data from the angle register
    i2c.beginTransmission(addr);
    i2c.write(ANGLE_REG_LSB);
    i2c.endTransmission(false);
    i2c.requestFrom(addr, (uint8_t)2);
    
    lowByte = i2c.read();
    highByte = i2c.read();

    // Little-endian
    int currentAngle = (lowByte << 8) | highByte;

    int delta = currentAngle - lastAngle;
    if (delta > 2047) {
        delta -= 4096;
    } else if (delta < -2047) {
        delta += 4096;
    }

    cumulativeAngle += delta;
    lastAngle = currentAngle;

    return cumulativeAngle - initialAngle;
}

void CalibrateIMU(){
    Serial.println("Calibrating IMU, assumed standing up");

    int n_samples = 100;
    float x_acc[n_samples] = {0}, z_acc[n_samples] = {0};
    float gyro[n_samples] = {0};
    float mean_x_acc = 0, mean_z_acc = 0, mean_gyro = 0;
    float variance_z_acc = 0, variance_x_acc = 0, variance_gyro = 0;

    sensors_event_t a, G, temp;
    float x_offset = -0.350, z_offset = 0.387, gyro_offset = -0.035;
    for (int sample = 0; sample < n_samples; sample++) {
        delay(Ts*1000);
        mpu.getEvent(&a, &G, &temp);
        x_acc[sample] = a.acceleration.x + x_offset;
        z_acc[sample] = a.acceleration.z + z_offset;
        gyro[n_samples] = G.gyro.y + gyro_offset;
    }

    // Mean: sum(x) / n
    for (int sample = 0; sample < n_samples; sample++) {
        mean_x_acc += x_acc[sample];
        mean_z_acc += z_acc[sample];
        mean_gyro += gyro[sample];
    }
    mean_x_acc /= n_samples;
    mean_z_acc /= n_samples;
    mean_gyro /= n_samples;

    // Standard deviation: sqrt(sum((x - mean)^2) / (n-1)), variance = std_dev^2
    for (int sample = 0; sample < n_samples; sample++) {
        variance_z_acc += pow(x_acc[sample] - mean_x_acc, 2);
        variance_x_acc += pow(z_acc[sample] - mean_z_acc, 2);
        variance_gyro += pow(gyro[sample] - mean_gyro, 2);
    }  
    variance_z_acc /= (n_samples - 1);
    variance_x_acc /= (n_samples - 1);
    variance_gyro /= (n_samples - 1);

    Serial.print("Mean x acc: "); Serial.println(mean_x_acc, 8);
    Serial.print("Variance: "); Serial.println(variance_x_acc, 8);
    Serial.print("offset:"); Serial.println(-mean_x_acc, 8);
    Serial.println("");
    Serial.print("Mean z acc: "); Serial.println(mean_z_acc, 8);
    Serial.print("Variance: "); Serial.println(variance_z_acc, 8);
    Serial.print("offset: "); Serial.println(-(g + mean_z_acc), 8);

    /*
    Mean x acc: -9.42101574
    Variance: 0.00239385
    Mean z acc: -0.23300368
    Variance: 0.00101341
    */

    // Laying flat on the table
    // Mean x: -9.5008
    // Mean z: 0.1112
    // Mean gyro: 0.0351
    // Variance gyro: 0.0199
    // Mean acc_angle: 1.5804
    // Variance acc_angle: 0.0199

    // Standing up
    // Mean x: 0.1050
    // Mean z: -10.1154
    // Mean gyro: 0.0341
    // Variance gyro: 0.0200
    // Mean acc_angle: -0.0084
    // Variance acc_angle: 0.0199
    while (true){}
}

VectorXf CalcX() {

    static float lastRadAngleR = 0;
    static float lastRadAngleL = 0;

    // Read angle for both encoders
    int angleR = readAngle(Wire, AS5600_ADDR, cumulativeAngleR, lastAngleR, initialAngleR);  // Right encoder
    int angleL = readAngle(I2C_2, AS5600_ADDR, cumulativeAngleL, lastAngleL, initialAngleL);  // Left encoder

    float radAngleR = angleR * 2*PI/4096;
    float radAngleL = angleL * 2*PI/4096;

    float angularVelR = (radAngleR - lastRadAngleR) / Ts;
    float angularVelL = (radAngleL - lastRadAngleL) / Ts;
    lastRadAngleR = radAngleR;
    lastRadAngleL = radAngleL;

    float velR = angularVelR * r;
    float velL = - angularVelL * r; // Negative sign due to encoder orientation

    float x_pos = (radAngleR - radAngleL)*r/2; // Negative sign due to encoder orientation

    float x_vel = (velR + velL) / 2;

    VectorXf x = VectorXf::Zero(2);
    x << x_pos, x_vel;
    
    return x;
}

void MotorControl(float voltage) {
    int pwmValue = abs(voltage) * 1024 / 7.6; // 8.4V is the maximum voltage
    pwmValue = constrain(pwmValue, 0, 1024);

    // Serial.print("PWM: "); Serial.println(pwmValue);

    if (voltage > 0) {
        ledcWrite(PWM_CHANNEL_A1, pwmValue);  // Forward
        ledcWrite(PWM_CHANNEL_A2, 0);
    } else if (voltage < 0) {
        ledcWrite(PWM_CHANNEL_A1, 0);
        ledcWrite(PWM_CHANNEL_A2, pwmValue);  // Reverse
    } else {
        ledcWrite(PWM_CHANNEL_A1, 0);
        ledcWrite(PWM_CHANNEL_A2, 0);  // Stop
    }

    if (voltage > 0) {
        ledcWrite(PWM_CHANNEL_B1, pwmValue);  // Forward
        ledcWrite(PWM_CHANNEL_B2, 0);
    } else if (voltage < 0) {
        ledcWrite(PWM_CHANNEL_B1, 0);
        ledcWrite(PWM_CHANNEL_B2, pwmValue);  // Reverse
    } else {
        ledcWrite(PWM_CHANNEL_B1, 0);
        ledcWrite(PWM_CHANNEL_B2, 0);  // Stop
    }
}