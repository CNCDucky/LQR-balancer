#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <ArduinoEigenDense.h>
#include "driver/ledc.h"
using namespace Eigen;
using namespace std;

// Matrix2d A; // 2x2 double matrix
// Matrix3f A; // 3x3 float matrix
// Matrix3d B(3, 3); // Dynamic-sized matrix (3x3)
// Vector3d v(1, 2, 3); // 3D vector with doubles

// Matrix2d C = A + A; // Addition
// Matrix2d D = A * A; // Multiplication
// Matrix2d E = A.inverse(); // Inverse
// double det = A.determinant(); // Determinant
// Vector2d x = A.colPivHouseholderQr().solve(Vector2d(1, 2)); // Solving linear systems

// MatrixXd I = MatrixXd::Identity(3, 3); // Identity matrix
// MatrixXd Z = MatrixXd::Zero(3, 3); // Zero matrix
// MatrixXd O = MatrixXd::Ones(3, 3); // Ones matrix

// Accessing elements:
// double val = A(0, 1); // Access element at row 0, col 1
// A(1, 0) = 5.0; // Modify an element

// Declare function prototypes here:
template <typename Derived>
void printMatrix(const char* name, const Eigen::MatrixBase<Derived>& m);

template <typename Derived>
void printVector(const char* name, const Eigen::MatrixBase<Derived>& v);

int readAngle(TwoWire &i2c, uint8_t addr, long &cumulativeAngle, int &lastAngle, long &initialAngle);

void MotorControl(float voltage);

void CalibrateIMU();

VectorXf ReadIMU();

void CompIMU();

VectorXf CalcX();

void ScanI2C();

Adafruit_MPU6050 mpu;

#define SDA_1 21
#define SCL_1 22
#define SDA_2 14
#define SCL_2 27
#define AS5600_ADDR 0x36
#define ANGLE_REG_LSB 0x0C  // Low byte of angle
#define ANGLE_REG_MSB 0x0D  // High byte of angle

TwoWire I2C_2 = TwoWire(1);  // Second I2C channel (custom pins)
VectorXf imu_vals = VectorXf::Zero(2);

int n = 4;          // Number of states
int m = 2;          // Number of inputs
int p = 4;          // Number of outputs
float Ts = 0.01;    // Sampling time

// System parameters
float Kt = 0.23, mu = 0.25, R = 3.5, M = 0.34, L = 0.04, r = 0.055/2, g = 9.82;

// Using LQR with integral action requires an augmented model
// This does not work atm. since the the augmeted model is not stable
// To fix this one could choose to add poles to the augmented model.
// Adding integral action seperately works fine.

#define INTEGRAL_ACTION true    // Integral action for LQR
#define FEED_FORWARD false      // For faster reference response

// Encoder readings
long cumulativeAngleR = 0;
long initialAngleR = 0;
long cumulativeAngleL = 0;
long initialAngleL = 0;
int lastAngleR = 0;
int lastAngleL = 0;

class EigenKalmanFilter {
    public:
        // System dimensions (default)
        int n = 4;          // Number of states
        int m = 2;          // Number of inputs
        int p = 4;          // Number of outputs
        float Ts = 0.01;    // Sampling time

        // Kalman state matricies
        MatrixXf A = MatrixXf::Zero(n, n);            // Transition matrix
        MatrixXf B = MatrixXf::Zero(n, m);            // Input matrix
        MatrixXf C = MatrixXf::Zero(p, n);            // Measurement model
        MatrixXf A_d = MatrixXf::Zero(n, n);          // Discretized transition matrix
        MatrixXf B_d = MatrixXf::Zero(n, m);          // Discretized input matrix
        MatrixXf K = MatrixXf::Zero(n, p);            // Kalman gain
        MatrixXf I = MatrixXf::Identity(n, n);

        // Covariances
        MatrixXf Q = MatrixXf::Zero(n, n);            // Process noise covariance
        MatrixXf R = MatrixXf::Zero(p, p);            // Measurement noise covariance
        MatrixXf P = MatrixXf::Zero(n, n);            // Estimation error covariance
        MatrixXf P_pred = MatrixXf::Zero(n, n);       // Predicted estimation error covariance    

        VectorXf x_prior = VectorXf::Zero(n);         // State prior/last estimated state x_k-1/k-1 
        VectorXf x_pred = VectorXf::Zero(n);          // Predicted state x_k/k-1
        VectorXf x = VectorXf::Zero(n);               // State estimate/ posterior x_k/k
        VectorXf u_prev = VectorXf::Zero(m);          // Previous input
        VectorXf v = VectorXf::Zero(p);               // Innovation
        MatrixXf S = MatrixXf::Zero(p, p);            // Innovation covariance


        void discretize_state_matricies(){
            // Discretization (Taylor series)
            MatrixXf Psi = MatrixXf::Zero(n,n);
            
            Psi = I * Ts + (A*(pow(Ts,2) / 2)) + (A*A*(pow(Ts,3) / 6)) + (A*A*A*(pow(Ts,4) / 24)) + (A*A*A*A*(pow(Ts,5) / 120));
            A_d = I + A * Psi;
            B_d = Psi * B;
        }

        void kalman_filter(VectorXf y){
            // Called each sampling interval Ts
            // Predicts the next state from prior states and previous input u
            x_pred = A_d*x_prior + B_d*u_prev;
            P_pred = A_d*P*A_d.transpose() + Q;

            // Update state with measurement y
            v = y - C*x_pred;
            S = C*P_pred*C.transpose() + R;
            K = P_pred*C.transpose()*S.inverse();
            P = P_pred - K*S*K.transpose();         // Reduced P_pred with measurement

            x = x_pred + K*v;                       // Corrected state estimate

            // for next iteration
            x_prior = x;
        }


};

class LinearQuadratic {
    public:
        // System dimensions (default)
        int n = 4;          // Number of states A_d.rows(),
        int m = 2;          // Number of inputs B_d.cols(),
        float Ts = 0.01;    // Sampling time

        // LQR Matrices
        MatrixXf Q = MatrixXf::Zero(n, n);            // State weight matrix
        MatrixXf R = MatrixXf::Zero(m, m);            // Input weight matrix
        MatrixXf P = MatrixXf::Identity(n,n);                             // Riccati solution with inital guess
        MatrixXf L = MatrixXf::Zero(m, n);            // LQR gain
        VectorXf x_ref = VectorXf::Zero(n);           // Reference state

        void init(const MatrixXf& A_d, const MatrixXf& B_d) {
            // A_d and B_d are discretized state space matricies
            // In some cases the Riccati solution might not converge,
            // The LQR gain L will be zero, preventing state based input.

            // Solve Discrete-time Algebraic Riccati Equation (DARE)
            MatrixXf P_prev = MatrixXf::Zero(n, n);
            float tolerance = 1;
            int max_iterations = 1000;
            int i = 0;

            while ((P - P_prev).norm() > tolerance && i <= max_iterations) {
                P_prev = P;

                MatrixXf K = P*B_d*(R + B_d.transpose()*P*B_d).inverse();
                P = Q + A_d.transpose()*(P - K*B_d.transpose()*P)*A_d;

                if (i == max_iterations){
                    Serial.println("The Riccati matrix P has not converged!");
                }
                i++;
            };

            // Compute LQR Gain: L = (R + B^T P B)^-1 B^T P A
            L = (R + B_d.transpose()*P*B_d).inverse()*B_d.transpose()*P*A_d;
            printMatrix("right", B_d.transpose()*P*A_d);
            printMatrix("left", (R + B_d.transpose()*P*B_d).inverse());
        }
};


// Motor control
#define AIN1 18     // Direction pin 1
#define AIN2 23     // Direction pin 2
#define BIN1 33     // Direction pin 1
#define BIN2 32     // Direction pin 2
#define STBY 19     // Standby pin, on when high

// PWM settings
#define PWM_FREQ 1000   // 5 kHz PWM frequency
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_A1 0
#define PWM_CHANNEL_A2 1
#define PWM_CHANNEL_B1 2
#define PWM_CHANNEL_B2 3

EigenKalmanFilter Balancer;
LinearQuadratic LQR;

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
                 -g, 0, 0, -mu / (M*R*r);

    Balancer.B << 0, 0,
                 -Kt / (R*M*L*L), -Kt / (R*M*L*L),
                 0, 0,
                 Kt / (M*R*r),  Kt / (M*R*r);

    Balancer.C = MatrixXf::Identity(p, p);

    // Weight Matrices
    LQR.Q << 10000, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0.001, 0,
             0, 0, 0, 0; // How much do we penalize error in each state?

    LQR.R = 0.0001 * MatrixXf::Identity(m, m);
    LQR.x_ref << 0, 0, 0, 0;

    Balancer.discretize_state_matricies(); // creates A_d and B_d
    LQR.init(Balancer.A_d, Balancer.B_d);

    Balancer.Q << 10, 0, 0, 0,
                  0, 10, 0, 0,
                  0, 0, 10, 0,
                  0, 0, 0, 10; // Process noise covariance

    Balancer.R << 0.01, 0, 0, 0,
                  0, 0.01, 0, 0,
                  0, 0, 0.001, 0,
                  0, 0, 0, 0.001;

    // Motor control
    // Initialize motor control pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
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
    printMatrix("LQR Gain (Lq)", LQR.L);

    Serial.println("System Initialized.");
    Serial.println();

    // CalibrateIMU(); // blocks the rest of the program
}

void loop() {

    unsigned static long lastTime;
    unsigned long currentTime = millis();

    if (currentTime - lastTime >= Ts * 1000) {  // Total loop time ~6ms icl serial print
        lastTime = currentTime;

        static VectorXf y = VectorXf::Zero(Balancer.n);

        // Read sensors
        VectorXf IMU = ReadIMU();    // angle and angular velocity (rad, rad/s)
        VectorXf x = CalcX();        // position and velocity (m, m/s)

        y << IMU(0), IMU(1), x(0), x(1);

        // Estimate current states
        Balancer.kalman_filter(y);

        // Serial.print("Angle: "); Serial.print(Balancer.x(0), 3); Serial.print(" rate: "); Serial.print(Balancer.x(1), 3);
        // Serial.print("x: "); Serial.print(Balancer.x(2), 3); Serial.print(" x_dot: "); Serial.println(Balancer.x(3), 3);
        // Serial.print("Angle: "); Serial.print(y(0), 3); Serial.print(" rate: "); Serial.print(y(1), 3);
        // Serial.print("x: "); Serial.print(y(2), 3); Serial.print(" x_dot: "); Serial.println(y(3), 3);

        // LQR
        VectorXf x_dev = Balancer.x - LQR.x_ref;
        VectorXf u = -LQR.L*x_dev;

        // Save current input for next iteration
        Balancer.u_prev = u;

        // If falling over:
        if (abs(IMU(0)) >= 1){ // rad
            u = VectorXf::Zero(m);;
        }

        MotorControl(u(0));
    }
}

VectorXf ReadIMU(){

    sensors_event_t a, G, temp;
    mpu.getEvent(&a, &G, &temp);

    static float gyro_ang = 0;

    float x_offset = 0.5, z_offset = 0.295, gyro_offset = 0.0351;

    float gravity_x = -a.acceleration.x + x_offset;
    float gravity_z = -a.acceleration.z - z_offset;

    float trig_ang = atan2(gravity_x, gravity_z);

    // Complementary filter
    float gamma = 0.85;
    float ang_vel = -(G.gyro.y - gyro_offset);
    gyro_ang += Ts*ang_vel;
    gyro_ang = gamma*gyro_ang + (1 - gamma)*trig_ang;

    imu_vals << gyro_ang, ang_vel; // z = [angle, angular velocity] from gyroscope data
    // Serial.print("Angle: "); Serial.println(gyro_ang);
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
    Serial.println("Calibrating IMU...");

    int n_samples = 100;
    float angle_samples[n_samples] = {0};
    float gyro_samples[n_samples] = {0};

    float mean_angle = 0;
    float mean_gyro = 0;

    float variance_angle = 0;
    float variance_gyro = 0;
    VectorXf result = VectorXf::Zero(2);

    for (int sample = 0; sample < n_samples; sample++) {
        result = ReadIMU();
        angle_samples[sample] = result(0);
        gyro_samples[sample] = result(1);
    }

    // Mean: sum(x) / n
    for (int sample = 0; sample < n_samples; sample++) {
        mean_angle += angle_samples[sample];
        mean_gyro += gyro_samples[sample];
    }
    mean_angle /= n_samples;
    mean_gyro /= n_samples;

    // Standard deviation: sqrt(sum((x - mean)^2) / (n-1)), variance = std_dev^2
    for (int sample = 0; sample < n_samples; sample++) {
        variance_angle += pow(angle_samples[sample] - mean_angle, 2);
        variance_gyro += pow(gyro_samples[sample] - mean_gyro, 2);
    }  
    variance_angle /= (n_samples - 1);
    variance_gyro /= (n_samples - 1);

    Serial.print("Mean gyro: "); Serial.println(mean_gyro, 8);
    Serial.print("Variance gyro: "); Serial.println(variance_gyro, 8);

    Serial.print("Mean angle: "); Serial.println(mean_angle, 8);
    Serial.print("Variance angle: "); Serial.println(variance_angle, 8);

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

    // After input into filter
    // Variance gyro: 0.00006533
    // Variance angle: 0.00001489

    while (true){}
}

VectorXf CalcX() {

    static float lastRadAngleR = 0;
    static float lastRadAngleL = 0;

    // Read angle for both encoders
    int angleR = readAngle(Wire, AS5600_ADDR, cumulativeAngleR, lastAngleR, initialAngleR);  // Right encoder
    int angleL = readAngle(I2C_2, AS5600_ADDR, cumulativeAngleL, lastAngleL, initialAngleL);  // Left encoder

    float radAngleR = angleR * 2*PI/4096;
    float angularVelR = (radAngleR - lastRadAngleR) / Ts;
    float velR = angularVelR * r;
    lastRadAngleR = radAngleR;

    float radAngleL = angleL * 2*PI/4096;
    float angularVelL = (radAngleL - lastRadAngleL) / Ts;
    float velL = -angularVelL * r; // Negative sign due to encoder orientation
    lastRadAngleL = radAngleL;

    float x_pos = (radAngleR - radAngleL)*r/2; // Negative sign due to encoder orientation

    float x_vel = (velR + velL) / 2;

    //Serial.print("x pos: "); Serial.println(x_pos);
    //Serial.print("x vel: "); Serial.println(x_vel);

    VectorXf x = VectorXf::Zero(2);
    x << x_pos, x_vel;
    
    return x;
}

void MotorControl(float voltage) {
    int pwmValue = abs(voltage) * 255 / 8.4; // 8.4V is the maximum voltage
    pwmValue = constrain(pwmValue, 0, 255);

    Serial.print("PWM: "); Serial.println(pwmValue);

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

// Print matrix
template <typename Derived>
void printMatrix(const char* name, const Eigen::MatrixBase<Derived>& m) {
    Serial.print(name);
    Serial.print(" (");
    Serial.print(m.rows());
    Serial.print("x");
    Serial.print(m.cols());
    Serial.println("):");

    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            Serial.print(m(i, j));
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();
}

// Print vector
template <typename Derived>
void printVector(const char* name, const Eigen::MatrixBase<Derived>& v) {
    Serial.print(name);
    Serial.print(" (");
    Serial.print(v.size());
    Serial.println("):");

    for (int i = 0; i < v.size(); ++i) {
        Serial.println(v(i));
    }
    Serial.println();
}