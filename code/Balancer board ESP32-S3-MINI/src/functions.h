#ifndef FUNCTIONS_H
#define FUNCTIONS_H
  #include <Adafruit_MPU6050.h>
  #include <ArduinoEigenDense.h>
  using namespace std;
  using namespace Eigen;
  Adafruit_MPU6050 mpu;

class LPfilter {
  private:
    float weight;
    float filteredValue;

  public:
    // Constructor
    LPfilter(float filterWeight, float initialValue){
      weight = filterWeight;
      filteredValue = initialValue;
    }

    float update(float input){
      filteredValue = (1-weight) * input + weight * filteredValue;
      return filteredValue;
    }
};

class PwmMotor {
  private:

    int IN1 = 0;
    int IN2 = 0;
    int freq = 200;
    int channel1 = 0;
    int channel2 = 0;
    int resolution = 10; // bits
    int maxDuty = (1 << resolution) - 1; // eg, 1 - 1023 for 10 bit resolution
    bool reverse = false;

    // motor parameters
    float ke = 0.01;
    float kt = 0.01;
    float R = 3;

    // Control params
    float Kp_T = 10;
    float Ki_T = 0;
    float Kd_T = 0;
    float Kp_w = 0.1;
    float Ki_w = 0;
    float Kd_w = 0;
    float intLimit = 1;

  public:
    // Constructor
    PwmMotor(int pin_IN1, int pin_IN2, int pwm_channel1, int pwm_channel2, int pwm_freq, int pwm_resolution, bool reverse_direction){
      IN1 = pin_IN1;
      IN2 = pin_IN2;
      freq = pwm_freq;
      resolution = pwm_resolution;
      channel1 = pwm_channel1;
      channel2 = pwm_channel2;
      reverse = reverse_direction;
    }

    void motorInit(){
      ledcSetup(channel1, freq, resolution);
      ledcSetup(channel2, freq, resolution);
      ledcAttachPin(IN1, channel1);
      ledcAttachPin(IN2, channel2);
    }

    void calculateParams(float U_rated, float w_noload, float tau_stall, float i_stall, float R_mot) {
      // example:
      // U_rated = 24; // V
      // w_noload = 12000*pi/30; // rad/s
      // tau_stall = 0.70; // Nm
      // i_stall = 3;
      // R = 4; // Ohm

      R = R_mot;
      // back emf and torque constant approx,
      // If tau_stall is unknown, assume k_e = k_t
      ke = U_rated/w_noload;
      kt = tau_stall/i_stall;
    }

    void setRegulatorParams(float Kp_T_reg, float Ki_T_reg, float Kd_T_reg, float Kp_w_reg, float Ki_w_reg, float Kd_w_reg) {
      Kp_T = Kp_T_reg;
      Ki_T = Ki_T_reg;
      Kd_T = Kd_T_reg;
      Kp_w = Kp_w_reg;
      Ki_w = Ki_w_reg;
      Kd_w = Kd_w_reg;
    }
    
    void motorWriteTorque(float tau_ref, float Vin, float i_mot, float phi, float dt) {

      static float int_i_err = 0;
      static float prev_i_err = 0;
      static float alpha = 0.85;
      float i_ref = tau_ref/kt;
      float V_drop = R*i_ref + ke*phi;
      float i_err = alpha*prev_i_err + (1-alpha)*(i_ref - i_mot);

      int_i_err += i_err*dt;
      if (int_i_err > intLimit) int_i_err = intLimit;
      if (int_i_err < -intLimit) int_i_err = -intLimit;

      float V_corr = Kp_T*i_err + Ki_T*int_i_err + Kd_T*(i_err - prev_i_err)/dt;
      prev_i_err = i_err;

      Serial.println(i_err);

      float V_cmd = V_drop + V_corr;

      // Convert to dutycycle
      float dutycycle = V_cmd/Vin;
      if (dutycycle > 1.0f) dutycycle = 1.0f;
      if (dutycycle < -1.0f) dutycycle = -1.0f;

      motorWrite(dutycycle);
    }

    void motorWriteSpeed(float phi_ref, float Vin, float i_mot, float phi, float dt) {

      static float int_w_err = 0;
      static float prev_w_err = 0;
      static float alpha = 0.95;

      float w_err = alpha*prev_w_err + (1-alpha)*(phi_ref - phi);
      int_w_err += w_err*dt;
      float tau_ref = Kp_w*w_err + Ki_w*int_w_err - Kd_w*(w_err - prev_w_err)/dt;
      prev_w_err = w_err;

      if (int_w_err > intLimit) int_w_err = intLimit;
      if (int_w_err < -intLimit) int_w_err = -intLimit;

      motorWriteTorque(tau_ref, Vin, i_mot, phi, dt);
    }

    void motorWrite(float dutycycle){
      bool forward = true;
      if (dutycycle < 0) forward = false;
      int duty = round(abs(dutycycle)*maxDuty);

      // not reversed
      if (forward == true){
        if (reverse == false){
          ledcWrite(channel1, duty);
          ledcWrite(channel2, 0);
        }
        else{
          ledcWrite(channel1, 0);
          ledcWrite(channel2, duty);
        }
      }
      else{
        if (reverse == false){
          ledcWrite(channel1, 0);
          ledcWrite(channel2, duty);
        }
        else{
          ledcWrite(channel1, duty);
          ledcWrite(channel2, 0);
        }
      }
    }

    void motorBrake(float dutycycle, bool forward){
      int duty = round(dutycycle*maxDuty);

      ledcWrite(channel1, duty);
      ledcWrite(channel2, duty);
    }

    void motorDisable(){
      ledcWrite(channel1, 0);
      ledcWrite(channel2, 0);
    }

};

class AS5600L {
  private:
    uint8_t address = 0x40;
    uint8_t ANGLE_REG_LSB = 0x0C;
    uint8_t ANGLE_REG_MSB = 0x0D;

    bool init = false;
    float lastAngle = 0;
    float lastAngVel = 0;
    uint16_t initialPos = 0;
    uint16_t currentPos = 0;
    uint16_t lastPos = 0;
    int delta = 0;

    TwoWire* i2c;

  public:
    float angle = 0;
    float angVel = 0;

    // Correct constructor
    AS5600L(TwoWire& wirePort, uint8_t i2c_address = 0x40) {
      i2c = &wirePort;
      address = i2c_address;
    }

    void readAngle(float dt) {
      // Read MSB first
      i2c->beginTransmission(address);
      i2c->write(ANGLE_REG_MSB);
      i2c->endTransmission(false);
      i2c->requestFrom(address, (uint8_t) 2);

      if (i2c->available() >= 2) {
        uint8_t highByte = i2c->read();
        uint8_t lowByte = i2c->read();

        currentPos = (lowByte << 8) | highByte;

        if (!init) {
          initialPos = currentPos;
          lastPos = currentPos;
          init = true;
        }

        delta = currentPos - lastPos;
        if (delta > 2048) delta -= 4096;
        else if (delta < -2048) delta += 4096;

        lastPos = currentPos;
        
        float alpha = 0;
        angle += delta*2*PI/4096;
        angVel = alpha*lastAngVel + (1-alpha)*(angle - lastAngle) / dt;
        lastAngle = angle;
        lastAngVel = angVel;
      }
    }
};


TwoWire Wire2 = TwoWire(1);
AS5600L encoderL(Wire, 0x40);
AS5600L encoderR(Wire2, 0x40);

VectorXf readEncoders(float dt) {

  float r = 0.025;

  encoderL.readAngle(dt);
  encoderR.readAngle(dt);
  float angleL = encoderL.angle, angleR = encoderR.angle;
  float angVelL = encoderL.angVel, angVelR = encoderR.angVel;

  float velL = angVelL*r;
  float velR = angVelR*r;

  float x_pos = (angleR + angleL)*r/2;
  float x_vel = (velR + velL)/2;

  VectorXf x = VectorXf::Zero(2);
  x << x_pos, x_vel;

  return x;
}

VectorXf readMPU(float dt){

  static bool init = false;
  if (init == false) {
    //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    init = true;
  }

  static VectorXf imu_vals = VectorXf::Zero(3);
  sensors_event_t a, G, temp;
  mpu.getEvent(&a, &G, &temp);

  static float gyro_ang = 0;
  float x_offset = -0.3040, z_offset = 0.1, gyro_offset = 0;

  float gravity_x = -a.acceleration.x - x_offset;
  float gravity_z = -a.acceleration.z - z_offset;
  float trig_ang = atan2(gravity_x, gravity_z);

  // Complementary filter
  float gamma = 0.95;
  float ang_vel = -(G.gyro.y - gyro_offset);
  gyro_ang = gamma*(gyro_ang + dt*ang_vel) + (1-gamma)*trig_ang;

  //Serial.print(gravity_x); 
  //Serial.print(" "); Serial.print(gravity_z);
  //Serial.print(" "); Serial.print(trig_ang);
  //Serial.print(" "); Serial.println(gyro_ang);

  imu_vals << -gyro_ang, -ang_vel, -trig_ang;
  return imu_vals;

}

#endif // FUNCTIONS_H