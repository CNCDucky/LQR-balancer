#ifndef FUNCTIONS_H
#define FUNCTIONS_H

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

    void motorWrite(float dutycycle, bool forward){

      int duty = round(dutycycle*maxDuty);
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

class AS5600 {
    private:
        uint8_t address = 0x36;
        uint8_t ANGLE_REG_LSB = 0x0C;
        uint8_t ANGLE_REG_MSB = 0x0D;
        
        unsigned long lastTime = 0;
        long initialPos = 0;
        float lastAngle = 0;
        bool init = false;

        float angle = 0;
        float angVel = 0;
        long currentPos = 0;
        long lastPos = 0;
        int delta = 0;

        TwoWire* i2c;  // Pointer to the I2C interface

    public:

        // Constructor to assign the I2C bus
        AS5600(TwoWire& wirePort, uint8_t i2c_address){
            i2c = &wirePort;
            address = i2c_address;
        }

        void readAngle() {
            unsigned long currentTime = micros();

            float dt = (currentTime - lastTime) / pow(10, 6);

            uint8_t lowByte, highByte;
            i2c->beginTransmission(address);
            i2c->write(ANGLE_REG_LSB);
            i2c->endTransmission(false);
            i2c->requestFrom(address, (uint8_t) 2);

            if (i2c->available() >= 2) {
                lowByte = i2c->read();
                highByte = i2c->read();

                currentPos = (lowByte << 8) | highByte;

                if (!init) {
                    initialPos = currentPos;
                    init = true;
                }

                currentPos -= initialPos;
                if (currentPos < 0) currentPos += 4096;
                else if (currentPos >= 4096) currentPos -= 4096;

                delta = currentPos - lastPos;
                if (delta > 2048) delta -= 4096;
                else if (delta < -2048) delta += 4096;
                lastPos = currentPos;

                angle += (float) delta * 2 * PI / 4096;
                angVel = (angle - lastAngle) / dt;
                lastAngle = angle;

                lastTime = currentTime;
            }
        }
};

#endif // FUNCTIONS_H