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

#endif // FUNCTIONS_H