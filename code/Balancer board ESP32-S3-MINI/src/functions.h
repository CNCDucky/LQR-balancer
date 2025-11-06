#ifndef FUNCTIONS_H
#define FUNCTIONS_H

class LPfilter {
  private:
    float weight;
    float filteredValue;

  public:
    // Constructor
    LPfilter(float filterWeight, float initialValue) {
      weight = filterWeight;
      filteredValue = initialValue;
    }

    float update(float input) {
      filteredValue = (1-weight) * input + weight * filteredValue;
      return filteredValue;
    }
};

#endif // FUNCTIONS_H