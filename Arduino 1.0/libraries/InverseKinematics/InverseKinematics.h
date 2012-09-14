#ifndef InverseKinematics_h
#define InverseKinematics_h

#include "Arduino.h"

class InverseKinematics
{
  public:
    Morse(int pin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif
