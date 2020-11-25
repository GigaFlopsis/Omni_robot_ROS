//
// Created by op on 16.11.2020.
//

#ifndef ARDUINO_OMNI_WHEEL_ENCODER_H
#define ARDUINO_OMNI_WHEEL_ENCODER_H
#include "Arduino.h"
#include <Encoder.h>

#define RAD_CICLE 0.00204530772


class MyEncoder {
public:
    MyEncoder(int _outputA, int _outputB, float _radius, int _rate = 10.);
    void Init();
    void Update();
    float GetVel();
private:
    Encoder Enc;
    float radius = 0.075;
    int outputA= 15;
    int outputB=14;
    float rate = 10.;
    float velocity = 0;
    float period = 0;
    long counter_period = 0;
    
    unsigned long my_timer;
    float dt;
};


#endif //ARDUINO_OMNI_WHEEL_ENCODER_H
