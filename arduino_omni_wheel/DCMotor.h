//
// Created by op on 17.11.2020.
//

#ifndef ARDUINO_OMNI_WHEEL_DCMOTOR_H
#define ARDUINO_OMNI_WHEEL_DCMOTOR_H


#include "Arduino.h"
#include <L298NX2.h>
#include "Encoder.h"


class DCMotor {
    public:
        DCMotor(int _EN_A, int _IN1_A, int _IN2_A, int _ENC_A, int _ENC_B, float _wheel_radius, float _rate);
        void Init();

        void Update();
        float GetVel();
        float GetOutput();
        float GetTargetVel();


        void SetVal(float targetVal);

        void SetKp(float val);
        void SetKi(float val);
        void SetKd(float val);
        
        float kP = 0.5;
        float kI = 0.4;
        float kD = 0.00;


    private:

        // Pin definition
        int EN_A = 4;
        int IN1_A = 5;
        int IN2_A = 6;

        int ENC_A = 2;
        int ENC_B = 3;

        // PID params

       
        float iLim = 0.8;

        float target_vel = 0.0;

        // robot params

        float rate = 10;
        float wheel_radius = 0.075;


        float error = 0;
        float last_error = 0;
        float rateError = 0;
        float cumError = 0;
        float dt;

        float output;

       // Initialize motor and encoder
        L298N A_motor;
        Encoder A_encoder;

        float vel;
        float period;

        unsigned long my_timer;


        float map(float x, float in_min, float in_max, float out_min, float out_max)
        {
        x = constrain(x,in_min,in_max);
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

};


#endif //ARDUINO_OMNI_WHEEL_DCMOTOR_H
