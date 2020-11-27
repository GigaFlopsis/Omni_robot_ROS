//
// Created by op on 17.11.2020.
//

#include "DCMotor.h"


DCMotor::DCMotor(int _EN_A, int _IN1_A, int _IN2_A, int _ENC_A, int _ENC_B, float _wheel_radius, float _rate) : 
EN_A(_EN_A), 
IN1_A(_IN1_A), 
IN2_A(_IN2_A), 
ENC_A(_ENC_A), 
ENC_B(_ENC_B),
rate(_rate), 
wheel_radius(_wheel_radius), 
A_motor(_EN_A, _IN1_A, _IN2_A), 
A_encoder(_ENC_A,_ENC_B,_wheel_radius,_rate)
{
    // // init params
    // EN_A = _EN_A;
    // IN1_A = _IN1_A;
    // IN2_A=_IN2_A;
    // ENC_A=_ENC_A;
    // ENC_B=_ENC_B;
    // wheel_radius = _wheel_radius;
    // rate = _rate;

    // A_motor(EN_A, IN1_A, IN2_A);
    // A_encoder(ENC_A,ENC_B,wheel_radius,rate);

    period = 1./rate * 1000;

   
}


void DCMotor::Init()
{
 // Wait for Serial Monitor to be opened
    A_motor.forward();
    A_motor.setSpeed(0);
    A_motor.stop();
    my_timer = millis(); // "сбросить" таймер

    A_encoder.Init();
    // delay(1000);
}

void DCMotor::Update()
{
    A_encoder.Update();

    if ((millis() - my_timer) >= period)
    {
        dt = (millis() - my_timer)*0.001;

        vel = A_encoder.GetVel();
        error = target_vel - vel;
        rateError = (error-last_error)/dt;
        last_error = error;        

        cumError += error * dt;  

        cumError = constrain(cumError, -iLim,iLim);
        output = error * kP + kI * cumError + rateError * kD;
         

        if (target_vel >= 0.0001)
        {
            A_motor.forward();
        }
        else
        {
            output = -output;
            A_motor.backward();
        }

        output = map(output, 0, 1., 0, 255.);
        if (abs(target_vel) < 0.1)
        {
            A_motor.setSpeed(0);
        }
        else
        {
            A_motor.setSpeed((int)output);
        }
        
        my_timer = millis(); // "сбросить" таймер
    }
}

float DCMotor::GetVel()
{
    return A_encoder.GetVel();
}
 
 float DCMotor::GetOutput()
{
    return output;
}

float DCMotor::GetTargetVel()
{
    return target_vel;
}


void DCMotor::SetVal(float targetVal)
{
    target_vel = targetVal;
}
void DCMotor::SetKp(float val)
{
    kP = val;
}

void DCMotor::SetKi(float val)
{
    kI = val;
}
void DCMotor::SetKd(float val)
{
    kD = val;
}