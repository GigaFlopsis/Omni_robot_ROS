/*
Author  : Andrea Lombardo
Site    : https://www.lombardoandrea.com
Source  : https://github.com/AndreaLombardo/L298N/

Based on Arduino Basic Fade example.

L298NX2 is not a new version of module or IC,
but it stands for a double implementation of L298N library.

With L298NX2 is possible to initialize two motors at once.

Speed range go from 0 to 255, default is 100.
Use setSpeed(speed) to change speed for both motors,
setSpeedA(speed) or setSpeedB(speed) for individual changes.

Sometimes at lower speed motors seems not running.
It's normal, may depends by motor and power supply.

Wiring schema in file "L298NX2 - Schema_with_EN_pin.png"
*/

// Include the (new) library
#include <L298NX2.h>
#include "Encoder.h"

// Pin definition
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

float kP = 0.12;
float kI = 0.23;
float kD = 0.001;
float dt;

float error = 0;
float last_error = 0;
float rateError = 0;
float cumError = 0;

float iLim = 0.4;

float target_vel = 0.0;

// Initialize both motors
L298N A_motor(EN_A, IN1_A, IN2_A);
Encoder A_encoder(14, 15, 0.075, 10);

//L298NX2 backward_motors(EN_C, IN1_C, IN2_C, EN_D, IN1_D, IN2_D);
float vel;
float period = 200;

unsigned long my_timer;



float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  x = constrain(x,in_min,in_max);
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
    // Used to display information
    Serial.begin(115200);
    // Wait for Serial Monitor to be opened

    A_motor.forward();
    A_motor.setSpeed(0);

        my_timer = millis(); // "сбросить" таймер

    A_motor.stop();
    delay(1000)
}

void loop()
{
    if (Serial.available() > 0)
    {
        target_vel = Serial.parseFloat();
        Serial.println(target_vel);
    }
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
        float output = error * kP + kI * cumError + rateError * kD;
         

        if (target_vel >= 0.0001)
        {
            A_motor.forward();
        }
        else
        {
            output = -output;
            A_motor.backward();
        }


        // Print info
        Serial.print(" error: ");
        Serial.print(error);

        Serial.print(" cumError: ");
        Serial.print(cumError);


        Serial.print(" sig: ");
        Serial.print(output);

        Serial.print(" target: ");
        Serial.print(target_vel);

        Serial.print(" vel: ");
        Serial.println(vel);

        output = map(output, 0, 1., 0, 255.);

        if (abs(target_vel) < 0.05)
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
