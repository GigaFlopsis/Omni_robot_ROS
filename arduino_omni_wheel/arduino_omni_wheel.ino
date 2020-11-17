/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/

#include "DCMotor.h"


float frame_rate = 10.;
// float wheel_radius = 0.075;

unsigned long my_timer;
float time_period;

DCMotor A_motor(4,5,6, 2, 3, 0.075, frame_rate);

void setup()
{
    time_period = 1./frame_rate * 1000;
    // Used to display information
    Serial.begin(115200);
    
    A_motor.Init();

    Serial.println("  init: ");
    my_timer = millis(); // "сбросить" таймер
}

void loop()
{
      if (Serial.available() > 0)
    {
        target_vel = Serial.parseFloat();
        Serial.println(target_vel);
    }


    A_motor.Update();

    if (Serial.av)

    if ((millis() - my_timer) >= time_period)
    {
        Serial.print("target: ");
        Serial.print(A_motor.GetTargetVel());

        Serial.print(" vel: ");
        Serial.print(A_motor.GetVel());
        

        Serial.print("  output: ");
        Serial.println(A_motor.GetOutput());
        
        my_timer = millis(); // "сбросить" таймер
    }
}