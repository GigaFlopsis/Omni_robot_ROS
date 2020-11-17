/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/

#include "DCMotor.h"


float frame_rate = 30.;


unsigned long my_timer;
float time_period;

DCMotor motor_A(4,5,6, 2, 3, 0.075, 10);
DCMotor motor_B(13,12,11, 10, 9, 0.075, 10);


float a = 0.5;
float b = 0.5;

float motors_vel[4] = {0., 0., 0., 0.};

float cmd_vel[3] = {0., 0., 0.};


void SplitVel(float vX, float vY, float w)
{
    motors_vel[0] = vX - vY - a*w - b*w;
    motors_vel[1] = vX + vY + a*w + b*w;
    motors_vel[2] = vX + vY - a*w -b*w;
    motors_vel[3] = vX - vY + a*w + b*w;
}


void setup()
{
    time_period = 1./frame_rate * 1000;
    // Used to display information
    Serial.begin(230400);
    
    motor_A.Init();
    motor_B.Init();

    Serial.println("  init: ");
    my_timer = millis(); // "сбросить" таймер
}

void loop()
{

    if (Serial.available() > 0)
    {
      for (int i=0; i < 3 ; i++)
      {
          cmd_vel[i] = Serial.parseFloat();

          Serial.print(cmd_vel[i]);
          Serial.print("  :   ");
      }
        SplitVel(cmd_vel[0], cmd_vel[1], cmd_vel[2]);
        motor_A.SetVal(motors_vel[0]);
        motor_B.SetVal(motors_vel[1]);

      Serial.println("");
    }

    motor_A.Update();
    motor_B.Update();

   
    if ((millis() - my_timer) >= time_period)
    {
        Serial.print("target: ");
        Serial.print(motor_A.GetTargetVel());
        Serial.print(" vel A: ");
        Serial.print(motor_A.GetVel());
        Serial.print("  output A: ");
        Serial.print(motor_A.GetOutput());
        Serial.print(" vel B: ");
        Serial.print(motor_B.GetVel());
        Serial.print("  output B: ");
        Serial.println(motor_B.GetOutput());
        my_timer = millis(); // "сбросить" таймер
    }
}