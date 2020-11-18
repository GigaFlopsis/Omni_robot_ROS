/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/

#include "DCMotor.h"


float frame_rate = 10.;


unsigned long my_timer;
float time_period;

DCMotor motor_A(4,5,6, 2, 3, 0.075, 10);
DCMotor motor_B(13,12,11, 10, 9, 0.075, 10);
DCMotor motor_C(14,15,16, 17, 18, 0.075, 10);
DCMotor motor_D(22,24,26, 28, 30, 0.075, 10);


float a = 0.5;
float b = 0.5;

float motors_vel[4] = {0., 0., 0., 0.};

float cmd_vel[3] = {0., 0., 0.};

char receive_cmd;


void SplitVel(float vX, float vY, float w)
{
    motors_vel[0] = vX - vY - a*w - b*w;
    motors_vel[1] = vX + vY + a*w + b*w;
    motors_vel[2] = vX + vY - a*w -b*w;
    motors_vel[3] = vX - vY + a*w + b*w;
}

void SetVel()
{
    for (int i=0; i < 3 ; i++)
    {
        cmd_vel[i] = Serial.parseFloat();
        Serial.print("  :   ");
        Serial.print(cmd_vel[i]);
        Serial.println("");
    }
    
    SplitVel(cmd_vel[0], cmd_vel[1], cmd_vel[2]);
    
    motor_A.SetVal(motors_vel[0]);
    motor_B.SetVal(motors_vel[1]);
    motor_C.SetVal(motors_vel[2]);
    motor_D.SetVal(motors_vel[3]);
}

void SetPID()
{
    float val = Serial.parseFloat();
        motor_A.kP = val;
        motor_B.kP = val;
        motor_C.kP = val;
        motor_D.kP = val;
    val = Serial.parseFloat();
        motor_A.kI = val;
        motor_B.kI = val;
        motor_C.kI = val;
        motor_D.kI = val; 
    val = Serial.parseFloat();
        motor_A.kD = val;
        motor_B.kD = val;
        motor_C.kD = val;
        motor_D.kD = val;
}

void PrintInfo()
{
    Serial.print("GOAL      ");
    Serial.print("vel_XYW");
    for (int i=0; i < 3 ; i++)
    {
        Serial.print(": ");
        Serial.print(cmd_vel[i]);
    }
    Serial.print("    ");
    
    Serial.print("vel_ABSD");
    for (int i=0; i < 4 ; i++)
    {
        Serial.print(": ");
        Serial.print(motors_vel[i]);
    }
    Serial.println("");
    
    Serial.print("curr      ");

    Serial.print(" vel: ");
        Serial.print(motor_A.GetVel());
        Serial.print(" ");
        Serial.print(motor_B.GetVel());
        Serial.print(" ");
        Serial.print(motor_C.GetVel());
        Serial.print(" ");
        Serial.print(motor_D.GetVel());
    Serial.print(" sig: ");    
        Serial.print(motor_A.GetOutput());
        Serial.print(" ");
        Serial.print(motor_B.GetOutput());
        Serial.print(" ");
        Serial.print(motor_C.GetOutput());
        Serial.print(" ");
        Serial.print(motor_D.GetOutput());
        Serial.println(" ");
    
}

void PrintPID()
{
    Serial.print("PID       ");
    Serial.print("kP:");
        Serial.print(motor_A.kP);
    Serial.print("  kI:");
        Serial.print(motor_A.kI);
    Serial.print("  kD:");
        Serial.println(motor_A.kD);
}


void setup()
{
    time_period = 1./frame_rate * 1000;
    // Used to display information
    Serial.begin(230400);
    
    motor_A.Init();
    motor_B.Init();
    motor_C.Init();
    motor_D.Init();

    Serial.println(" init: ");

    delay(1000);
    my_timer = millis(); // "сбросить" таймер
}

void loop()
{
    if (Serial.available() > 0)
    {
      // Parsing data
      receive_cmd = Serial.read();
      if (receive_cmd == 'v')
      {
          Serial.print("Set vel\t");
          SetVel();
      }

    // Print PID info
      if (receive_cmd == 'i')
      {
        PrintPID();
      }

    // Set PID info
        if (receive_cmd == 'p')
        {
            Serial.println("Set PID\t");
            SetPID();
            PrintPID();
        }
    }

    // Update controls
    motor_A.Update();
    motor_B.Update();
    motor_C.Update();
    motor_D.Update();

    if ((millis() - my_timer) >= time_period)
    {
        PrintInfo();
        my_timer = millis(); // "reset timer
    }
}