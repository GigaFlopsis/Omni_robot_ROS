/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/

#include "DCMotor.h"

float frame_rate = 2.;


unsigned long my_timer;
float time_period;

DCMotor motor_A(8,10,9, 32, 34, 0.075, 10); // +
DCMotor motor_B(2,3,4, 47, 49, 0.075, 10);            //_EN_A, int _IN1_A, int _IN2_A,
DCMotor motor_C(13,11,12, 28, 30, 0.075, 10); //+
DCMotor motor_D(7,6,5, 46, 48, 0.075, 10);


char char_array[100];
int availableBytes = 0;

float a = 0.1375;
float b = 0.265;

float motors_vel[4] = {0., 0., 0., 0.};

float cmd_vel[3] = {0., 0., 0.};

float receive_cmd[3] = {0.,0.,0.,};

void SplitVel(float vX, float vY, float w)
{
    motors_vel[0] = vX + vY - a*w - b*w;
    motors_vel[1] = vX - vY + a*w + b*w;
    motors_vel[2] = vX - vY - a*w -b*w;
    motors_vel[3] = vX + vY + a*w + b*w;
}

void SetVel()
{
    for (int i=0; i < 3 ; i++)
    {
        cmd_vel[i] = receive_cmd[i];
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
    float val = receive_cmd[0];
        motor_A.kP = val;
        motor_B.kP = val;
        motor_C.kP = val;
        motor_D.kP = val;
    val = receive_cmd[1];
        motor_A.kI = val;
        motor_B.kI = val;
        motor_C.kI = val;
        motor_D.kI = val; 
    val = receive_cmd[2];
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



void ParserData()
{
    int k = 0;
    String msg = "";
    for (int i=2; i < sizeof(char_array); i++)
        {
            msg+=char_array[i];          
            if (char_array[i] == ' ' || char_array[i] == NULL)
            {
                
                receive_cmd[k]=msg.toFloat();

                Serial.print(k);
                Serial.print(": ");
                Serial.println(receive_cmd[k]);

                k++;
                msg ="";
                if (k == 3)
                    break;
            }
            
        }
    // Serial.println("end ");
    // Serial.println(k);
}



void ClearArray()
{
        for(int i=0; i < sizeof(char_array); i++)
        {
            char_array[i] = NULL;
        }

}

void setup()
{

    
    time_period = 1./frame_rate * 1000;
    // Used to display information
    Serial.begin(1000000);
    Serial.setTimeout(100);

    motor_A.Init();
    motor_B.Init();
    motor_C.Init();
    motor_D.Init();

    delay(1000);
    my_timer = millis(); // "сбросить" таймер
}


void loop()
{
    
    while (Serial.available() >0 )
    {
        availableBytes = Serial.available();
        for(int i=0; Serial.available() > 0; i++)
        {
            char_array[i] = Serial.read();
            delay(1);
        }
        // Serial.println(char_array);
            
        if (char_array[0] == 'v')
        {
            Serial.println("Set vel\t");
            ParserData();
            SetVel();
        }

        // Print PID info
        if (char_array[0] == 'i')
        {
            PrintPID();
        }

        // Set PID info
        if (char_array[0] == 'p')
        {
            Serial.println("Set PID\t");
            ParserData();
            SetPID();
            PrintPID();
        }
        ClearArray();
    }

    // Update controls
    motor_A.Update();
    motor_B.Update();
    motor_C.Update();
    motor_D.Update();
        
    if((millis() - my_timer) >= time_period)
    {
        PrintInfo();
        my_timer = millis(); // "reset timer
    }
}
