/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/

#include "DCMotor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

float frame_rate = 10.;
unsigned long my_timer;
float time_period;

DCMotor motor_A(1,2,3, 4, 5, 0.075, 10); // +
DCMotor motor_B(6,7,8, 9, 10, 0.075, 10);            //_EN_A, int _IN1_A, int _IN2_A,
DCMotor motor_C(13,12,11,14, 15, 0.075, 10); //+
DCMotor motor_D(15,16,17, 18, 19, 0.075, 10);


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

// ROS Node

std_msgs::Float32MultiArray motors_target_float_msg;
std_msgs::Float32MultiArray current_vel_float_msg;
geometry_msgs::Twist current_robot_vel_msg;

ros::Publisher motors_target_pub("motors/target", &motors_target_float_msg);
ros::Publisher motors_current_pub("motors/current", &current_vel_float_msg);
ros::Publisher robot_vel_pub("robot/velosity", &current_robot_vel_msg);

void CmdVelClb( const geometry_msgs::Twist& msgs)
{
    SplitVel(msgs.linear.x,msgs.linear.y,msgs.angular.z);
    
    motor_A.SetVal(motors_vel[0]);
    motor_B.SetVal(motors_vel[1]);
    motor_C.SetVal(motors_vel[2]);
    motor_D.SetVal(motors_vel[3]);

    for(int i;i < 4;i++)
    {
        motors_target_float_msg.data[i] = motors_vel[i]; 
    }
    motors_target_pub.publish(&motors_target_float_msg);   
}

void PublishMotorsVel()
{
    // Publish current velocity for each motor
    current_vel_float_msg.data[0] = motor_A.GetVel();
    current_vel_float_msg.data[1] = motor_B.GetVel();
    current_vel_float_msg.data[2] = motor_C.GetVel();
    current_vel_float_msg.data[3] = motor_D.GetVel();

    motors_current_pub.publish(&current_vel_float_msg);
}

void PublishRobotVel()
{
    // Publish current velocity for each motor
    current_robot_vel_msg.linear.x = 1.0;
    current_robot_vel_msg.linear.y = 2.0;
    current_robot_vel_msg.angular.z = 3.0;

    robot_vel_pub.publish(&current_robot_vel_msg);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", CmdVelClb);


void setup()
{    
    // init ros msgs
    motors_target_float_msg.data_length = 4;
    motors_target_float_msg.data = (float*)malloc(sizeof(float) * 4);

    current_vel_float_msg.data_length = 4;
    current_vel_float_msg.data = (float*)malloc(sizeof(float) * 4);

    time_period = 1./frame_rate * 1000;
    motor_A.Init();
    motor_B.Init();
    motor_C.Init();
    motor_D.Init();
    
    delay(1000);
    my_timer = millis(); // "сбросить" таймер
    
    // ros init
    nh.initNode();
    nh.advertise(motors_target_pub);
    nh.advertise(motors_current_pub);
    nh.advertise(robot_vel_pub);
    
    nh.subscribe(sub);
    
}


void loop()
{
     
    // // Update controls
    motor_A.Update();
    motor_B.Update();
    motor_C.Update();
    motor_D.Update();

    if((millis() - my_timer) >= time_period)
    {
        // PrintInfo();
        PublishMotorsVel();
        PublishRobotVel();
        my_timer = millis(); // "reset timer
    }
    nh.spinOnce();
}
