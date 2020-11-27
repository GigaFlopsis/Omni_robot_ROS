/*
Author  : Dmitry Devitt
Source  : https://github.com/AndreaLombardo/L298N/

*/
#define REG_RATE 30.0

#include "DCMotor.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <EEPROM.h>



ros::NodeHandle  nh;

float frame_rate = 10.;

unsigned long main_timer;
unsigned long safety_timer;
unsigned long publish_timer;
unsigned long current_time;

float time_period;

DCMotor motor_A(3,23,22, 14, 15, 0.075, REG_RATE); // +
DCMotor motor_B(4,20,21, 9, 10, 0.075, REG_RATE);            //_EN_A, int _IN1_A, int _IN2_A,
DCMotor motor_C(5,19,18,16, 17, 0.075, REG_RATE); //+
DCMotor motor_D(6,1,2, 12, 11, 0.075, REG_RATE);



float a = 0.1375;
float b = 0.265;

// values for an equation
float tang_dev_ab;

float motors_vel[4] = {0., 0., 0., 0.};
float cmd_vel[3] = {0., 0., 0.};
float receive_cmd[3] = {0.,0.,0.,};


void SplitVel(float vX, float vY, float w)
{
    motors_vel[0] = vX - vY - a*w - b*w;
    motors_vel[1] = vX - vY + a*w + b*w;
    motors_vel[2] = vX + vY - a*w -b*w;
    motors_vel[3] = vX + vY + a*w + b*w;
}


// ROS Node

std_msgs::Float32MultiArray motors_target_float_msg;
std_msgs::Float32MultiArray current_vel_float_msg;
geometry_msgs::Point pid_config_msg;

geometry_msgs::Twist current_robot_vel_msg;

ros::Publisher motors_target_pub("motors/goal_vel", &motors_target_float_msg);
ros::Publisher motors_current_pub("motors/current_vel", &current_vel_float_msg);
ros::Publisher robot_vel_pub("robot/velosity", &current_robot_vel_msg);
ros::Publisher pid_config_pub("motors/pid_param", &pid_config_msg);

void CmdVelClb( const geometry_msgs::Twist& msgs)
{
    SplitVel(msgs.linear.x,msgs.linear.y,msgs.angular.z);
    
    motor_A.SetVal(motors_vel[0]);
    motor_B.SetVal(motors_vel[1]);
    motor_C.SetVal(motors_vel[2]);
    motor_D.SetVal(motors_vel[3]);

    for(int i=0;i < 4;i++)
    {
        motors_target_float_msg.data[i] = motors_vel[i]; 
    }
    motors_target_pub.publish(&motors_target_float_msg);   

    safety_timer = millis(); // "reset timer
}


void SetVel(bool set_zero = false)
{
    if (set_zero)
    {
        for (int i=0; i < 3 ; i++)
        {
            cmd_vel[i] = 0;
        }
    }
    else
    {
         for (int i=0; i < 3 ; i++)
        {
            cmd_vel[i] = receive_cmd[i];
        }
    }

    SplitVel(cmd_vel[0], cmd_vel[1], cmd_vel[2]);
    
    motor_A.SetVal(motors_vel[0]);
    motor_B.SetVal(motors_vel[1]);
    motor_C.SetVal(motors_vel[2]);
    motor_D.SetVal(motors_vel[3]);


    for(int i=0;i < 4;i++)
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
        // Publish current velocity of robot

    current_robot_vel_msg.linear.x = (motor_A.GetVel()+motor_B.GetVel()+motor_C.GetVel()+motor_D.GetVel())/4.;
    current_robot_vel_msg.linear.y = (-motor_A.GetVel()
                                      + motor_B.GetVel()
                                      - motor_C.GetVel()
                                      + motor_D.GetVel())/4.;

    current_robot_vel_msg.angular.z = ((tang_dev_ab*motor_A.GetVel())-(tang_dev_ab*motor_B.GetVel())+(tang_dev_ab*motor_C.GetVel())-(tang_dev_ab*motor_D.GetVel()))/-4.;
    
    robot_vel_pub.publish(&current_robot_vel_msg);
}

void PublishPIDs()
{
    // ros publisher for puplish current PID params
    pid_config_msg.x = motor_A.kP;
    pid_config_msg.y = motor_A.kI;
    pid_config_msg.z = motor_A.kD;

    pid_config_pub.publish(&pid_config_msg);
}

void PIDconfigClb(const geometry_msgs::Point& data)
{
    // ros subscribeer for set  PID params
    float val = data.x;
        motor_A.kP = val;
        motor_B.kP = val;
        motor_C.kP = val;
        motor_D.kP = val;
    val = data.y;
        motor_A.kI = val;
        motor_B.kI = val;
        motor_C.kI = val;
        motor_D.kI = val; 
    val = data.z;
        motor_A.kD = val;
        motor_B.kD = val;
        motor_C.kD = val;
        motor_D.kD = val;

    PublishPIDs();

    UpdateDataFromStorage(data.x, data.y,data.z);
}

void PIDconfigClb(float p, float i, float d)
{
    // ros subscribeer for set  PID params
    float val = p;
        motor_A.kP = val;
        motor_B.kP = val;
        motor_C.kP = val;
        motor_D.kP = val;
    val = i;
        motor_A.kI = val;
        motor_B.kI = val;
        motor_C.kI = val;
        motor_D.kI = val; 
    val = d;
        motor_A.kD = val;
        motor_B.kD = val;
        motor_C.kD = val;
        motor_D.kD = val;

    PublishPIDs();

    // UpdateDataFromStorage(data.x, data.y,data.z);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", CmdVelClb);
ros::Subscriber<geometry_msgs::Point> pid_sub("/motors/pid/set", PIDconfigClb);



/* 
 * 
 * EEPROM data functions
 * 
*/

void RestoreDataFromStorage()
{
// restore data from memory

  float p,i,d = 0.00f;   //Variable to store data read from EEPROM.
  unsigned int eeAddress = 0; //EEPROM address to start reading fro  
  EEPROM.get( eeAddress, p );

  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  EEPROM.get( eeAddress, i );

  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  EEPROM.get( eeAddress, d );
  
  if (p < 0.0001 && i < 0.0001 && d < 0.0001)
    {
        // if data is empty, return
        return;
    }

    PIDconfigClb(p,i,d);

}

void UpdateDataFromStorage(float p,float i,float d)
{
    unsigned int eeAddress = 0;
    EEPROM.put(eeAddress, p);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, i);
    eeAddress += sizeof(float);
    EEPROM.put(eeAddress, d);
}
/* end */

void setup()
{    
    // init ros msgs
    motors_target_float_msg.data_length = 4;
    motors_target_float_msg.data = (float*)malloc(sizeof(float) * 4);

    current_vel_float_msg.data_length = 4;
    current_vel_float_msg.data = (float*)malloc(sizeof(float) * 4);

    time_period = 1./frame_rate * 1000;
    tang_dev_ab = 1. / (a+b);
    
    
    motor_A.Init();
    motor_B.Init();
    motor_C.Init();
    motor_D.Init();
    
    delay(1000);
    
    // init timers
    main_timer = millis(); // "сбросить" таймер
    safety_timer =  millis();
    publish_timer = millis();
    
    // ros init
    nh.initNode();
    nh.advertise(motors_target_pub);
    nh.advertise(motors_current_pub);
    nh.advertise(robot_vel_pub);
    nh.advertise(pid_config_pub);

    nh.subscribe(cmd_vel_sub);
    nh.subscribe(pid_sub);

    RestoreDataFromStorage();
}


void loop()
{
     // Update controls
    motor_A.Update();
    motor_B.Update();
    motor_C.Update();
    motor_D.Update();
    current_time = millis(); 
    
    if((current_time - main_timer) >= time_period)
    {
        // PrintInfo();
        PublishMotorsVel();
        PublishRobotVel();
        main_timer = millis(); // "reset timer
    }

    // publish data every 1 sec
    if((current_time - publish_timer) >= 1000)
    {
        PublishPIDs();
        publish_timer = millis(); // "reset timer
    }



    // safety stop after loose control to 1 sec
    if((current_time - safety_timer) >= 1000)
    {
        // PrintInfo();
        SetVel(true);       //publish zero
    }
    nh.spinOnce();
}
