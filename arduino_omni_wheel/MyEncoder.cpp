//
// Created by op on 16.11.2020.
//

#include "MyEncoder.h"

MyEncoder::MyEncoder(int _outputA, int _outputB, float _radius, int _rate = 10.) : Enc(_outputA, _outputB)
{
    rate = _rate;
    period = 1.0 / _rate * 1000.;
    outputA = _outputA;
    outputB = _outputB;
    dt = 1. / _rate;
}

void MyEncoder::Init()
{
    my_timer = millis(); // "сбросить" таймер
}

void MyEncoder::Update()
{
    if ((millis() - my_timer) >= period)
    {
        counter_period = Enc.read();
        velocity = radius * (counter_period * RAD_CICLE) / dt;

        //        path = radius*(counter*RAD_CICLE);
        //        Serial.print("Position: ");
        //        Serial.print(counter);
        //
        //        Serial.print("   path: ");
        //        Serial.print(path);
        //        Serial.print("   vel: ");
        //        Serial.println(velocity);

        my_timer = millis();
        counter_period = 0;
        Enc.write(0);
    }
}

float MyEncoder::GetVel()
{
    return velocity;
}
