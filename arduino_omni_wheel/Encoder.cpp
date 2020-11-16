//
// Created by op on 16.11.2020.
//

#include "Encoder.h"

Encoder::Encoder(int _outputA, int _outputB, float _radius, int _rate = 10.)
{
    rate = _rate;
    period = 1.0/ _rate *1000.;
    outputA = _outputA;
    outputB = _outputB;
    dt = 1./_rate;
    pinMode (_outputA,INPUT);
    pinMode (_outputB,INPUT);

    // Reads the initial state of the outputA
    aLastState = digitalRead(outputA);

    my_timer = millis();   // "сбросить" таймер
}

void Encoder::Update()
{
    if ((millis() - my_timer) >= period)
    {
        velocity = radius*(counter_period*RAD_CICLE)/dt;

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
    }

    aState = digitalRead(outputA); // Reads the "current" state of the outputA
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (aState != aLastState){
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (digitalRead(outputB) != aState) {
//            counter += 0.5;
            counter_period += 0.5;
        } else {
//            counter -= 0.5;
            counter_period -= 0.5;
        }
    }
    aLastState = aState; // Updates the previous state of the outputA with the current state
}

float Encoder::GetVel()
{
    return velocity;
}

