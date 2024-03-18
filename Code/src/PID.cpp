// Inclure les bibliothèques nécessaires
#include <iostream>
#include <unistd.h>
#include <cmath>

namespace pid
{
    PID::PID(float Kp, float Ki, float Kd, float limMax, float limMin, float dt): 
        Kp(Kp), Ki(Kp), Kd(Kd), limMax(limMax), limMin(limMin), dt(dt), 
        integral(0), prevError(0), derivative(0), prevMessurement(0), output(0){}
    
    // Fonction de contrôle PID pour l'axe roll
    float PID::update(float target, float current) {

        error = target - current;
        integral += error * dt;
        derivative = (error - prev_error) / dt;

        output = Kp * error + Ki * integral + Kd * derivative;
        prevError = error;

        if(output >= limMax){

            output = limMax;

        }else if(output <= limMin){

            output = limMin;
        }

        return output;
    }
    
}   



