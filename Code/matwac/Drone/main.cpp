#include <libmpu/MPU6050.h>
#include <libmotor/Motor.hpp>
#include <iostream>

using namespace motor;

MPU6050 device(0x68);
Motor motor1(13);
Motor motor2(18);
Motor motor3(12);
Motor motor4(19);

float kp = 1.0; // Constante proportionnelle du PID
float ki = 0.1; // Constante intégrale du PID
float kd = 0.01; // Constante dérivée du PID

float dt = 0.02; // Fréquence d'échantillonnage en secondes
float alpha = 0.1; // Coefficient de lissage du filtre passe-bas

// Variables pour le calcul du PID
float errorX = 0.0, errorY = 0.0;
float lastErrorX = 0.0, lastErrorY = 0.0;

float integralX = 0.0, integralY = 0.0;
float derivativeX = 0.0, derivativeY = 0.0;

float currentAngleX = 0.0, currentAngleY = 0.0;
float filteredAngleX = 0.0, filteredAngleY = 0.0;
float targetAngleX = 0.0, targetAngleY = 0.0;

void updatePID() {
    // Calcul des erreurs
    errorX = targetAngleX - filteredAngleX;
    errorY = targetAngleY - filteredAngleY;

    // Calcul des termes proportionnels
    float proportionalX = kp * errorX;
    float proportionalY = kp * errorY;

    // Calcul des termes intégraux
    integralX += ki * errorX * dt;
    integralY += ki * errorY * dt;

    // Calcul des termes dérivés
    derivativeX = kd * (errorX - lastErrorX) / dt;
    derivativeY = kd * (errorY - lastErrorY) / dt;

    // Calcul de la commande à appliquer à chaque moteur
    float motor1Speed = proportionalX - proportionalY - integralX + integralY - derivativeX + derivativeY;
    float motor2Speed = proportionalX + proportionalY + integralX + integralY + derivativeX + derivativeY;
    float motor3Speed = -proportionalX + proportionalY - integralX + integralY - derivativeX - derivativeY;
    float motor4Speed = -proportionalX - proportionalY + integralX + integralY + derivativeX - derivativeY;

    // Appliquer la commande à chaque moteur
    motor1.setSpeed((int)round(motor1Speed));
    motor2.setSpeed((int)round(motor2Speed));
    motor3.setSpeed((int)round(motor3Speed));
    motor4.setSpeed((int)round(motor4Speed));

    // Mise à jour des erreurs
    lastErrorX = errorX;
    lastErrorY = errorY;
}

int main() {

    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);

    sleep(1); // Attendre que le MPU6050 se stabilise

    device.calc_yaw = true;

    while (true) {

        // Lecture des angles
        device.getAngle(0 , &currentAngleX);
        device.getAngleY(1 , &currentAngleY);

        std::cout << "AngleX : " << currentAngleX << std::endl;
        std::cout << "AngleY : " << currentAngleY << std::endl;

        // Mise à jour du PID
        updatePID();

        std::cout << "Motor 1 : " << motor1.getSpeed() << std::endl;
        std::cout << "Motor 2 : " << motor2.getSpeed() << std::endl;
        std::cout << "Motor 3 : " << motor3.getSpeed() << std::endl;
        std::cout << "Motor 4 : " << motor4.getSpeed() << std::endl;

        // Attendre le temps d'échantillonnage
        usleep(dt * 1000000);
    }

    return 0;
}
