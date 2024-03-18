#include <libmpu/MPU6050.h>
#include <libmotor/Motor.hpp>
#include <iostream>

using namespace motor;

MPU6050 device(0x68);
Motor motor1(13);
Motor motor2(18);
Motor motor3(12);
Motor motor4(19);

float currentAngleX, currentAngleY = 0.0;

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
        device.getAngle(1 , &currentAngleY);

        std::cout << "AngleX : " << currentAngleX << std::endl;

        motor1.setSpeed((int)round(currentAngleX));

        // Attendre le temps d'échantillonnage
        usleep(1000);
    }

    return 0;
}
