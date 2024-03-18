#include <libmpu/MPU6050.h>
#include <libmotor/Motor.hpp>
#include <libpid/PID.hpp>
#include <iostream>

using namespace motor;

MPU6050 device(0x68);
PID pidRoll(1, 1, 1, 50, 0, 0.1);
PID pidPitch(1, 1, 1, 50, 0, 0.1);
PID pidYaw(1, 1, 1, 50, 0, 0.1);
Motor motor1(13);
Motor motor2(18);
Motor motor3(12);
Motor motor4(19);

float currentRoll, currentPitch, currentYaw = 0.0;
int targetRoll, targetPitch, targetYaw = 0;
float ax_off, ay_off, az_off, gr_off, gp_off, gy_off = 0.0;

int main() {

    sleep(1); // Attendre que le MPU6050 se stabilise 

    while (true) {

        device.calc_yaw = true;

        device.getOffsets(&ax_off, &ay_off, &az_off, &gr_off, &gp_off, &gy_off);

        // Lecture des angles
        device.getAngle(0 , &currentRoll);
        device.getAngle(1 , &currentPitch);
        device.getAngle(2 , &currentYaw);

        std::cout << "X : " << currentRoll << " Y : " << currentPitch << " Z : " << currentYaw << std::endl;

        /* motor1.setSpeed(speed - rollAdjust - pitchAdjust);
        motor2.setSpeed(speed + rollAdjust + pitchAdjust);
        motor3.setSpeed(speed - rollAdjust + pitchAdjust);
        motor4.setSpeed(speed + rollAdjust - pitchAdjust); */

        // Attendre le temps d'échantillonnage
        sleep(1);

        device.calc_yaw = false;
    } 

    return 0;
}