#include <libmotor/Motor.hpp>
#include <iostream>
#include <pigpio.h>

using namespace motor;

int main() {

    gpioInitialise();
    gpioSetMode(12, PI_OUTPUT);
    gpioWrite(12,0);

    time_sleep(5);
    std::cout << "Activation ..." << std::endl;
    time_sleep(1);
    gpioServo(12, 0);
    time_sleep(1);
    gpioServo(12, 700);
    time_sleep(2);
    gpioServo(12, 1230);
    time_sleep(2);

    for(int i = 1200; i < 1400; i +=5)
    {
	gpioServo(12, i);
	std::cout << i << std::endl;
	time_sleep(0.1);
    }

    for(int i = 1700; i < 1900; i += 5)
    {
	std::cout << i << std::endl;
	gpioServo(12, i);
	time_sleep(0.5);
    }

    gpioTerminate();

    return 0;
}
