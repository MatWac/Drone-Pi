#include <pigpio.h>
#include <iostream>
#include <libmotor/Motor.hpp>

namespace motor
{
    Motor::Motor(int pin) : m_pin(pin), m_level(0) {

        gpioInitialise();
        std::cout << "Initilialisation ..." << std::endl;
        gpioSetMode(m_pin, PI_OUTPUT);
        gpioServo(m_pin, 0);
        time_sleep(2);
        std::cout << "Activation ..." << std::endl;
        gpioServo(m_pin, 700);
        time_sleep(2);
        std::cout << "Done" << std::endl;
    }

    Motor::~Motor() {
        gpioServo(m_pin, 0);
        gpioTerminate();
    }

    void Motor::setSpeed(int level) {

        if(level == -1)
        {
            gpioServo(m_pin, 700);

        }else if(level <= 100 && level >= 0){

            m_level = 1230 + 5*level;
            gpioServo(m_pin, m_level);

        }else{

            std::cout << "Bad Level" << std::endl;
        }
    }

    int Motor::getSpeed() const {
        return m_level;
    }
}