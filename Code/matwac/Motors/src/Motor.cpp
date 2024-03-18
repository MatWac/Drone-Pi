#include <pigpio.h>
#include <iostream>
#include <libmotor/Motor.hpp>

namespace motor
{
    Motor::Motor(int pin) : m_pin(pin), m_level(0) {
        gpioInitialise();
        gpioSetMode(m_pin, PI_OUTPUT);
        gpioSetPWMrange(m_pin, 200);
        gpioPWM(m_pin, 130);
	time_sleep(2);
    }

    Motor::~Motor() {
        gpioPWM(m_pin, 0);
        gpioTerminate();
    }

    void Motor::setSpeed(int level) {

	m_level = level;

        if(level == 0){
	    gpioPWM(m_pin, 191);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 1){
	    gpioPWM(m_pin, 192);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 2){
	    gpioPWM(m_pin, 193);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 3){
	    gpioPWM(m_pin, 194);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 4){
	    gpioPWM(m_pin, 195);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 5){
	    gpioPWM(m_pin, 196);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 6){
	    gpioPWM(m_pin, 197);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 7){
	    gpioPWM(m_pin, 198);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 8){
	    gpioPWM(m_pin, 199);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else if(level == 9){
	    gpioPWM(m_pin, 200);
	    time_sleep(0.1);
	    std::cout << "Level : " << level << std::endl;
         }else {
	    std::cout << "Bad Level : " << level << std::endl;
         }
    }

    int Motor::getSpeed() const {
        return m_level;
    }
}