#include <pigpio.h>
#include <iostream>

int main()
{
    // Initialisation de pigpio
    if (gpioInitialise() < 0)
    {
        std::cerr << "Erreur lors de l'initialisation de pigpio." << std::endl;
        return 1;
    }

    // Numéro du GPIO à lire
    int gpio = 16;

    // Configuration du GPIO en entrée
    gpioSetMode(gpio, PI_INPUT);

    time_sleep(5);
    // Lecture de la valeur du GPIO
    int value = gpioRead(gpio);

    // Affichage de la tension
    std::cout << value << std::endl;
    // Libération des ressources utilisées par pigpio
    gpioTerminate();

    return 0;
}

