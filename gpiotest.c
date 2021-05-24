#include <stdio.h>
#include <signal.h>
#include "gpiodev.h"

volatile sig_atomic_t done = 0;
void sig_handler(int sig)
{
    done = 1;
}

int main()
{
    signal(SIGINT, &sig_handler); // set up signal handler
    char c = '\0';
    int _idx = 1, idx = 976;
    if (gpiodev_pinout == PINOUT_RPI)
    {
        idx = 13;
        _idx = 13;
    }
    else if (gpiodev_pinout == PINOUT_AD9361)
    {
        idx = 976;
        _idx = 1;
    }
    else if (gpiodev_pinout == PINOUT_AD9364)
    {
        idx = 960;
        _idx = 0;
    }
    gpioSetMode(_idx, GPIO_OUT);
    while (!done)
    {
        switch (c)
        {
        case 's':
        case 'S':
            printf("Enter GPIO pin number: ");
            while (scanf(" %d", &idx) < 0);
            if (gpiodev_pinout == PINOUT_RPI)
            {
                _idx = idx;
            }
            else if (gpiodev_pinout == PINOUT_AD9361)
            {
                _idx = idx - 976;
            }
            else if (gpiodev_pinout == PINOUT_AD9364)
            {
                _idx = idx - 960;
            }
            gpioSetMode(_idx, GPIO_OUT);
            c = '\0';
            break;

        case 'h':
        case 'H':
            gpioWrite(_idx, GPIO_HIGH);
            c = '\0';
            break;

        case 'l':
        case 'L':
            gpioWrite(_idx, GPIO_LOW);
            c = '\0';
            break;

        case 'q':
        case 'Q':
            done = 1;
            continue;
            break;
        case '\n':
            c = '\0';
            continue;
            break;
        default:
            printf("GPIO: %d, [s]elect GPIO, toggle [h]igh, toggle [l]ow, [q]uit: ", idx);
            break;
        }
        c = getchar();
    }
    return 0;
}