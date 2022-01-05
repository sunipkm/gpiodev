#include <stdio.h>
#include <signal.h>
#include "gpiodev.h"

volatile sig_atomic_t done = 0;
void sig_handler(int sig)
{
    done = 1;
}

static char *modestr[] =
    {
        "in      ",
        "out     ",
        "irq_fall",
        "irq_rise",
        "irq_both",
        "error    "};

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
    while (!done)
    {
        switch (c)
        {
        case 's':
        case 'S':
            printf("Enter GPIO pin number: ");
            while (scanf(" %d", &idx) < 0)
                ;
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
            c = '\0';
            break;

        case 'i':
        case 'I':
            gpioSetMode(_idx, GPIO_IN);
            c = '\0';
            break;

        case 'o':
        case 'O':
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
        {
            int mode = gpioGetMode(_idx);
            if (mode < 0)
                mode = 5;
            printf("\nGPIO: %d | Mode: %s | Value: %d\n", idx, modestr[mode], gpioRead(_idx));
            if (mode == GPIO_OUT)
                printf("[s]elect GPIO, [i]nput mode, set [h]igh, set [l]ow, [q]uit: ");
            else
                printf("[s]elect GPIO, [o]utput mode, [q]uit: ");
            break;
        }
        }
        c = getchar();
    }
    printf("\n\n");
    return 0;
}