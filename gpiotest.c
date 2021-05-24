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
    if (gpioInitialize() < 0)
    {
        eprintf("Could not initialize GPIO");
        return 0;
    }
    signal(SIGINT, &sig_handler); // set up signal handler
    char c = '\0';
    int _idx = 1, idx = 976;
    gpioSetMode(_idx, GPIO_OUT);
    while(!done)
    {
        switch(c)
        {
            case 's':
            case 'S':
                printf("Enter GPIO pin number: ");
                scanf(" %d", &idx);
                _idx = idx - 975;
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
    gpioDestroy();
}