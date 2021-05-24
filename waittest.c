#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "gpiodev.h"

volatile sig_atomic_t done = 0;
void sig_handler(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    int pin = -1;
    if (argc == 1)
    {
        if (gpiodev_pinout == PINOUT_RPI)
            pin = 11;
        else
        {
            eprintf("IRQ testing is only supported on Raspberry Pi");
            return 0;
        }
    }
    signal(SIGINT, &sig_handler); // set up signal handler
    printf("The program will not exit until it receives an interrupt\n");
    fflush(stdout);
    gpioWaitIRQ(pin, GPIO_IRQ_RISE, -1);
    printf("Interrupt received\n");
    return 0;
}