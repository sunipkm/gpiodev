#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "gpiodev.h"

volatile sig_atomic_t done = 0;
void sig_handler(int sig)
{
    done = 1;
}

static int counter = 0;
void gpio_callback(void *ptr)
{
    int *val = ptr;
    (*val)++;
    eprintf("IRQ triggered %d times", *val);
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
    gpioRegisterIRQ(pin, GPIO_IRQ_RISE, &gpio_callback, &counter, 1000); // register pin 11 as IRQ on rising edge, 1 second timeout
    while(!done)
    {
        sleep(1);
    }
    gpioUnregisterIRQ(pin);
    return 0;
}