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
    int c;
    printf("Setting pin to input, measure voltage at pin 11. Press enter to continue.\n");
    gpioSetMode(pin, GPIO_IN);
    while(((c = getchar()) != EOF) && (c != '\n'));
    printf("Wait a second...\n");
    sleep(1);
    gpioSetPullUpDown(pin, GPIO_PUD_UP);
    printf("Setting pin with pull up, measure voltage at pin 11 again. Press enter to continue\n");
    while(((c = getchar()) != EOF) && (c != '\n'));
    sleep(1);
    printf("The program will not exit until it receives an interrupt. To trigger an interrupt, short pin 11 to pin 9 (GND)\n");
    fflush(stdout);
    gpioWaitIRQ(pin, GPIO_IRQ_FALL, -1);
    printf("Interrupt received\n");
    return 0;
}