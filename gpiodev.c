#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/file.h>
#include <errno.h>
#define GPIODEV_INTERNAL
#include "gpiodev.h"
#undef GPIODEV_INTERNAL

gpioprops __gpiodev_props_dev; /// Memory allocation for the GPIO properties struct
gpiopins __gpiodev_pins_dev;   /// Memory allocation for the GPIO pins struct

int __gpiodev_gpio_initd = 0; /// Default: Uninitialized

int gpioInitialize(void)
{
    // allow only one instance of gpioInitialize()
    int pid_file = open("/var/run/gpiodev.pid", O_CREAT | O_RDWR, 0666);
    int rc = flock(pid_file, LOCK_EX | LOCK_NB);
    if (rc)
    {
        if (EWOULDBLOCK == errno) // another instance is running
        {
            fprintf(stderr, "%s: Fatal error, another instance of software is running and trying to access gpiodev concurrently, aborting...\n", __func__);
            return -1;
        }
    }
    // memory allocations
    __gpiodev_props_dev.fd_val = (int *)malloc(NUM_GPIO_PINS * sizeof(int));
    __gpiodev_props_dev.fd_mode = (int *)malloc(NUM_GPIO_PINS * sizeof(int));
    __gpiodev_props_dev.val = (uint8_t *)malloc(NUM_GPIO_PINS * sizeof(uint8_t));
    __gpiodev_props_dev.mode = (uint8_t *)malloc(NUM_GPIO_PINS * sizeof(uint8_t));

    __gpiodev_pins_dev.fd = __gpiodev_props_dev.fd_val; // copy the value file descriptor array for access by gpioRead/gpioWrite
    __gpiodev_pins_dev.mode = (__gpiodev_props_dev.mode);
    __gpiodev_pins_dev.val = (__gpiodev_props_dev.val);
    for (int i = 0; i < NUM_GPIO_PINS; i++) // indicate all pins are uninitialized
    {
        __gpiodev_props_dev.fd_mode[i] = -1;
        __gpiodev_props_dev.fd_val[i] = -1;
    }
    int fd;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1)
    {
        fprintf(stderr, "%s: %s -> Failed to open export for writing.\n", __func__, "/sys/class/gpio/export");
        return -1;
    }
    __gpiodev_props_dev.fd_export = fd;
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1)
    {
        fprintf(stderr, "%s: %s -> Failed to open unexport for writing.\n", __func__, "/sys/class/gpio/unexport");
        return -1;
    }
    __gpiodev_props_dev.fd_unexport = fd;
    __gpiodev_gpio_initd = 1;
    return 1;
}

static int GPIOExport(int pin)
{
#define BUFFER_MAX 10
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(__gpiodev_props_dev.fd_export, buffer, bytes_written);
    return (0);
}

static int GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(__gpiodev_props_dev.fd_unexport, buffer, bytes_written);
    return (0);
}

int gpioSetMode(int pin, int mode)
{
    if (__gpiodev_gpio_initd == 0) // GPIO uninitialized
    {
        if (gpioInitialize() < 0)
            return -1;
        __gpiodev_gpio_initd = 1; // Indicate that GPIO has been initialized
    }
    int bcmpin = __gpiodev_gpio_lut_pins[pin];
    if (bcmpin < 0)
    {
        fprintf(stderr, "GPIODEV: Error, pin %d is not available for GPIO operation.\n", pin);
        return -1;
    }

    if (mode != GPIO_IN && mode != GPIO_OUT)
    {
        fprintf(stderr, "GPIODEV: Error, mode is not GPIO_IN or GPIO_OUT for pin %d.\n", pin);
    }
    if (!GPIOExport(bcmpin)) // pin export successful
    {
        char path[256];
        int fd = -1;

        if (__gpiodev_props_dev.fd_mode[pin] < 0) // pin mode uninitialized
        {
            snprintf(path, 256, "/sys/class/gpio/gpio%d/direction", bcmpin); // Direction fd
            fd = open(path, O_WRONLY);
            if (fd == -1)
            {
                fprintf(stderr, "%s: Failed to open gpio %d direction for writing: %s\n", __func__, bcmpin, path);
                return (-1);
            }
            __gpiodev_props_dev.fd_mode[pin] = fd; // save the direction file descriptor
        }
        char modestr[] = "in\0out";
        if (write(fd, &modestr[mode == GPIO_IN ? 0 : 3], mode == GPIO_IN ? 2 : 3) == -1)
        {
            fprintf(stderr, "%s: Failed to set direction for pin %d!\n", __func__, bcmpin);
            return (-1);
        }

        // open file for value access
        if (__gpiodev_props_dev.fd_val[pin] < 0)
        {
            snprintf(path, 256, "/sys/class/gpio/gpio%d/value", bcmpin);
            fd = open(path, mode == GPIO_IN ? O_RDONLY : O_WRONLY); // Open as read/write depending on mode
            if (fd == -1)
            {
                fprintf(stderr, "%s: Failed to open gpio value for read/write: %s\n", __func__, path);
                return (-1);
            }
            __gpiodev_props_dev.fd_val[pin] = fd;
        }
        if (mode == GPIO_OUT)
        {
            __gpiodev_props_dev.val[pin] = GPIO_LOW;
            gpioWrite(pin, GPIO_LOW);
        }
        else
        {
            __gpiodev_props_dev.val[pin] = gpioRead(pin);
        }
    }
    return 1;
}

int gpioRead(int pin)
{
    char value_str[3];
    lseek(__gpiodev_pins_dev.fd[pin], 0, SEEK_SET);
    if (read(__gpiodev_pins_dev.fd[pin], value_str, 3) == -1)
    {
        fprintf(stderr, "%s: Failed to read value from pin %d!\n", __func__, pin);
        return (-1);
    }
    return (atoi(value_str));
}

int gpioWrite(int pin, int value)
{
    static const char s_values_str[] = "01";

    if (1 != write(__gpiodev_pins_dev.fd[pin], &s_values_str[GPIO_LOW == value ? 0 : 1], 1))
    {
        fprintf(stderr, "%s: Failed to write value to %d!\n", __func__, pin);
        return (-1);
    }
    return (0);
}

void gpioDestroy(void)
{
    for (int i = 0; i < NUM_GPIO_PINS; i++)
    {
        if (__gpiodev_props_dev.fd_mode[i] >= 0) // if opened
        {
            gpioWrite(i, GPIO_LOW);
            close(__gpiodev_props_dev.fd_val[i]);
            close(__gpiodev_props_dev.fd_mode[i]);
            GPIOUnexport(__gpiodev_gpio_lut_pins[i]);
        }
    }
    close(__gpiodev_props_dev.fd_export);
    close(__gpiodev_props_dev.fd_unexport);
    free(__gpiodev_props_dev.fd_val);
    free(__gpiodev_props_dev.fd_mode);
    free(__gpiodev_props_dev.val);
    free(__gpiodev_props_dev.mode);
    return;
}

#ifdef UNIT_TEST
#include <stdio.h>
#include <signal.h>

volatile sig_atomic_t done = 0;
void sig_handler(int sig)
{
    done = 1;
}

int main()
{
    gpioInitialize();
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

#endif
