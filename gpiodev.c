#define _GNU_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/file.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#define GPIODEV_INTERNAL
#include "gpiodev.h"
#undef GPIODEV_INTERNAL

static gpioprops gpio_props_dev;                    /// Memory allocation for the GPIO properties struct
static gpiopins gpio_pins_dev;                      /// Memory allocation for the GPIO pins struct
static pthread_t *gpio_irq_threads;                 /// Memory allocation for IRQ threads
static gpio_irq_params *irq_params;                 /// Memory allocation for IRQ params
static volatile int fdmem = -1;                     /// Memory allocation for /dev/mem
static volatile uint32_t *gpio_reg = MAP_FAILED;    /// Memory allocation for GPIO register base pointer
static volatile uint32_t *syst_reg = MAP_FAILED;    /// Memory allocation for system register (for gpio delay)
static volatile uint32_t pi_peri_phys = 0x20000000; /// Base pointer for peripheral system
static volatile bool pi_ispi = false;
static volatile bool pi_is_2711 = false;
static volatile bool pi_pud_avail = false;
static volatile bool pi_syst_avail = false;
static volatile bool pi_mmap_rdy = false;

#define GPIO_BASE (pi_peri_phys + 0x00200000)
#define GPIO_LEN 0xF4 /* 2711 has more registers */

#define SYST_BASE (pi_peri_phys + 0x00003000)
#define SYST_LEN 0x1C

#define SYST_CLO 1

static int gpio_initd = 0; /// Default: Uninitialized

static uint32_t gpioHardwareRevision(void);

static uint32_t *map_mem(int fd, uint32_t addr, uint32_t len)
{
    return (uint32_t *)mmap(0, len, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, addr);
}

int gpioInitialize(void)
{
#if GPIODEV_SINGLE_INSTANCE > 0
    // allow only one instance of gpioInitialize()
    eprintf("Initialized in single instance mode");
    int pid_file = open("/var/run/gpiodev.pid", O_CREAT | O_RDWR, 0666);
    int rc = flock(pid_file, LOCK_EX | LOCK_NB);
    if (rc)
    {
        if (EWOULDBLOCK == errno) // another instance is running
        {
            fprintf(stderr, "%s: Fatal error, another instance of software is running and trying to access gpiodev concurrently, aborting...\n", __func__);
            exit(0);
        }
    }
#endif
    // memory allocations
    gpio_props_dev.fd_val = (int *)malloc(NUM_GPIO_PINS * sizeof(int));
    gpio_props_dev.fd_mode = (int *)malloc(NUM_GPIO_PINS * sizeof(int));
    gpio_props_dev.val = (uint8_t *)malloc(NUM_GPIO_PINS * sizeof(uint8_t));
    gpio_props_dev.mode = (uint8_t *)malloc(NUM_GPIO_PINS * sizeof(uint8_t));
    gpio_props_dev.pud = (uint8_t *)malloc(NUM_GPIO_PINS * sizeof(uint8_t));
    memset(gpio_props_dev.pud, 0x0, sizeof(uint8_t)); // initiate as pull up turned off
    gpio_irq_threads = (pthread_t *)malloc(NUM_GPIO_PINS * sizeof(pthread_t));
    irq_params = (gpio_irq_params *)malloc(NUM_GPIO_PINS * sizeof(gpio_irq_params));

    gpio_pins_dev.fd = gpio_props_dev.fd_val; // copy the value file descriptor array for access by gpioRead/gpioWrite
    gpio_pins_dev.mode = (gpio_props_dev.mode);
    gpio_pins_dev.val = (gpio_props_dev.val);
    for (int i = 0; i < NUM_GPIO_PINS; i++) // indicate all pins are uninitialized
    {
        gpio_props_dev.fd_mode[i] = -1;
        gpio_props_dev.fd_val[i] = -1;
        gpio_props_dev.mode[i] = -1;
    }
    int fd;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1)
    {
        fprintf(stderr, "%s: %s -> Failed to open export for writing.\n", __func__, "/sys/class/gpio/export");
        goto cleanup;
    }
    gpio_props_dev.fd_export = fd;
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1)
    {
        fprintf(stderr, "%s: %s -> Failed to open unexport for writing.\n", __func__, "/sys/class/gpio/unexport");
        goto cleanup;
    }
    gpio_props_dev.fd_unexport = fd;
    // obtain hardware revision, check if Pi
    uint32_t rev = gpioHardwareRevision();
#ifdef GPIODEBUG
    eprintf("GPIO revision: 0x%x", rev);
#else
    ((void) rev);
#endif
    if (pi_ispi) // if it is a pi
    {
        if ((fdmem = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
        {
            eprintf("Could not open /dev/mem");
        }
        else
        {
            pi_mmap_rdy = true;
            gpio_reg = map_mem(fdmem, GPIO_BASE, GPIO_LEN);
            if ((gpio_reg != NULL) && (gpio_reg != MAP_FAILED))
            {
                pi_pud_avail = true;
            }
            else
            {
                eprintf("Failed to map gpio registers");
            }
            syst_reg = map_mem(fdmem, SYST_BASE, SYST_LEN);
            if ((syst_reg != NULL) && (syst_reg != MAP_FAILED)) // meaning the previous step was successful
            {
                pi_syst_avail = true;
            }
            else
            {
                eprintf("Failed to map system registers");
            }
        }
    }
    gpio_initd = 1;
    atexit(gpioDestroy);
    return 1;
cleanup:
    gpio_initd = 1;
    gpioDestroy();
    return -1;
}

static int GPIOExport(int pin)
{
#define BUFFER_MAX 10
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(gpio_props_dev.fd_export, buffer, bytes_written);
    return (0);
}

static int GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(gpio_props_dev.fd_unexport, buffer, bytes_written);
    return (0);
}

int gpioSetMode(int pin, enum GPIO_MODE mode)
{
    static char modestr[] = "in\0out";
    char path[256];
    int fd = -1;
    if (gpio_initd == 0) // GPIO uninitialized
    {
        if (gpioInitialize() < 0)
            return -1;
        gpio_initd = 1; // Indicate that GPIO has been initialized
    }
    int bcmpin = gpio_lut_pins[pin];
    if (bcmpin < 0)
    {
        fprintf(stderr, "GPIODEV: Error, pin %d is not available for GPIO operation.\n", pin);
        return -1;
    }
    if (mode > GPIO_IRQ_BOTH)
    {
        fprintf(stderr, "GPIODEV: Error, mode is not valid for pin %d.\n", pin);
    }
    if (gpio_props_dev.fd_val[pin] > 0)
    {
        goto set_mode; // override opening direction again
    }
    if (GPIOExport(bcmpin)) // pin export unsuccessful
        return -1;

    if (gpio_props_dev.fd_mode[pin] < 0) // pin mode uninitialized
    {
        snprintf(path, 256, "/sys/class/gpio/gpio%d/direction", bcmpin); // Direction fd
        fd = open(path, O_WRONLY);
        if (fd == -1)
        {
            fprintf(stderr, "%s: Failed to open gpio %d direction for writing: %s\n", __func__, bcmpin, path);
            return (-1);
        }
        gpio_props_dev.fd_mode[pin] = fd; // save the direction file descriptor
    }
set_mode:
    if (write(gpio_props_dev.fd_mode[pin], &modestr[mode == GPIO_OUT ? 3 : 0], mode == GPIO_OUT ? 3 : 2) == -1) // default: in
    {
        fprintf(stderr, "%s: Failed to set direction for pin %d!\n", __func__, bcmpin);
        return (-1);
    }
    gpio_pins_dev.mode[pin] = mode; // save the mode in which the pin has been opened
    // open file for value access
    if (gpio_props_dev.fd_val[pin] < 0)
    {
        snprintf(path, 256, "/sys/class/gpio/gpio%d/value", bcmpin);
        fd = open(path, mode == GPIO_OUT ? O_WRONLY : O_RDONLY); // Open as read/write depending on mode
        if (fd == -1)
        {
            fprintf(stderr, "%s: Failed to open gpio value for read/write: %s\n", __func__, path);
            return (-1);
        }
        gpio_props_dev.fd_val[pin] = fd;
    }
    if (mode == GPIO_OUT)
    {
        gpio_props_dev.val[pin] = GPIO_LOW;
        gpioWrite(pin, GPIO_LOW);
    }
    else if (mode == GPIO_IN)
    {
        gpio_props_dev.val[pin] = gpioRead(pin);
        // set up IRQ to none
        snprintf(path, 256, "/sys/class/gpio/gpio%d/edge", bcmpin);
        fd = open(path, O_WRONLY); // open as write
        if (fd < 0)
        {
            eprintf("Error opening %s for write", path);
        }
        static char MODE_NONE[] = "none";
        if (write(fd, MODE_NONE, sizeof(MODE_NONE) - 1) < 0)
        {
            eprintf("Error disabling interrupt on pin %d", pin);
        }
        close(fd);
    }
    return 1;
}

int gpioRead(int pin)
{
    char value_str[3];
    lseek(gpio_pins_dev.fd[pin], 0, SEEK_SET);
    if (read(gpio_pins_dev.fd[pin], value_str, 3) == -1)
    {
        fprintf(stderr, "%s: Failed to read value from pin %d!\n", __func__, pin);
        return (-1);
    }
    return (atoi(value_str));
}

int gpioWrite(int pin, int value)
{
    static const char s_values_str[] = "01";

    if (1 != write(gpio_pins_dev.fd[pin], &s_values_str[GPIO_LOW == value ? 0 : 1], 1))
    {
        fprintf(stderr, "%s: Failed to write value to %d!\n", __func__, pin);
        return (-1);
    }
    return (0);
}

// this function sets up the IRQ thread
static void *gpio_irq_thread(void *params)
{
    gpio_irq_params *irqparams = (gpio_irq_params *)params;
    gpioRead(irqparams->pin);                                                             // consume pending IRQs
    struct pollfd pfd = {.fd = gpio_props_dev.fd_val[irqparams->pin], .events = POLLPRI}; // set up poll
    while (1)
    {
        int pollret = poll(&pfd, 1, irqparams->tout_ms); // block until something happens
        if (pollret > 0)                                 // something happened
        {
            if (pfd.revents == POLLHUP)
            {
                eprintf("pollhup on pin %d", irqparams->pin);
            }
            else if (pfd.revents == POLLNVAL)
            {
                eprintf("pollnval on pin %d", irqparams->pin);
            }
            else if (pfd.revents == POLLERR)
            {
                eprintf("pollerr on pin %d", irqparams->pin);
            }
            else
            {
                irqparams->callback(irqparams->userdata); // perform the callback
            }
            gpioRead(irqparams->pin); // clear IRQ
        }
        else if (pollret == 0) // timeout
        {
#ifdef GPIODEBUG
            eprintf("IRQ timeout on pin %d", irqparams->pin);
#endif
            continue;
        }
        else if (pollret < 0) // error
        {
            eprintf("Error on poll pin %d", irqparams->pin);
            perror("gpiodev poll");
            break;
        }
    }
    return NULL;
}

int gpioWaitIRQ(int pin, enum GPIO_MODE mode, int tout_ms)
{
    static char MODE_FALLING[] = "falling";
    static char MODE_RISING[] = "rising";
    static char MODE_BOTH[] = "both";
    static char MODE_NONE[] = "none";
    int retval = -1;
    if ((mode < GPIO_IRQ_FALL) || (mode > GPIO_IRQ_BOTH))
    {
        eprintf("Invalid pin mode");
        return -1;
    }
    if ((gpio_initd == 0) || (gpio_pins_dev.mode[pin] > GPIO_IRQ_BOTH))
    {
        if (gpioSetMode(pin, mode) < 0)
        {
            eprintf("Error setting pin mode");
            return -1;
        }
    }
    char *irq_mode;
    int irq_mode_bytes = 0;
reinit:
    if (gpio_props_dev.mode[pin] == GPIO_IRQ_FALL)
    {
        irq_mode = MODE_FALLING;
        irq_mode_bytes = sizeof(MODE_FALLING) - 1;
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IRQ_RISE)
    {
        irq_mode = MODE_RISING;
        irq_mode_bytes = sizeof(MODE_RISING) - 1;
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IRQ_BOTH)
    {
        irq_mode = MODE_BOTH;
        irq_mode_bytes = sizeof(MODE_BOTH) - 1;
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IN)
    {
        gpio_props_dev.mode[pin] = mode;
        goto reinit;
    }
    else
    {
        eprintf("IRQ mode value %d on pin %d, not supported", gpio_props_dev.mode[pin], pin);
        goto exit;
    }
    // set up interrupt
    char fname[256];
    snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/edge", gpio_lut_pins[pin]); // create edge select
    int fd = open(fname, O_RDWR);
    if (fd < 0)
    {
        eprintf("Pin %d does not support interrupt generation", pin);
        goto exit;
    }
    if (write(fd, irq_mode, irq_mode_bytes) != irq_mode_bytes) // write edge select
    {
        eprintf("Could not set IRQ mode setting on pin %d", pin);
        goto cleanup;
    }
    // set up poll
    struct pollfd pfd = {.fd = gpio_pins_dev.fd[pin], .events = POLLPRI};
    // clear IRQ
    gpioRead(pin);
    retval = poll(&pfd, 1, tout_ms);
    if (retval > 0) // something happened
    {
        if (pfd.revents == POLLHUP)
        {
            eprintf("pollhup on pin %d", pin);
            retval = -1;
        }
        else if (pfd.revents == POLLNVAL)
        {
            eprintf("pollnval on pin %d", pin);
            retval = -1;
        }
        else if (pfd.revents == POLLERR)
        {
            eprintf("pollerr on pin %d", pin);
            retval = -1;
        }
        else
        {
            retval = 1; // indicate interrupt received
        }
        gpioRead(pin); // clear IRQ
    }
    else if (retval == 0) // timeout
    {
        retval = 0;
    }
    else
    {
        retval = -1;
        eprintf("Error on poll pin %d", pin);
        perror("gpiodev poll");
    }
    irq_mode = MODE_NONE;
    irq_mode_bytes = sizeof(MODE_NONE) - 1;
    if (write(fd, irq_mode, irq_mode_bytes) != irq_mode_bytes) // disable edge select
    {
        eprintf("Could not set IRQ mode setting on pin %d", pin);
    }
cleanup:
    close(fd);
exit:
    return retval;
}

int gpioRegisterIRQ(int pin, enum GPIO_MODE mode, gpio_irq_callback_t func, void *userdata, int tout_ms)
{
    int retval = -1;
    if ((gpio_initd == 0) || (gpio_pins_dev.mode[pin] > GPIO_IRQ_BOTH))
    {
        if (gpioSetMode(pin, mode) < 0)
        {
            eprintf("Error setting pin mode");
            return -1;
        }
    }
    char irq_mode[20];
    int irq_mode_bytes = 0;
    if (gpio_props_dev.mode[pin] == GPIO_OUT) // should never trigger
    {
        eprintf("Pin is output, IRQ not available");
        goto exit;
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IN) // should never trigger
    {
        eprintf("Pin is input, IRQ trigger not set");
        goto exit;
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IRQ_FALL)
    {
        irq_mode_bytes = snprintf(irq_mode, sizeof(irq_mode), "falling");
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IRQ_RISE)
    {
        irq_mode_bytes = snprintf(irq_mode, sizeof(irq_mode), "rising");
    }
    else if (gpio_props_dev.mode[pin] == GPIO_IRQ_BOTH)
    {
        irq_mode_bytes = snprintf(irq_mode, sizeof(irq_mode), "both");
    }
    else
    {
        eprintf("IRQ mode value %d on pin %d, not supported", gpio_props_dev.mode[pin], pin);
        goto exit;
    }
    char fname[256];
    snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/edge", gpio_lut_pins[pin]); // create edge select
    int fd = open(fname, O_RDWR);
    if (fd < 0)
    {
        eprintf("Pin %d does not support interrupt generation", pin);
        goto exit;
    }
    if (write(fd, irq_mode, irq_mode_bytes) != irq_mode_bytes) // write edge select
    {
        eprintf("Could not set IRQ mode setting on pin %d", pin);
        goto cleanup;
    }
    gpio_irq_params *param = &(irq_params[pin]);
    param->pin = pin;
    param->tout_ms = tout_ms;
    param->userdata = userdata;
    param->callback = func;
    int rc = pthread_create(&(gpio_irq_threads[pin]), NULL, &gpio_irq_thread, param);
    if (rc)
    {
        eprintf("Error creating IRQ thread for pin %d", pin);
        perror("pthread_create");
    }
    retval = 1;
cleanup:
    close(fd);
exit:
    return retval;
}

int gpioUnregisterIRQ(int pin)
{
    if (gpio_pins_dev.mode[pin] == GPIO_IN)
    {
        eprintf("No IRQ registered on pin, exiting");
        return 0;
    }
    else if (gpio_pins_dev.mode[pin] == GPIO_OUT)
    {
        eprintf("Can not unregister IRQ on output pin");
        return 0;
    }
    if ((gpio_pins_dev.mode[pin] > GPIO_OUT) && (gpio_pins_dev.mode[pin] <= GPIO_IRQ_BOTH)) // valid pin
    {
        pthread_cancel(gpio_irq_threads[pin]); // cancel the IRQ thread for the pin
        return gpioSetMode(pin, GPIO_IN);      // set the pin to input
    }
    else
    {
        eprintf("Pin not initialized");
        return -1;
    }
    return -1; // should never reach this point
}

void gpioDestroy(void)
{
    if (gpio_initd)
    {
        if (pi_ispi) // take care of RPi config
        {
            if (pi_pud_avail)
            {
                for (int i = 0; i < NUM_GPIO_PINS; i++)
                    if (gpio_props_dev.pud[i]) // pull up/down set
                        gpioSetPullUpDown(i, GPIO_PUD_OFF);
                munmap((void *)gpio_reg, GPIO_LEN);
                pi_pud_avail = false;
            }
            if (pi_syst_avail)
            {
                munmap((void *)syst_reg, SYST_LEN);
                pi_syst_avail = false;
            }
            if (pi_mmap_rdy)
            {
                close(fdmem);
                pi_mmap_rdy = false;
            }
            pi_ispi = false;
        }
        for (int i = 0; i < NUM_GPIO_PINS; i++) // close all running IRQs
            pthread_cancel(gpio_irq_threads[i]);
        for (int i = 0; i < NUM_GPIO_PINS; i++)
        {
            if (gpio_props_dev.fd_mode[i] >= 0) // if opened, close pins
            {
                if (gpio_props_dev.mode[i] == GPIO_OUT) // set pin to low if pin is output
                    gpioWrite(i, GPIO_LOW);
                close(gpio_props_dev.fd_val[i]);
                close(gpio_props_dev.fd_mode[i]);
                GPIOUnexport(gpio_lut_pins[i]);
            }
        }
        close(gpio_props_dev.fd_export);
        close(gpio_props_dev.fd_unexport);
        free(gpio_props_dev.fd_val);
        free(gpio_props_dev.fd_mode);
        free(gpio_props_dev.val);
        free(gpio_props_dev.mode);
        free(gpio_irq_threads);
        free(irq_params);
#if GPIODEV_SINGLE_INSTANCE > 0
        int pid_file = open("/var/run/gpiodev.pid", O_RDWR); // should be open
        int rc = flock(pid_file, LOCK_UN);
        if (rc)
        {
            eprintf("Error unlocking PID file");
        }
        unlink("/var/run/gpiodev.pid");
        gpio_initd = 0;
#endif
    }
    return;
}

uint32_t gpioHardwareRevision(void)
{
    static uint32_t rev = 0;

    FILE *fp;
    char buf[512];
    char term;

    if (rev)
        return rev;

    fp = fopen("/proc/cpuinfo", "r");

    if (fp != NULL)
    {
        while (fgets(buf, sizeof(buf), fp) != NULL)
        {
            if (!strncasecmp("revision\t:", buf, 10))
            {
                if (sscanf(buf + 10, "%x%c", &rev, &term) == 2)
                {
                    if (term != '\n')
                        rev = 0;
                }
            }
        }
        fclose(fp);
    }

    /* (some) arm64 operating systems get revision number here  */

    if (rev == 0)
    {
        fp = fopen("/proc/device-tree/system/linux,revision", "r");

        if (fp != NULL)
        {
            uint32_t tmp;
            if (fread(&tmp, 1, 4, fp) == 4)
            {
                /*
               for some reason the value returned by reading
               this /proc entry seems to be big endian,
               convert it.
            */
                rev = ntohl(tmp);
                rev &= 0xFFFFFF; /* mask out warranty bit */
            }
            fclose(fp);
        }
    }

    pi_ispi = 0;
    rev &= 0xFFFFFF; /* mask out warranty bit */

    /* Decode revision code */

    if ((rev & 0x800000) == 0) /* old rev code */
    {
        if ((rev > 0) && (rev < 0x0016)) /* all BCM2835 */
        {
            pi_ispi = 1;
            pi_peri_phys = 0x20000000;
        }
        else
        {
            eprintf("unknown revision=%x", rev);
            rev = 0;
        }
    }
    else /* new rev code */
    {
        switch ((rev >> 12) & 0xF) /* just interested in BCM model */
        {

        case 0x0: /* BCM2835 */
            pi_ispi = 1;
            pi_peri_phys = 0x20000000;
            break;

        case 0x1: /* BCM2836 */
        case 0x2: /* BCM2837 */
            pi_ispi = 1;
            pi_peri_phys = 0x3F000000;
            break;

        case 0x3: /* BCM2711 */
            pi_ispi = 1;
            pi_peri_phys = 0xFE000000;
            pi_is_2711 = 1;
            break;

        default:
            rev = 0;
            pi_ispi = 0;
            eprintf("unknown revision %x", rev);
            break;
        }
    }

    return rev;
}

#define GPPUD 37
#define GPPUDCLK0 38

/* BCM2711 has different pulls */

#define GPPUPPDN0 57

static void myGpioSleep(int seconds, int micros)
{
    struct timespec ts, rem;

    ts.tv_sec = seconds;
    ts.tv_nsec = micros * 1000;

    while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
    {
        /* copy remaining time to ts */
        ts = rem;
    }
}

/* ----------------------------------------------------------------------- */

static uint32_t myGpioDelay(uint32_t micros)
{
    uint32_t start;

    start = syst_reg[SYST_CLO];
    /* Longest busy delay */

#define PI_MAX_BUSY_DELAY 100
#define MILLION 1000000
    if (micros <= PI_MAX_BUSY_DELAY)
    {
        while ((syst_reg[SYST_CLO] - start) <= micros)
            ;
    }
    else
    {
        myGpioSleep(micros / MILLION, micros % MILLION);
    }

    return (syst_reg[SYST_CLO] - start);
}

int gpioSetPullUpDown(int pin, unsigned pud)
{
#define BANK (gpio >> 5)
#define BIT (1 << (gpio & 0x1F))
    int retval = -1;
    if (!gpio_initd)
    {
        gpioInitialize();
        if (gpioSetMode(pin, GPIO_IN) < 0)
        {
            eprintf("Error setting pin mode to input");
            goto ret;
        }
    }
    if (pi_ispi && pi_pud_avail)
    {
        if (gpio_props_dev.mode[pin] == GPIO_OUT) // can not be set on output pin
        {
            eprintf("Pin is output, can not set pull up/down mode");
            goto ret;
        }
        unsigned gpio = gpio_lut_pins[pin];
        gpio_props_dev.pud[pin] = pud; // store pull up enable mode
        int shift = (gpio & 0xf) << 1;
        uint32_t bits;
        uint32_t pull;

        if (pi_is_2711)
        {
            switch (pud)
            {
            case GPIO_PUD_OFF:
                pull = 0;
                break;
            case GPIO_PUD_UP:
                pull = 1;
                break;
            case GPIO_PUD_DOWN:
                pull = 2;
                break;
            default:
                eprintf("%d not a valid pullup/down state", pud);
                retval = -1;
                goto ret;
                break;
            }

            bits = *(gpio_reg + GPPUPPDN0 + (gpio >> 4));
            bits &= ~(3 << shift);
            bits |= (pull << shift);
            *(gpio_reg + GPPUPPDN0 + (gpio >> 4)) = bits;
        }
        else
        {
            *(gpio_reg + GPPUD) = pud;

            if (pi_syst_avail)
                myGpioDelay(1);
            else
                usleep(10); // 10 microseconds

            *(gpio_reg + GPPUDCLK0 + BANK) = BIT;
            if (pi_syst_avail)
                myGpioDelay(1);
            else
                usleep(10); // 10 microseconds

            *(gpio_reg + GPPUD) = 0;

            *(gpio_reg + GPPUDCLK0 + BANK) = 0;
        }
        retval = 1;
    }
ret:
    return retval;
}