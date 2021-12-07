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

#if (GPIODEV_PINOUT == PINOUT_RPI)

#define PI_MIN_GPIO 0
#define PI_MAX_GPIO 53

/* user_gpio: 0-31 */

#define PI_MAX_USER_GPIO 31

#define PI_CLEAR 0
#define PI_SET 1

#define PI_OFF 0
#define PI_ON 1

#define PI_INPUT 0
#define PI_OUTPUT 1
#define PI_ALT0 4
#define PI_ALT1 5
#define PI_ALT2 6
#define PI_ALT3 7
#define PI_ALT4 3
#define PI_ALT5 2

#define PI_CLOCK_PWM 0
#define PI_CLOCK_PCM 1

/* dutycycle: 0-range */

#define PI_DEFAULT_DUTYCYCLE_RANGE 255

/* range: 25-40000 */

#define PI_MIN_DUTYCYCLE_RANGE 25
#define PI_MAX_DUTYCYCLE_RANGE 40000

/* pulsewidth: 0, 500-2500 */

#define PI_SERVO_OFF 0
#define PI_MIN_SERVO_PULSEWIDTH 500
#define PI_MAX_SERVO_PULSEWIDTH 2500

/* hardware PWM */

#define PI_HW_PWM_MIN_FREQ 1
#define PI_HW_PWM_MAX_FREQ 125000000
#define PI_HW_PWM_MAX_FREQ_2711 187500000
#define PI_HW_PWM_RANGE 1000000

#define PWM_FREQS 18

#define PI_WAVE_BLOCKS 4

#define CYCLES_PER_BLOCK 80
#define PULSE_PER_CYCLE 25

static int pwmFreq[PWM_FREQS];

#define SUPERCYCLE 800
#define SUPERLEVEL 20000

#define PAGES_PER_BLOCK 53

#define CBS_PER_IPAGE 117
#define LVS_PER_IPAGE 38
#define OFF_PER_IPAGE 38
#define TCK_PER_IPAGE 2
#define ON_PER_IPAGE 2
#define PAD_PER_IPAGE 7

typedef struct
{
    uint8_t is;
    uint8_t pad;
    uint16_t width;
    uint16_t range; /* dutycycles specified by 0 .. range */
    uint16_t freqIdx;
    uint16_t deferOff;
    uint16_t deferRng;
} gpioInfo_t;

static gpioInfo_t gpioInfo[PI_MAX_GPIO + 1];

typedef struct
{ /* linux/arch/arm/mach-bcm2708/include/mach/dma.h */
    uint32_t info;
    uint32_t src;
    uint32_t dst;
    uint32_t length;
    uint32_t stride;
    uint32_t next;
    uint32_t pad[2];
} rawCbs_t;

typedef struct
{
    rawCbs_t cb[128];
} dmaPage_t;

static dmaPage_t **dmaVirt = MAP_FAILED;

typedef struct
{
    rawCbs_t cb[CBS_PER_IPAGE];
    uint32_t level[LVS_PER_IPAGE];
    uint32_t gpioOff[OFF_PER_IPAGE];
    uint32_t tick[TCK_PER_IPAGE];
    uint32_t gpioOn[ON_PER_IPAGE];
    uint32_t periphData;
    uint32_t pad[PAD_PER_IPAGE];
} dmaIPage_t;
static dmaIPage_t **dmaIVirt = MAP_FAILED;
static volatile uint32_t *pwmReg = MAP_FAILED;
static bool pi_pwmreg_avail = false;
#define PWM_BASE (pi_peri_phys + 0x0020C000)
#define PWM_LEN 0x28

static bool pwm_reg_avail = false;

#define GPIO_UNDEFINED 0
#define GPIO_WRITE 1
#define GPIO_PWM 2
#define GPIO_SERVO 3
#define GPIO_HW_CLK 4
#define GPIO_HW_PWM 5
#define GPIO_SPI 6
#define GPIO_I2C 7

#define GPSET0 7
#define GPSET1 8

#define GPCLR0 10
#define GPCLR1 11

typedef struct
{
    unsigned bufferMilliseconds;
    unsigned clockMicros;
    unsigned clockPeriph;
    unsigned DMAprimaryChannel;
    unsigned DMAsecondaryChannel;
    unsigned socketPort;
    unsigned ifFlags;
    unsigned memAllocMode;
    unsigned dbgLevel;
    unsigned alertFreq;
    uint32_t internals;
    /*
      0-3: dbgLevel
      4-7: alertFreq
      */
} gpioCfg_t;

#define PI_DEFAULT_BUFFER_MILLIS 120
#define PI_DEFAULT_CLK_MICROS 5
#define PI_DEFAULT_CLK_PERIPHERAL PI_CLOCK_PCM
#define PI_DEFAULT_IF_FLAGS 0
#define PI_DEFAULT_FOREGROUND 0
#define PI_DEFAULT_DMA_CHANNEL 14
#define PI_DEFAULT_DMA_PRIMARY_CHANNEL 14
#define PI_DEFAULT_DMA_SECONDARY_CHANNEL 6
#define PI_DEFAULT_DMA_PRIMARY_CH_2711 7
#define PI_DEFAULT_DMA_SECONDARY_CH_2711 6
#define PI_DEFAULT_DMA_NOT_SET 15
#define PI_DEFAULT_SOCKET_PORT 8888
#define PI_DEFAULT_SOCKET_PORT_STR "8888"
#define PI_DEFAULT_SOCKET_ADDR_STR "localhost"
#define PI_DEFAULT_UPDATE_MASK_UNKNOWN 0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_B1 0x03E7CF93
#define PI_DEFAULT_UPDATE_MASK_A_B2 0xFBC7CF9C
#define PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS 0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_ZERO 0x0080000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI2B 0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI3B 0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI4B 0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_COMPUTE 0x00FFFFFFFFFFFFLL

static volatile gpioCfg_t gpioCfg =
    {
        PI_DEFAULT_BUFFER_MILLIS,
        PI_DEFAULT_CLK_MICROS,
        PI_DEFAULT_CLK_PERIPHERAL,
        PI_DEFAULT_DMA_NOT_SET, /* primary DMA */
        PI_DEFAULT_DMA_NOT_SET, /* secondary DMA */
        PI_DEFAULT_SOCKET_PORT,
        PI_DEFAULT_IF_FLAGS,
        0, /* PI_MEM_ALLOC_AUTO */
        0, /* dbgLevel */
        0, /* alertFreq */
        0, /* internals */
};

static unsigned bufferBlocks; /* number of blocks in buffer */
static unsigned bufferCycles; /* number of cycles */

static bool pi_dmavirt_avail = false;

static bool pi_pwm_avail = false;

#define PWM_FREQS 18

typedef struct
{
    uint16_t valid;
    uint16_t servoIdx;
} clkCfg_t;
static const clkCfg_t clkCfg[] =
    {
        /* valid servo */
        {0, 0},  /*  0 */
        {1, 17}, /*  1 */
        {1, 16}, /*  2 */
        {0, 0},  /*  3 */
        {1, 15}, /*  4 */
        {1, 14}, /*  5 */
        {0, 0},  /*  6 */
        {0, 0},  /*  7 */
        {1, 13}, /*  8 */
        {0, 0},  /*  9 */
        {1, 12}, /* 10 */
};

static const uint16_t pwmRealRange[PWM_FREQS] =
    {25, 50, 100, 125, 200, 250, 400, 500, 625,
     800, 1000, 1250, 2000, 2500, 4000, 5000, 10000, 20000};

static const uint16_t pwmCycles[PWM_FREQS] =
    {1, 2, 4, 5, 8, 10, 16, 20, 25,
     32, 40, 50, 80, 100, 160, 200, 400, 800};

static void initPWM(unsigned bits);

#endif

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
    ((void)rev);
#endif
    if (pi_ispi) // if it is a pi
    {
#if (GPIODEV_PINOUT == PINOUT_RPI)
        int servoCycles, superCycles;

        servoCycles = gpioCfg.bufferMilliseconds / 20;
        if (gpioCfg.bufferMilliseconds % 20)
            servoCycles++;

        bufferCycles = (SUPERCYCLE * servoCycles) / gpioCfg.clockMicros;

        superCycles = bufferCycles / SUPERCYCLE;
        if (bufferCycles % SUPERCYCLE)
            superCycles++;

        bufferCycles = SUPERCYCLE * superCycles;

        bufferBlocks = bufferCycles / CYCLES_PER_BLOCK;
#endif

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
#if (GPIODEV_PINOUT == PINOUT_RPI)
            dmaVirt = mmap(
                0, PAGES_PER_BLOCK * (bufferBlocks + PI_WAVE_BLOCKS) * sizeof(dmaPage_t *),
                PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
                -1, 0);

            if (dmaVirt == MAP_FAILED)
            {
                eprintf("Failed to map DMA registers");
            }
            else
            {
                pi_dmavirt_avail = true;
            }
            pwmReg = map_mem(fdmem, PWM_BASE, PWM_LEN);

            if (pwmReg == MAP_FAILED)
            {
                eprintf("Failed to map PWM registers");
            }
            else
            {
                pi_pwmreg_avail = true;
            }
#endif
        }

#if (GPIODEV_PINOUT == PINOUT_RPI)
        dmaIVirt = (dmaIPage_t **)dmaVirt;
        for (int i = 0; i < PWM_FREQS; i++)
        {
            pwmFreq[i] =
                (1000000.0 /
                 ((float)PULSE_PER_CYCLE * gpioCfg.clockMicros * pwmCycles[i])) +
                0.5;
        }

        if (pi_pwmreg_avail && pi_dmavirt_avail)
        {
            pi_pwm_avail = true;
            initPWM(10);
        }
#endif
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
#if (GPIODEV_PINOUT == PINOUT_RPI)
            if (pi_dmavirt_avail)
            {
                munmap(dmaVirt, PAGES_PER_BLOCK * (bufferBlocks + PI_WAVE_BLOCKS) * sizeof(dmaPage_t *));
            }
            if (pi_pwmreg_avail)
            {
                munmap((void *)pwmReg, PWM_LEN);
            }
#endif
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

#if (GPIODEV_PINOUT == PINOUT_RPI)
/* ----------------------------------------------------------------------- */

static void myOffPageSlot(int pos, int *page, int *slot)
{
    *page = pos / OFF_PER_IPAGE;
    *slot = pos % OFF_PER_IPAGE;
}

static int myGpioRead(unsigned gpio)
{
#define GPLEV0 13
    if ((*(gpio_reg + GPLEV0 + BANK) & BIT) != 0)
        return PI_ON;
    else
        return PI_OFF;
}

static void myGpioWrite(unsigned gpio, unsigned level)
{
    if (level == PI_OFF)
        *(gpio_reg + GPCLR0 + BANK) = BIT;
    else
        *(gpio_reg + GPSET0 + BANK) = BIT;
}

static void myGpioSetMode(unsigned gpio, unsigned mode)
{
    int reg, shift;

    reg = gpio / 10;
    shift = (gpio % 10) * 3;

    gpio_reg[reg] = (gpio_reg[reg] & ~(7 << shift)) | (mode << shift);
}

static void mySetGpioOff(unsigned gpio, int pos)
{
    int page, slot;

    myOffPageSlot(pos, &page, &slot);

    dmaIVirt[page]->gpioOff[slot] |= (1 << gpio);
}

/* ----------------------------------------------------------------------- */

static void myClearGpioOff(unsigned gpio, int pos)
{
    int page, slot;

    myOffPageSlot(pos, &page, &slot);

    dmaIVirt[page]->gpioOff[slot] &= ~(1 << gpio);
}

/* ----------------------------------------------------------------------- */

static void mySetGpioOn(unsigned gpio, int pos)
{
    int page, slot;

    page = pos / ON_PER_IPAGE;
    slot = pos % ON_PER_IPAGE;

    dmaIVirt[page]->gpioOn[slot] |= (1 << gpio);
}

/* ----------------------------------------------------------------------- */

static void myClearGpioOn(unsigned gpio, int pos)
{
    int page, slot;

    page = pos / ON_PER_IPAGE;
    slot = pos % ON_PER_IPAGE;

    dmaIVirt[page]->gpioOn[slot] &= ~(1 << gpio);
}

static void myGpioSetPwm(unsigned gpio, int oldVal, int newVal)
{
    int switchGpioOff;
    int newOff, oldOff, realRange, cycles, i;
    int deferOff, deferRng;

    switchGpioOff = 0;

    realRange = pwmRealRange[gpioInfo[gpio].freqIdx];

    cycles = pwmCycles[gpioInfo[gpio].freqIdx];

    newOff = (newVal * realRange) / gpioInfo[gpio].range;
    oldOff = (oldVal * realRange) / gpioInfo[gpio].range;

    deferOff = gpioInfo[gpio].deferOff;
    deferRng = gpioInfo[gpio].deferRng;

    if (gpioInfo[gpio].deferOff)
    {
        for (i = 0; i < SUPERLEVEL; i += deferRng)
        {
            myClearGpioOff(gpio, i + deferOff);
        }
        gpioInfo[gpio].deferOff = 0;
    }

    if (newOff != oldOff)
    {
        if (newOff && oldOff) /* PWM CHANGE */
        {
            if (newOff != realRange)
            {
                for (i = 0; i < SUPERLEVEL; i += realRange)
                    mySetGpioOff(gpio, i + newOff);
            }

            if (newOff > oldOff)
            {
                for (i = 0; i < SUPERLEVEL; i += realRange)
                    myClearGpioOff(gpio, i + oldOff);
            }
            else
            {
                gpioInfo[gpio].deferOff = oldOff;
                gpioInfo[gpio].deferRng = realRange;
            }
        }
        else if (newOff) /* PWM START */
        {
            if (newOff != realRange)
            {
                for (i = 0; i < SUPERLEVEL; i += realRange)
                    mySetGpioOff(gpio, i + newOff);
            }

            /* schedule new gpio on */

            for (i = 0; i < SUPERCYCLE; i += cycles)
                mySetGpioOn(gpio, i);
        }
        else /* PWM STOP */
        {
            /* deschedule gpio on */

            for (i = 0; i < SUPERCYCLE; i += cycles)
                myClearGpioOn(gpio, i);

            for (i = 0; i < SUPERLEVEL; i += realRange)
                myClearGpioOff(gpio, i + oldOff);

            switchGpioOff = 1;
        }

        if (switchGpioOff)
        {
            *(gpio_reg + GPCLR0) = (1 << gpio);
            *(gpio_reg + GPCLR0) = (1 << gpio);
        }
    }
}

#define PWM_CTL 0
#define PWM_STA 1
#define PWM_DMAC 2
#define PWM_RNG1 4
#define PWM_DAT1 5
#define PWM_FIFO 6
#define PWM_RNG2 8
#define PWM_DAT2 9

#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_PWEN2 (1 << 8)
#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

#define PWM_DMAC_ENAB (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ(x) (x)

static void initPWM(unsigned bits)
{
    /* reset PWM */

    pwmReg[PWM_CTL] = 0;

    myGpioDelay(10);

    pwmReg[PWM_STA] = -1;

    myGpioDelay(10);

    /* set number of bits to transmit */

    pwmReg[PWM_RNG1] = bits;

    myGpioDelay(10);

    dmaIVirt[0]->periphData = 1;

    /* enable PWM DMA, raise panic and dreq thresholds to 15 */

    pwmReg[PWM_DMAC] = PWM_DMAC_ENAB |
                       PWM_DMAC_PANIC(15) |
                       PWM_DMAC_DREQ(15);

    myGpioDelay(10);

    /* clear PWM fifo */

    pwmReg[PWM_CTL] = PWM_CTL_CLRF1;

    myGpioDelay(10);

    /* enable PWM channel 1 and use fifo */

    pwmReg[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

static void myGpioSetServo(unsigned gpio, int oldVal, int newVal)
{
    int newOff, oldOff, realRange, cycles, i;
    int deferOff, deferRng;

    realRange = pwmRealRange[clkCfg[gpioCfg.clockMicros].servoIdx];
    cycles = pwmCycles[clkCfg[gpioCfg.clockMicros].servoIdx];

    newOff = (newVal * realRange) / 20000;
    oldOff = (oldVal * realRange) / 20000;

    deferOff = gpioInfo[gpio].deferOff;
    deferRng = gpioInfo[gpio].deferRng;

    if (gpioInfo[gpio].deferOff)
    {
        for (i = 0; i < SUPERLEVEL; i += deferRng)
        {
            myClearGpioOff(gpio, i + deferOff);
        }
        gpioInfo[gpio].deferOff = 0;
    }

    if (newOff != oldOff)
    {
        if (newOff && oldOff) /* SERVO CHANGE */
        {
            for (i = 0; i < SUPERLEVEL; i += realRange)
                mySetGpioOff(gpio, i + newOff);

            if (newOff > oldOff)
            {
                for (i = 0; i < SUPERLEVEL; i += realRange)
                    myClearGpioOff(gpio, i + oldOff);
            }
            else
            {
                gpioInfo[gpio].deferOff = oldOff;
                gpioInfo[gpio].deferRng = realRange;
            }
        }
        else if (newOff) /* SERVO START */
        {
            for (i = 0; i < SUPERLEVEL; i += realRange)
                mySetGpioOff(gpio, i + newOff);

            /* schedule new gpio on */

            for (i = 0; i < SUPERCYCLE; i += cycles)
                mySetGpioOn(gpio, i);
        }
        else /* SERVO STOP */
        {
            /* deschedule gpio on */

            for (i = 0; i < SUPERCYCLE; i += cycles)
                myClearGpioOn(gpio, i);

            /* if in pulse then delay for the last cycle to complete */

            if (myGpioRead(gpio))
                myGpioDelay(PI_MAX_SERVO_PULSEWIDTH);

            /* deschedule gpio off */

            for (i = 0; i < SUPERLEVEL; i += realRange)
                myClearGpioOff(gpio, i + oldOff);
        }
    }
}

static void switchFunctionOff(unsigned gpio)
{
    switch (gpioInfo[gpio].is)
    {
    case GPIO_SERVO:
        /* switch servo off */
        myGpioSetServo(gpio, gpioInfo[gpio].width, 0);
        gpioInfo[gpio].width = 0;
        break;

    case GPIO_PWM:
        /* switch pwm off */
        myGpioSetPwm(gpio, gpioInfo[gpio].width, 0);
        gpioInfo[gpio].width = 0;
        break;

    case GPIO_HW_CLK:
        /* No longer disable clock hardware, doing that was a bug. */
        gpioInfo[gpio].width = 0;
        break;

    case GPIO_HW_PWM:
        /* No longer disable PWM hardware, doing that was a bug. */
        gpioInfo[gpio].width = 0;
        break;
    }
}

int gpioPWM(unsigned pin, unsigned val)
{
    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    if (val > gpioInfo[gpio].range)
    {
        eprintf("gpio %d, bad dutycycle (%d)", gpio, val);
        return -2;
    }

    if (gpioInfo[gpio].is != GPIO_PWM)
    {
        switchFunctionOff(gpio);

        gpioInfo[gpio].is = GPIO_PWM;

        if (!val)
            myGpioWrite(gpio, 0);
    }

    myGpioSetMode(gpio, GPIO_OUT);

    myGpioSetPwm(gpio, gpioInfo[gpio].width, val);

    gpioInfo[gpio].width = val;

    return 0;
}

/* ----------------------------------------------------------------------- */

int gpioGetPWMdutycycle(unsigned pin)
{
    unsigned pwm;

    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    switch (gpioInfo[gpio].is)
    {
    case GPIO_PWM:
        return gpioInfo[gpio].width;

    case GPIO_HW_PWM:
        return PI_HW_PWM_RANGE / 2;

    case GPIO_HW_CLK:
        return PI_HW_PWM_RANGE / 2;

    default:
        eprintf("not a PWM gpio (%d)", pin);
    }
}

/* ----------------------------------------------------------------------- */

int gpioSetPWMrange(unsigned pin, unsigned range)
{
    int oldWidth, newWidth;

    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    oldWidth = gpioInfo[gpio].width;

    if (oldWidth)
    {
        if (gpioInfo[gpio].is == GPIO_PWM)
        {
            newWidth = (range * oldWidth) / gpioInfo[gpio].range;

            myGpioSetPwm(gpio, oldWidth, 0);
            gpioInfo[gpio].range = range;
            gpioInfo[gpio].width = newWidth;
            myGpioSetPwm(gpio, 0, newWidth);
        }
    }

    gpioInfo[gpio].range = range;

    /* return the actual range for the current gpio frequency */

    return pwmRealRange[gpioInfo[gpio].freqIdx];
}

/* ----------------------------------------------------------------------- */

int gpioGetPWMrange(unsigned pin)
{
    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    switch (gpioInfo[gpio].is)
    {
    case GPIO_HW_PWM:
    case GPIO_HW_CLK:
        return PI_HW_PWM_RANGE;

    default:
        return gpioInfo[gpio].range;
    }
}

/* ----------------------------------------------------------------------- */

int gpioGetPWMrealRange(unsigned pin)
{
    unsigned pwm;

    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    switch (gpioInfo[gpio].is)
    {
    case GPIO_HW_PWM:
    case GPIO_HW_CLK:
        return PI_HW_PWM_RANGE;

    default:
        return pwmRealRange[gpioInfo[gpio].freqIdx];
    }
}

/* ----------------------------------------------------------------------- */

int gpioSetPWMfrequency(unsigned pin, unsigned frequency)
{
    int i, width;
    unsigned diff, best, idx;

    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    if (frequency > pwmFreq[0])
        idx = 0;
    else if (frequency < pwmFreq[PWM_FREQS - 1])
        idx = PWM_FREQS - 1;
    else
    {
        best = 100000; /* impossibly high frequency difference */
        idx = 0;

        for (i = 0; i < PWM_FREQS; i++)
        {
            if (frequency > pwmFreq[i])
                diff = frequency - pwmFreq[i];
            else
                diff = pwmFreq[i] - frequency;

            if (diff < best)
            {
                best = diff;
                idx = i;
            }
        }
    }

    width = gpioInfo[gpio].width;

    if (width)
    {
        if (gpioInfo[gpio].is == GPIO_PWM)
        {
            myGpioSetPwm(gpio, width, 0);
            gpioInfo[gpio].freqIdx = idx;
            myGpioSetPwm(gpio, 0, width);
        }
    }

    gpioInfo[gpio].freqIdx = idx;

    return pwmFreq[idx];
}

/* ----------------------------------------------------------------------- */

int gpioGetPWMfrequency(unsigned pin)
{
    unsigned pwm, clock;

    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    switch (gpioInfo[gpio].is)
    {
    default:
        return pwmFreq[gpioInfo[gpio].freqIdx];
    }
}

/* ----------------------------------------------------------------------- */

int gpioServo(unsigned pin, unsigned val)
{
    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    if ((val != PI_SERVO_OFF) && (val < PI_MIN_SERVO_PULSEWIDTH))
        eprintf("gpio %d, bad pulsewidth (%d)", gpio, val);

    if (val > PI_MAX_SERVO_PULSEWIDTH)
        eprintf("gpio %d, bad pulsewidth (%d)", gpio, val);

    if (gpioInfo[gpio].is != GPIO_SERVO)
    {
        switchFunctionOff(gpio);

        gpioInfo[gpio].is = GPIO_SERVO;

        if (!val)
            myGpioWrite(gpio, 0);
    }

    myGpioSetMode(gpio, PI_OUTPUT);

    myGpioSetServo(gpio, gpioInfo[gpio].width, val);

    gpioInfo[gpio].width = val;

    return 0;
}

/* ----------------------------------------------------------------------- */

int gpioGetServoPulsewidth(unsigned pin)
{
    if (!pi_ispi)
    {
        eprintf("Function not available on non-Raspberry Pi devices");
        return -1;
    }
    if (!pi_pwm_avail)
    {
        return -1;
    }

    int gpio = gpio_lut_pins[pin];

    if (gpioInfo[gpio].is != GPIO_SERVO)
        eprintf("not a servo gpio (%d)", gpio);

    return gpioInfo[gpio].width;
}

#endif // if (GPIODEV_PINOUT == PINOUT_RPI)