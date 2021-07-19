
/**
 * @file gpiodev.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Header file containing function prototypes and properties of
 * the GPIO sysfs access module.
 * @version 1.0
 * @date 2020-08-30
 * 
 * @copyright Copyright (c) 2020
 * 
 * @license GPL v3
 */

#ifndef _GPIODEV_H
#define _GPIODEV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>

#ifndef DOXYGEN
#define PINOUT_AD9361 0 //!< Activate GPIO LUT for AD9361 (SPACE HAUC)
#define PINOUT_AD9364 1 //!< Activate GPIO LUT for AD9364 (Test)
#endif
#define PINOUT_RPI 2    //!< Activate GPIO LUT for Raspberry Pi (Default)

#ifndef GPIODEV_PINOUT
/**
 * @brief GPIODEV_PINOUT parameter defines the pin LUT used. The values can be 0 (PINOUT_AD9361), 1 (PINOUT_AD9364), 2 (PINOUT_RPI)
 * 
 */
#define GPIODEV_PINOUT PINOUT_RPI
#endif

#ifndef GPIODEV_SINGLE_INSTANCE
/**
 * @brief GPIODEV_SINGLE_INSTANCE = 0 allows any number of programs using gpiodev library to operate simultaneously.
 * Set this value > 0 to allow PID locking of the gpiodev interface.
 * 
 */
#define GPIODEV_SINGLE_INSTANCE 0
#else
#undef GPIODEV_SINGLE_INSTANCE
#define GPIODEV_SINGLE_INSTANCE 1
#warning "gpiodev: GPIODEV_SINGLE_INSTANCE specified. Only one program will be allowed to call gpioInitialize()"
#endif

static int gpiodev_pinout __attribute__((used)) = GPIODEV_PINOUT; //!< Runtime variable to determine pin layout program is compiled with

#define GPIO_LOW 0  //!< Low voltage on GPIO
#define GPIO_HIGH 1 //!< High voltage on GPIO

/**
 * @brief Enumerates the available GPIO Pin modes.
 * 
 */
enum GPIO_MODE
{
    GPIO_IN,       //!< Mode GPIO input
    GPIO_OUT,      //!< Mode GPIO output
    GPIO_IRQ_FALL, //!< Trigger IRQ on falling edge
    GPIO_IRQ_RISE, //!< Trigger IRQ on rising edge
    GPIO_IRQ_BOTH, //!< Trigger IRQ on both rising and falling edge
};

/**
 * @brief Enumerates available GPIO pull up states, only available on Raspberry Pi
 * 
 */
enum GPIO_PUD
{
    GPIO_PUD_OFF,  //!< Disable pull up/down
    GPIO_PUD_DOWN, //!< Set pin as pull down
    GPIO_PUD_UP,   //!< Set pin as pull up
};

#ifndef eprintf
/**
 * @brief Wrapper to print a string to stderr with printf like arg support. Prepends provided string with the function from which it is called
 * and line number by default, and appends a newline.
 * 
 */
#define eprintf(str, ...)                                                        \
    {                                                                            \
        fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr);                                                          \
    }
#endif
/**
 * @brief IRQ callback function of form void func(void *ptr). ptr points to the location of any input parameter/struct
 * 
 */
typedef void (*gpio_irq_callback_t)(void *);

#ifdef GPIODEV_INTERNAL
/**
 * @brief GPIO look up table for easy access. Currently mapped to AD9361 CRR
 * system. The indices map the ps7 GPIO pins to physical pins
 */
static int gpio_lut_pins[] =
#if (GPIODEV_PINOUT == PINOUT_AD9361)
    {
        -1,
        976, // TR_BF
        977, // TX_LD
        978, // RX_LD
        979, // PWR_TGL
        980, // 5V_CTRL
        981, // TR_UDC
        982, // CS_UDC
        983, // TS_CS_1
        984, // TS_CS_2
        985, // TS_CS_3
        986, // TS_CS_4
        987, // TS_CS_5
        988, // TS_CS_6
        989, // TS_CS_7
        990, // CAM_RST
        991, // PLL_LOCK
        960  // FIFO_RST
};

#elif (GPIODEV_PINOUT == PINOUT_AD9364)
    {
        960,
        961,
        962,
        963,
        964,
        965,
        966,
        967,
        968,
        969,
        970,
        971,
        972,
        973,
        974,
        975,
        976,
        977,
        978,
        979,
        980,
        981,
        982,
        983,
        984,
        985,
        986,
        987,
        988,
        989,
        990,
        991};
#elif (GPIODEV_PINOUT == PINOUT_RPI)
    {
        -1,
        -1, // Pin 1, 3V3
        -1, // Pin 2, 5V
        -1, // Pin 3, I2C1 SDA
        -1, // Pin 4, GND
        -1, // Pin 5, I2C1 SCL
        -1, // Pin 6, GND
        4,  // Pin 7, GPCLK0
        -1, // Pin 8, UART TX
        -1, // Pin 9, GND
        -1, // Pin 10, UART RX
        17, // Pin 11
        18, // Pin 12, PCM CLK
        27, // Pin 13
        -1, // Pin 14, GND
        22, // Pin 15
        23, // Pin 16
        -1, // Pin 17, 3V3 Power
        24, // Pin 18
        -1, // Pin 19, SPI0 MOSI
        -1, // Pin 20, GND
        -1, // Pin 21, SPI0 MISO
        25, // Pin 22
        -1, // Pin 23, SPI0 SCLK
        -1, // Pin 24, SPI0 CE0
        -1, // Pin 25, GND
        -1, // Pin 26, SPI0 CE1
        0,  // Pin 27, EEPROM SDA
        1,  // Pin 28, EEPROM SCL
        5,  // Pin 29
        -1, // Pin 30, GND
        6,  // Pin 31
        12, // Pin 32, PWM0
        13, // Pin 33, PWM1
        -1, // Pin 34, GND
        19, // Pin 35, PCM FS
        16, // Pin 36
        26, // Pin 37
        20, // Pin 38, PCM DIN
        -1, // Pin 39, GND
        21, // Pin 40, PCM DOUT
};
#endif
/**
 * @brief Initialize GPIO sysfs subsystem. Should be called before calling gpioSetMode(). Otherwise called by default on the first instance gpioSetMode() is run.
 */
int gpioInitialize(void);
/**
 * @brief Free allocated memory for GPIO pins etc. Should be called before exiting the program, set to be called automatically.
 */
void gpioDestroy(void);

/**
 * @brief Number of avaliable GPIO pins in the system.
 */
const int NUM_GPIO_PINS = sizeof(gpio_lut_pins) / sizeof(int);

/**
 * @brief Structure containing complete definition of the GPIO pin system, used
 * only by the init and destroy functions. Read and write functions use a subset
 * for cache purposes.
 */
typedef struct
{
    int *fd_val;     /// File descriptors containing the GPIO inout values
    uint8_t *mode;   /// I/O modes of the GPIO pin, can assume values GPIO_IN, GPIO_OUT and GPIO_INOUT
    uint8_t *val;    /// Last read/set value of the GPIO pin
    uint8_t *pud;    /// Status of pull up of the GPIO pin, by default no pull up
    int *fd_mode;    /// File descriptors to IO modes of the GPIO pins
    int fd_export;   /// File descriptor for GPIO export
    int fd_unexport; /// File descriptor for GPIO unexport
} gpioprops;

/** @brief Structure containing information for GPIO pin access.
 * Small footprint for cache optimizations. The pointers must be
 * populated at initialization.
 */
typedef struct
{
    int *fd;       /// File descriptions for accessing GPIO inout
    uint8_t *val;  /// Value of last read/set values
    uint8_t *mode; /// Mode of the GPIO pin
} gpiopins;

typedef struct
{
    int pin;                      // pin number
    gpio_irq_callback_t callback; // callback function
    void *userdata;               // callback user data
    int tout_ms;                  // poll timeout
} gpio_irq_params;

#endif // GPIODEV_INTERNAL

/**
 * @brief Set mode of GPIO Pins.
 * 
 * @param pin of type int, corresponds to the LUT index
 * @param mode of type enum GPIO_MODE
 * 
 * @returns Positive on success, negative on failure
 */
int gpioSetMode(int pin, enum GPIO_MODE mode);
/**
 * @brief Register GPIO Pin as interrupt
 * 
 * @param pin of type int, corresponds to the LUT index
 * @param mode of type enum GPIO_MODE
 * @param func Pointer to callback function of type void func(unsigned long long ptr)
 * @param userdata Pointer to userdata to be passed to callback function
 * @param tout_ms Timeout for poll in ms (-1 for indefinite)
 * @return int Positive on success, negative on error
 */
int gpioRegisterIRQ(int pin, enum GPIO_MODE mode, gpio_irq_callback_t func, void *userdata, int tout_ms);
/**
 * @brief Unregister IRQ handler on pin and set mode to standard input
 * 
 * @param pin Pin number
 * @return int Positive on success, negative on error, zero on no change
 */
int gpioUnregisterIRQ(int pin);
/**
 * @brief This function blocks until an interrupt is received on the GPIO pin,
 * or if the interrupt times out
 * 
 * @param pin Pin index in LUT
 * @param mode GPIO_IRQ_*
 * @param tout_ms IRQ poll timeout
 * @return int Positive on interrupt (number of interrupts received), 0 on timeout, negative on error
 */
int gpioWaitIRQ(int pin, enum GPIO_MODE mode, int tout_ms);
/**
 * @brief Write values GPIO_HIGH or GPIO_LOW to the GPIO pin indicated.
 * gpioRead or gpioWrite WILL NOT WORK if this function is not called
 * prior to GPIO use.
 * 
 * @param pin of type int, corresponds to the LUT index
 * @param val of type int, either GPIO_HIGH or GPIO_LOW
 * 
 * @returns Positive on success, negative on failure
 */
int gpioWrite(int pin, int val);

/**
 * @brief Read value of the GPIO pin indicated.
 * 
 * @param pin of type int, corresponds to the LUT index
 * 
 * @returns The state of the pin, or error if not GPIO_LOW or GPIO_HIGH
 */
int gpioRead(int pin);
/**
 * @brief Set pull-up on a pin (available only in Raspberry Pi)
 * 
 * @param pin Physical pin number
 * @param pud of type enum GPIO_PUD
 * @return int returns positive on success, negative on error
 */
int gpioSetPullUpDown(int pin, enum GPIO_PUD pud);

#ifdef __cplusplus
}
#endif

#endif // _GPIODEV_H
