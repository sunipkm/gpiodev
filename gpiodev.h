
/**
 * @file gpiodev.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Header file containing function prototypes and properties of
 * the GPIO sysfs access module.
 * @version 0.1
 * @date 2020-08-30
 * 
 * @copyright Copyright (c) 2020
 * 
 * @license GPL v3
 */

#ifndef SH_GPIODEV_H
#define SH_GPIODEV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <pthread.h>

#define GPIO_LOW 0  /// Low voltage on GPIO
#define GPIO_HIGH 1 /// High voltage on GPIO
#define GPIO_IN 0
#define GPIO_OUT 1
#define GPIO_INOUT 2
#define GPIO_FNAME_MAX_LEN 256

#ifdef GPIODEV_INTERNAL
/**
 * @brief GPIO look up table for easy access. Currently mapped to AD9361 CRR
 * system. The indices map the ps7 GPIO pins to physical pins
 */
int __gpiodev_gpio_lut_pins[] = {
    -1,
    976, // TR_BF
    977, // TX_LD
    978, // RX_LD
    979, // PWR_TGL
    980, // 5V_CTRL
    981, // TR_UDC
    982, // PLL_LOCK
    983, // TS_CS_1
    984, // TS_CS_2
    985, // TS_CS_3
    986, // TS_CS_4
    987, // TS_CS_5
    988, // TS_CS_6
    989, // TS_CS_7
    990, // UDC_CS
    991, // CAM_RST
    960  // FIFO_RST
};

/**
 * @brief Number of avaliable GPIO pins in the system.
 */
const int NUM_GPIO_PINS = sizeof(__gpiodev_gpio_lut_pins) / sizeof(int);

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
#endif // GPIODEV_INTERNAL

/**
 * @brief Initialize GPIO sysfs subsystem. Must be called before calling gpioSetMode().
 */
int gpioInitialize(void);
/**
 * @brief Free allocated memory for GPIO pins etc.
 */
void gpioDestroy(void);

/**
 * @brief Set mode of GPIO Pins.
 * 
 * @param pin of type int, corresponds to the LUT index
 * @param mode of type int, either GPIO_IN or GPIO_OUT
 * 
 * @returns Positive on success, negative on failure
 */
int gpioSetMode(int pin, int mode);

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

#ifdef __cplusplus
}
#endif

#endif // SH_GPIODEV_H
