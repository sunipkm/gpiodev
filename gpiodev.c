#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#define GPIODEV_INTERNAL
#include "gpiodev.h"
#undef GPIODEV_INTERNAL
#include <macros.h>

gpioprops __gpiodev_props_dev; /// Memory allocation for the GPIO properties struct
gpiopins __gpiodev_pins_dev;   /// Memory allocation for the GPIO pins struct

int __gpiodev_gpio_initd = 0; /// Default: Uninitialized

int gpioInitialize(void)
{
	// memory allocations
	__gpiodev_props_dev.fd_val = (int *) malloc (NUM_GPIO_PINS * sizeof(int));
	__gpiodev_props_dev.fd_mode = (int *) malloc (NUM_GPIO_PINS * sizeof(int));
	__gpiodev_props_dev.val = (uint8_t *) malloc (NUM_GPIO_PINS * sizeof(uint8_t));
	__gpiodev_props_dev.mode = (uint8_t *) malloc (NUM_GPIO_PINS * sizeof(uint8_t));

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
		fprintf(stderr, RED "Failed to open export for writing.\n" CLR);
		return -1;
	}
	__gpiodev_props_dev.fd_export = fd;
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1)
	{
		fprintf(stderr, RED "Failed to open unexport for writing.\n" CLR);
		return -1;
	}
	__gpiodev_props_dev.fd_unexport = fd;
    __gpiodev_gpio_initd = 1;
	return 1;
}

static int GPIOExport(int pin)
{
#define BUFFER_MAX 3
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
	write(__gpiodev_props_dev.fd_unexport, buffer, bytes_written);
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
		fprintf(stderr, RED "GPIODEV: Error, pin %d is not available for GPIO operation.\n" CLR, pin);
		return -1;
	}

	if (mode != GPIO_IN && mode != GPIO_OUT)
	{
		fprintf(stderr, RED "GPIODEV: Error, mode is not GPIO_IN or GPIO_OUT for pin %d.\n" CLR, pin);
	}
	if (!GPIOExport(bcmpin)) // pin export successful
	{
		char path[256];
		int fd;

		if (__gpiodev_props_dev.fd_mode[pin] < 0) // pin mode uninitialized
		{
			snprintf(path, 256, "/sys/class/gpio/gpio%d/direction", bcmpin); // Direction fd
			fd = open(path, O_WRONLY);
			if (fd == -1)
			{
				fprintf(stderr, RED "Failed to open gpio direction for writing: %s\n" CLR, path);
				return (-1);
			}
			__gpiodev_props_dev.fd_mode[pin] = fd; // save the direction file descriptor
		}
		char modestr[] = "in\0out";
		if (write(fd, &modestr[mode == GPIO_IN ? 0 : 3], mode == GPIO_IN ? 2 : 3) == -1)
		{
			fprintf(stderr, "Failed to set direction for pin %d!\n", pin);
			return (-1);
		}

		// open file for value access
		if (__gpiodev_props_dev.fd_val[pin] < 0)
		{
			snprintf(path, 256, "/sys/class/gpio/gpio%d/value", bcmpin);
			fd = open(path, mode == GPIO_IN ? O_RDONLY : O_WRONLY); // Open as read/write depending on mode
			if (fd == -1)
			{
				fprintf(stderr, RED "Failed to open gpio value for read/write: %s\n" CLR, path);
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

	if (read(__gpiodev_pins_dev.fd[pin], value_str, 3) == -1)
	{
		fprintf(stderr, RED "Failed to read value from pin %d!\n" CLR, pin);
		return (-1);
	}
	return (atoi(value_str));
}

int gpioWrite(int pin, int value)
{
	static const char s_values_str[] = "01";

	if (1 != write(__gpiodev_pins_dev.fd[pin], &s_values_str[GPIO_LOW == value ? 0 : 1], 1))
	{
		fprintf(stderr, "Failed to write value!\n");
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
