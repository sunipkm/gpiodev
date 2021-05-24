# GPIO Userspace Driver around the Sysfs backend
This module can be used to provide GPIO accessability in a C/C++ program, including interrupt handler support.
To overcome the issue of internal pin numbering that is different from header pin numbering (for example Raspberry Pi 
where pin 11 is internally called GPIO 17 etc), a look up table is defined in the header to allow for pin number 
translation from physical pins to internal representation.

For basic testing, two programs `gpiotest.out` and `irqtest.out` can be generated and run on a Raspberry Pi.

There is extensive documentation available through doxygen.