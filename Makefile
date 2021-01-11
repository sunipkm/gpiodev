CC=gcc
TARGET=gpio_tester.out

all: $(TARGET)
	sudo ./$(TARGET)

$(TARGET):
	$(CC) -std=c11 -O2 -DUNIT_TEST gpiodev.c -o $(TARGET)


.PHONY: clean

clean:
	rm -vrf $(TARGET)
