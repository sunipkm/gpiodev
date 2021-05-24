CC=gcc
EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDLDFLAGS:= -lpthread $(LDFLAGS)
SHELL=/bin/bash

COBJ=gpiodev.o

BRANCH=$(shell git branch --show-current)

ifeq ($(BRANCH), "master")
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9361
endif
ifeq ($(BRANCH), "ad9364")
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9364
endif

all: gpiotest irqtest
	echo "Targets gpiotest.out and irqtest.out finished building for branch $(BRANCH)"

gpiotest: gpiotest.o $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $< $(COBJ) $(EDLDFLAGS)

irqtest: irqtest.o $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $< $(COBJ) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<


.PHONY: doc

doc:
	doxygen .doxyconfig

.PHONY: clean

clean:
	rm -vf *.out
	rm -vf *.o

spotless: clean
	rm -vrf doc