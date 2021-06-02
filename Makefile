CC=gcc
EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDLDFLAGS:= -lpthread $(LDFLAGS)

COBJ=gpiodev.o

HOSTNAME=$(shell hostname)

ifeq ($(HOSTNAME), spacehauc)
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9361
endif
ifeq ($(HOSTNAME), ad9361)
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9361
endif
ifeq ($(HOSTNAME), fl9361)
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9361
endif
ifeq ($(HOSTNAME), ad9364)
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9364
endif
ifeq ($(HOSTNAME), adrv9364)
	EDCFLAGS += -DGPIODEV_PINOUT=PINOUT_AD9364
endif

all: gpiotest irqtest waittest pulluptest
	echo "Targets gpiotest.out, irqtest.out, waittest.out and pulluptest.out finished building for device $(HOSTNAME)"

gpiotest: gpiotest.o $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $< $(COBJ) $(EDLDFLAGS)

irqtest: irqtest.o $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $< $(COBJ) $(EDLDFLAGS)

waittest: waittest.o $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $< $(COBJ) $(EDLDFLAGS)

pulluptest: pulluptest.o $(COBJ)
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