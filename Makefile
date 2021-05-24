CC=gcc
EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDLDFLAGS:= -lpthread $(LDFLAGS)

COBJ=gpiodev.o

all: gpiotest irqtest
	echo "Targets gpiotest.out and irqtest.out finished building"

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