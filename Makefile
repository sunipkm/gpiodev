CC=gcc
EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDLDFLAGS:= -lpthread $(LDFLAGS)

COBJ=gpiodev.o

all: gpiotest irqtest
	echo "Targets gpiotest.out and irqtest.out finished building"

gpiotest: $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $@.c $(COBJ) $(EDLDFLAGS)

irqtest: $(COBJ)
	$(CC) $(EDCFLAGS) -o $@.out $@.c $(COBJ) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<


.PHONY: clean

clean:
	rm -vf *.out
	rm -vf *.o
