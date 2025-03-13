CC=gcc
CFLAGS=-Wall
LDFLAGS=-lm -lpigpiod_if2
EXEC=a.out
SRC= $(wildcard *.c)
HEAD= $(wildcard *.h)
OBJ= $(SRC:.c=.o)

all: $(EXEC)

a.out: $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

main.o: $(HEAD)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean mrproper

clean:
	rm -rf *.o

mrproper: clean
	rm -rf $(EXEC)