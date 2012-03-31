
OBJ=main.o

CFLAGS=-I/opt/local/include
LDFLAGS=-L/opt/local/lib -lgsl

all: oscillator

oscillator: $(OBJ)
	$(CC) $(LDFLAGS) $< -o $@
