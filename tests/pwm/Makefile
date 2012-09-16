
OBJ=main.o

CFLAGS=-I/opt/local/include -g
LDFLAGS=-L/opt/local/lib -lgsl

all: pwm

pwm: $(OBJ)
	$(CC) $(LDFLAGS) $< -o $@

clean:
	rm -rf *.o
	rm -rf pwm
