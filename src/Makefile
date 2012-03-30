
OBJ=main.o misc_utils.o dyn_model.o

CFLAGS=-I/opt/local/include -Wall
LDFLAGS=-L/opt/local/lib -lgsl

all: obldc-csim

obldc-csim: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

clean:
	rm -rf *.o obldc-csim
