
OBJ=main.o misc_utils.o dyn_model.o controller.o switch.o

CFLAGS=-I/opt/local/include -Wall
CPPFLAGS=-I/opt/local/include -Wall -I/Users/esdentem/QtSDK/Desktop/Qt/4.8.1/gcc/include -I/Users/esdentem/QtSDK/Desktop/Qt/4.8.1/gcc/lib/QtCore.framework/Versions/Current/Headers/
LDFLAGS=-L/opt/local/lib -lgsl -F/Users/esdentem/QtSDK/Desktop/Qt/4.8.1/gcc/lib -framework QtCore

all: obldc-csim

obldc-csim: $(OBJ)
	$(CXX) $(LDFLAGS) $(OBJ) -o $@

clean:
	rm -rf *.o obldc-csim
