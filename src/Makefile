
OBJ=main.o misc_utils.o dyn_model.o controller.o

CFLAGS=-I/opt/local/include -Wall
CPPFLAGS=-I/opt/local/include -Wall -I/Users/esdentem/QtSDK/Desktop/Qt/4.8.0/gcc/include -I/Users/esdentem/QtSDK/Desktop/Qt/4.8.0/gcc/lib/QtCore.framework/Versions/4/Headers/
LDFLAGS=-L/opt/local/lib -lgsl -F/Users/esdentem/QtSDK/Desktop/Qt/4.8.0/gcc/lib -framework QtCore

all: obldc-csim

obldc-csim: $(OBJ)
	$(CXX) $(LDFLAGS) $(OBJ) -o $@

clean:
	rm -rf *.o obldc-csim
