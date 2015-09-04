ifeq ($(shell uname), Linux)
   LINUX_BT = 1 # remove or comment this line if compiling in Linux without BlueZ development files
endif

ifdef LINUX_BT
   CFLAGS = -DHASBLUETOOTH
   LIBS = -lbluetooth
endif
CXX = g++-4.7

all: test

test: bitalino.o test.o
	$(CXX) $^ $(LIBS) -lrt -o $@

bitalino.o: bitalino.cpp
	$(CXX) $(CFLAGS) -lrt -c $< -o $@

test.o: test.cpp
	$(CXX) -std=c++11 -lrt -c $< -o $@
