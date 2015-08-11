# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-g -c -std=c++0x

INCLUDEFLAGS=-I/usr/local/include/fcl/

override INCLUDEFLAGS += -I/usr/include/eigen3 -I/usr/local/include/openrave-0.9 -I/usr/include/boost -I/usr/include/jsoncpp/
override INCLUDEFLAGS += -L/usr/lib/ -L/usr/local/lib/

LDFLAGS= -lfcl
BOOSTFLAGS= -lboost_system -lboost_program_options


all: fcl_test clean

fcl_test: Kinematics.o fcl_test.o
	$(CC) $(INCLUDEFLAGS) Kinematics.o fcl_test.o -o fcl_test $(LDFLAGS) $(BOOSTFLAGS)

fcl_test.o: fcl_test.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) fcl_test.cpp -o fcl_test.o
	
Kinematics.o: Kinematics.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) Kinematics.cpp -o Kinematics.o
	
clean:
	rm *.o
