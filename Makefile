# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-g -c -std=c++11

PYLIBPATH = $(shell python-config --exec-prefix)/lib

INCLUDEFLAGS=-I/usr/local/include/fcl/ -I/usr/include/python2.7/

override INCLUDEFLAGS += -I/usr/include/eigen3 -I/usr/local/include/openrave-0.9 -I/usr/include/boost -I/usr/include/jsoncpp/
override INCLUDEFLAGS += -L/usr/lib/ -L/usr/local/lib/

LDFLAGS= -L$(PYLIBPATH) $(shell python-config --libs) -lfcl
OPTS = $(shell python-config --include) -O2
BOOSTFLAGS= -lboost_system -lboost_program_options -lboost_python


all: kin.so util.so obstacle.so clean
	@python ./kin_test.py
	
fcl_test: Kinematics.o Terrain.o Obstacle.o Utils.o fcl_test.o
	$(CC) $(INCLUDEFLAGS) $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS)
	
util.so: Kinematics.o Utils.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS) 
	
kin.so: Kinematics.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $< -o $@ $(LDFLAGS) $(BOOSTFLAGS)
	
Kinematics.o: Kinematics.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
obstacle.so: Kinematics.o Utils.o Terrain.o Obstacle.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS)

fcl_test.o: fcl_test.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) $< -o $@
	
Obstacle.o: Obstacle.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
Terrain.o: Terrain.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
Utils.o: utils.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@

	
clean:
	rm *.o
