CFLAGS = -I./build -Wall -I/usr/include/eigen3 -g -fPIC -std=c++11 
LDFLAGS = -lpthread -lCpl

OBJS = \
build/Configuration.o \
build/Contact.o \
build/Stance.o \
build/Vertex.o \
build/Tree.o \
build/Planner.o \
build/v_repExtMultiContactPlanner.o \
build/v_repLib.o \
        


OS = $(shell uname -s)
ECHO=@

ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	OPTION = -shared
	EXT = so
else
	CFLAGS += -D__APPLE__
	OPTION = -dynamiclib -current_version 1.0
	EXT = dylib
endif

TARGET = ./bin/libv_repExtMultiContactPlanner.$(EXT)

VREP_HOME = /home/paolo/Software/V-REP_PRO_EDU_V3_6_1_Ubuntu18_04

default: v_repExtMultiContactPlannerLib
	cp ./bin/libv_repExtMultiContactPlanner.${EXT} ${VREP_HOME}

v_repExtMultiContactPlannerLib: $(OBJS)
		@echo "Linking $(OBJS)  to $(TARGET)"
		$(ECHO)$(CXX) $(CFLAGS) $(OBJS) $(OPTION) -o $(TARGET) $(LDFLAGS) $(LIBS)

%.o: %.c
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

./build/%.o: ./%.cpp
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

clean:
		@echo "Cleaning $(OBJS) $(TARGET)"
		$(ECHO)rm -rf $(OBJS) $(TARGET)
