# Fallback Makefile for lib/remi_pi

# Compiler and flags
CXX := g++
CXXFLAGS := -Wall -O2 -DREMI_PI

# Source files
SRCS := $(wildcard *.cc)
OBJS := $(SRCS:.cc=.o)

DEFINES+=$(USER_DEFINES)

DEFINES+=-DDEFAULT_HARDWARE='"$(HARDWARE_DESC)"'
CFLAGS=-W -Wall -Wextra -Wno-unused-parameter -O3 -g -fPIC $(DEFINES) -march=native

INCDIR=../../include

TARGET=framebuffer

# Default target to build the library
all: $(TARGET).a

$(TARGET).a : $(OBJECTS)
	$(AR) rcs $@ $^

# Rule to compile source files into object files
%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o : %.c compiler-flags
	$(CC)  -I$(INCDIR) $(CFLAGS) -c -o $@ $<

# Clean up build artifacts
clean:
	rm -f $(OBJS) $(TARGET).a

.PHONY: all clean
