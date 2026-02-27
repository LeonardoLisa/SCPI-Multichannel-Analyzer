# Compiler settings
CXX := g++

# Retrieve ROOT flags and libraries dynamically
ROOT_CFLAGS := $(shell root-config --cflags)
ROOT_LIBS   := $(shell root-config --libs)

CXXFLAGS := -Wall -Wextra -O3 -std=c++17 -pthread $(ROOT_CFLAGS)
LDFLAGS  := -pthread $(ROOT_LIBS)

# Target executable name
TARGET := pmode_root.x

# Source files
SRCS := pmode_root.cpp RpiFastIrq.cpp

# Object files
OBJS := $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files into object files
%.o: %.cpp RpiFastIrq.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean