# Compiler settings
CXX := g++
CXXFLAGS := -Wall -Wextra -O3 -std=c++17
LDFLAGS := -pthread

# Target executable name
TARGET := irq_test.x

# Source files
SRCS := main.cpp RpiFastIrq.cpp

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