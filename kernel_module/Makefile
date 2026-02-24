# Define the module object
obj-m += rpi_fast_irq.o

# Get the path to the kernel headers for the currently running kernel
KDIR := /lib/modules/$(shell uname -r)/build

# Get the current working directory
PWD := $(shell pwd)

# Default target to build the module
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Target to clean up compiled files
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean