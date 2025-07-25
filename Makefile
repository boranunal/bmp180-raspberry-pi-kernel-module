# BMP180 Kernel Module Makefile
# Author: Utku Boran Unal
# Description: Cross-compilation Makefile for BMP180 driver on Raspberry Pi 4

# Module name
MODULE_NAME := bmp180

# Source files
obj-m += $(MODULE_NAME).o

# Cross-compilation settings
# Default values - can be overridden from command line
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

# Kernel source directory - MUST be specified
# Example: make KDIR=/path/to/rpi/linux/headers
KDIR ?= 

# Default kernel directory for native compilation (if not cross-compiling)
ifeq ($(KDIR),)
    KDIR := /lib/modules/$(shell uname -r)/build
endif

# Compiler flags
ccflags-y := -Wall -Wextra -std=gnu99

# Build target
all: check_kdir
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

# Check if kernel directory is specified and exists
check_kdir:
ifeq ($(KDIR),)
	@echo "Error: KDIR not specified!"
	@echo "Usage examples:"
	@echo "  make KDIR=/path/to/rpi/kernel/headers"
	@echo "  make KDIR=/usr/src/linux-headers-5.15.0-rpi"
	@echo "  make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- KDIR=/path/to/kernel"
	@exit 1
endif
	@if [ ! -d "$(KDIR)" ]; then \
		echo "Error: Kernel directory '$(KDIR)' does not exist!"; \
		exit 1; \
	fi
	@if [ ! -f "$(KDIR)/Makefile" ]; then \
		echo "Error: '$(KDIR)' does not appear to be a valid kernel build directory!"; \
		echo "Make sure it contains kernel headers and Makefile."; \
		exit 1; \
	fi

# Clean build artifacts
clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) clean
	rm -f *.o *.ko *.mod.* .*.cmd modules.order Module.symvers
	rm -rf .tmp_versions

# Install module (copy to modules directory)
install: all
	@echo "Installing $(MODULE_NAME).ko..."
	@if [ -z "$(INSTALL_MOD_PATH)" ]; then \
		echo "Warning: INSTALL_MOD_PATH not set. Module will be installed to host system."; \
		echo "For cross-compilation, use: make install INSTALL_MOD_PATH=/path/to/rpi/rootfs"; \
	fi
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) modules_install

# Help target
help:
	@echo "BMP180 Kernel Module Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build the kernel module (default)"
	@echo "  clean   - Remove build artifacts"
	@echo "  install - Install the module"
	@echo "  help    - Show this help message"
	@echo ""
	@echo "Variables:"
	@echo "  KDIR            - Kernel headers directory (REQUIRED)"
	@echo "  ARCH            - Target architecture (default: arm64)"
	@echo "  CROSS_COMPILE   - Cross-compiler prefix (default: aarch64-linux-gnu-)"
	@echo "  INSTALL_MOD_PATH- Installation root path for cross-compilation"
	@echo ""
	@echo "Examples:"
	@echo "  # Basic cross-compilation for RPi4:"
	@echo "  make KDIR=/home/user/rpi-kernel/linux"
	@echo ""
	@echo "  # With custom toolchain:"
	@echo "  make ARCH=arm64 CROSS_COMPILE=aarch64-rpi4-linux-gnu- KDIR=/path/to/headers"
	@echo ""
	@echo "  # For RPi3 (32-bit):"
	@echo "  make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/path/to/headers"
	@echo ""
	@echo "  # Install to target rootfs:"
	@echo "  make install KDIR=/path/to/headers INSTALL_MOD_PATH=/path/to/rpi/rootfs"
	@echo ""
	@echo "  # Native compilation (on Raspberry Pi):"
	@echo "  make KDIR=/lib/modules/\$$(uname -r)/build"

# Show current configuration
config:
	@echo "Current build configuration:"
	@echo "  MODULE_NAME     = $(MODULE_NAME)"
	@echo "  ARCH            = $(ARCH)"
	@echo "  CROSS_COMPILE   = $(CROSS_COMPILE)"
	@echo "  KDIR            = $(KDIR)"
	@echo "  INSTALL_MOD_PATH= $(INSTALL_MOD_PATH)"
	@echo "  PWD             = $(PWD)"

# Development targets
debug: ccflags-y += -DDEBUG -g
debug: all

# Create a simple test script
test-script:
	@echo "#!/bin/bash" > test_bmp180.sh
	@echo "# BMP180 Module Test Script" >> test_bmp180.sh
	@echo "" >> test_bmp180.sh
	@echo "echo 'Loading BMP180 module...'" >> test_bmp180.sh
	@echo "sudo insmod bmp180.ko" >> test_bmp180.sh
	@echo "echo 'Module loaded. Check dmesg for output:'" >> test_bmp180.sh
	@echo "dmesg | tail -10" >> test_bmp180.sh
	@echo "echo 'Module info:'" >> test_bmp180.sh
	@echo "modinfo bmp180.ko" >> test_bmp180.sh
	@echo "echo 'To unload: sudo rmmod bmp180'" >> test_bmp180.sh
	@chmod +x test_bmp180.sh
	@echo "Created test_bmp180.sh script"

.PHONY: all clean install help config check_kdir debug test-script