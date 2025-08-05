# Simple Kernel Module Example for BMP180 Sensor 

Example kernel module to read data from a BMP180 sensor using I2C, primarily build for Raspberry Pi. Purpose of this project is to demonstrate kernel space driver basics. This module reads temperature and pressure data from the BMP180 sensor and prints it to the kernel log. 

## Prerequisites

- Raspberry Pi with Raspberry Pi OS
- Kernel headers built for your current kernel version
- BMP180 sensor
- i2c-tools(optional for debugging)

## Building the Kernel

To build the linux kernel for Raspberry Pi follow instructions from the official Raspberry Pi documentation. 
(https://www.raspberrypi.com/documentation/computers/linux_kernel.html#configure-the-kernel)

Cross-compile the kernel if possible (takes too much time on the Raspberry Pi, might cause problems if the Raspberry Pi is headless). The Makefile is set up for cross-compilation.

## Building the Module

To build the kernel module, navigate to the directory containing the `Makefile` and run:

```bash
make KDIR=/path/to/kernel/headers
```
To build the device tree overlay, run following command on the Raspberry Pi:

## Building the Device Tree Overlay

Make sure you have the device tree compiler installed:

```bash
dtc -@ -I dts -O dtb -o bmp180.dtbo bmp180-overlay.dts
```
Then copy the `bmp180.dtbo` file to `/boot/overlays/` directory:

```bash
sudo cp bmp180.dtbo /boot/overlays/
```

## Loading and Unloading the Module
To load the module:

```bash
sudo insmod bmp180.ko
```
To unload the module:

```bash
sudo rmmod bmp180
```

## Usage 

Currently this module has no usage at all. Only way to see the data is to check the kernel log:

```bash
dmesg | grep bmp180
```
or

```bash
dmesg --follow
```

## Conflicting Modules

By default, Raspberry Pi OS has the module bmp280 enabled, which conflicts with this module. To disable it, run:

```bash
sudo modprobe -r bmp280
```
If that doesn't work, you can delete the module from the kernel:

```bash
sudo rm /lib/modules/$(uname -r)/kernel/drivers/iio/pressure/bmp280*
```
or backup the module:

```bash
mkdir -p ~/.backup/modules
sudo mv /lib/modules/$(uname -r)/kernel/drivers/iio/pressure/bmp280* ~/.backup/modules/
```
or better yet, if you are willing to spend some time, you can recompile the kernel without the bmp280 module, and install your custom kernel with the bmp180 module.


## Acknowledgements

- [Raspberry Pi](https://www.raspberrypi.com/)
- [BMP180 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)
- [Linux Kernel Documentation](https://www.kernel.org/doc/html/latest/)


