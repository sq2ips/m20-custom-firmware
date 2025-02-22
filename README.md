# m20-custom-firmware
The goal of the project is to reverse engineer the Meteomodem M20 radiosonde and build custom firmware for it for use in ham radio baloons and the [Horus Binary](https://github.com/projecthorus/horusdemodlib/wiki) V2 radio protocol. 

# Code
The code is writen in C using STM32CubeMX and Low Layer (LL) libraries and compiled using arm-none-eabi toolchain. Now fits into the original STM32L051R6T6 chip.

# Stage
In this state the code works to the point where it gets GPS data and sends it using Horus Binary over radio. However this code is currently highly experimental and will probably not yet work correctly in the intended application.

# What works
- GPS (NMEA): :heavy_check_mark:
- GPS (XM1110) :heavy_check_mark:
- Radio: :heavy_check_mark:
- Uart: :heavy_check_mark:
- Outside temperature sensor: :x:
- LPS22 barometer + temp sensor: :x: (to be done soon)
- humidity sensor: :x:

# Images

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/side.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/pcb.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/pcb2.jpg?raw=true)

# Schematics
Great pcb reverse enginering work was made by [joyel24](https://github.com/joyel24/M20-radiosonde-firmware-alt), [PDF link](https://www.egimoto.com/dwld/17528ed1858138.pdf) (although there are some errors in it)

# Used libraries
- NMEA parsed based on https://github.com/sztvka/stm32-nmea-gps-hal
- LPS22 implementation based on https://github.com/KitSprout/KSDK/tree/master/firmwareSTM32/KSSTM_Module_LPS22HB/Program/modules
- Radio modules implementation based on https://github.com/adamgreig/wombat
- Horus Binary encoder based on https://github.com/whallmann/RS41HUP_V2/blob/master/horus_l2.c

# GPS
There are 2 variants of GPS modules, both of them are supported.
## New GPS (NMEA)
In newer M20 sondes u-blox MAX-M10M that uses NMEA protocol.

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_new.jpg?raw=true)

## Old GPS (XM1110)
In older M20 sondes XM1110 GPS module was used. It transmits data over UART but with custom firmware that transmits only binary protocol data.

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_old.jpg?raw=true)
Data format:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/GPS.png?raw=true)

# Barometer and temp sensor
LPS22HB sensor is used with SPI interface. File lps22hb.c and lps22hb.c are a library for this sensor.

# Radio
TODO

# Running the firmware
First you need to obtain the code, you can do it with `git`:
```bash
git clone https://github.com/sq2ips/m20-custom-firmware.git
```
Or directly from github.
# Configuration
Before building the firmware you fist need to configure parameters located in the [`config.h`](https://github.com/sq2ips/m20-custom-firmware/blob/main/m20/Core/Inc/config.h) file.
TODO description
# Building the firmware
Beofere flashing the firmware you need to build it first, there are a few ways you can do it depending on the platform:
## Building directly on Linux
To build directly on linux you need the arm-none-eabi toolchain, you can install it from your package manager depending on the linux distro.
For example, on Debian it will look like this:
```bash
sudo apt install gcc-arm-none-eabi
```
### WARNING: in difrent distros version of the toolchain can varry and result in a too big result file
Another way that avoids this problem is downloading the toolchain from [here](https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz).
After downloading you need to extract it:
```bash
tar -xvf arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz
```
and install the binaries into your system:
```bash
sudo cp arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/* /usr/local/bin/
```
You will also need `make` to build.
```bash
sudo apt install make
```
After installing everything you can go into the directory where you do downloaded the code, then:
```bash
cd m20
```
And now you can `make` the firmware:
```bash
make
```
After succesful building you should see a memoy usage table like this:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/memory.png?raw=true)

## Building with Docker on Linux
## Building with Docker on Windows
## Building with WSL on Windows (TODO)
