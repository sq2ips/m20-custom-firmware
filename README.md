# m20-custom-firmware
The goal of this project is to reverse engineer the Meteomodem M20 radiosonde and build custom free and open-source firmware for its usage in ham radio baloons based on the [Horus Binary V2](https://github.com/projecthorus/horusdemodlib/wiki) radio protocol.

# Code
The code is writen in C using STM32CubeMX (not to be confused with STM32CubeIDE) and Low Layer (LL) libraries and compiled using arm-none-eabi toolchain. Now it fits into the original STM32L051R6T6 chip.
Building and flashing instructions are placed further in this file.

# Stage
In this stage the code works to the point where it gets GPS and sensors data, then sends it using Horus Binary V2 protocol over radio. However this code is currently in the experimental/testing phase and there are some problems with it. Keeping that in mind, making flights with it is possible. Thanks to SP9AOB and SP6MPL for conducting test flights. If you are making a flight with this firmware let me know, it really helps with finding bugs and testing the code.

# What works
- GPS (NMEA): :heavy_check_mark:
- GPS (XM1110) :heavy_check_mark:
- Radio (Horus): :heavy_check_mark:
- LPS22 barometer + temp sensor: :heavy_check_mark:
- Uart: :heavy_check_mark:
- Outside temperature sensor: :heavy_check_mark:
- humidity sensor: :x:

# Features list
The currently implemented features are:
- GPS time, position, altitude, speed, ascent rate and number of satellites (NMEA and XM1110)
- Sending data over radio using Horus Binary V2 protocol
- Getting battery voltage
- Getting temperature and pressure
- Getting external temperature
- Watchdog timer

# Planned work
- making use of STM32 energy saving states
- Implementing XM1110 GPS speed data
- implementing humidity sensors
- implementing APRS

# Known issues
- u-blox GPS module altitude limit to 12km (mode change implementation in progress)
- High frequency instability (no TCXO)
- Transmitted frequency "jumps"

# Horus 4FSK tone spacing
Due to hardware limitations (system clock PLL setting options) it is not possible to generate a clock signal for the radio module whose frequency is divisible by 9. That results in no possibility of having a 270Hz tone spacing standardized by the RS41ng project. The tone spacing is set to 244Hz acquired by an 8MHz clock signal. The limitation is directly connected with the method of implementing FSK and there seems to be no way to overcome it without hardware intervention. The effect is that receiving stations must set a different from standard tone spacing or the SNR will be very low, for comparison:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/snr270.png?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/snr244.png?raw=true)

# Authors
- Paweł SQ2IPS
- Jędrzej SQ2DK

# License
See LICENSE

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
In newer M20 sondes u-blox MAX-M10M that uses NMEA protocol is used.

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_new.jpg?raw=true)

## Old GPS (XM1110)
In older M20 sondes XM1110 GPS module is used. It transmits data over UART but with custom firmware that transmits only binary protocol data.

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_old.jpg?raw=true)

Data format:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/GPS.png?raw=true)

# Barometer and temp sensor
LPS22HB sensor is used with SPI interface.

# External temperature sensor
A NTC is used for external temperature measuring with addable resistors, the schematic looks like this:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/NTC.jpg?raw=true)

# Radio
TODO

# Running the firmware
## What you will need
Hardware requirements:
- A working M20 radiosonde :)
- A ST-Link v2 programmer USB dongle that looks like this:

![alt text](https://cdn-shop.adafruit.com/970x728/2548-01.jpg)

- 5 male to female goldpin jumper wires
- A computer with Linux or Windows

## Recomended hardware modifications
## Load resistor
If you have a sonde with new GPS module, there is a additional parallel 62 Ohm resistor added at the output of the voltage converter, all it does it drawing ~53mA from the line and converting it to heat. I have no idea why it was added, removing it does not make the power supply unstable or anything like that, maybe it was added for draining the battery quicker. You can safely remove this resistor to save some energy from the battery.
This is the resistor:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/rezystor.jpg?raw=true)

Thermal camera image:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/termo.jpg?raw=true)

(Image from SP9AOB)

## Downloading code
First you need to obtain the code, you can do it with `git`:
```bash
git clone https://github.com/sq2ips/m20-custom-firmware.git
```
From github website "code" button, or directly from [here](https://github.com/sq2ips/m20-custom-firmware/archive/refs/heads/main.zip), and then unzip the file.
# Configuration
Before building the firmware you fist need to configure parameters located in the [`config.h`](https://github.com/sq2ips/m20-custom-firmware/blob/main/m20/Core/Inc/config.h) file.
Parameters list:
| parameter | type (and unit) | description |
|-----------|------|-------------|
| `PAYLOAD_ID` | uint16 | Payload ID transmitted in Horus Binary frame, in order to conduct a flight you need to request one for your callsign, more information in the [Protocol documentation](https://github.com/projecthorus/horusdemodlib/wiki#how-do-i-transmit-it). For testing ID 256 is used |
| `TIME_PERIOD` | uint (in seconds) | Time between transmition of frames. Should not be lower than 4. |
| `GPS_TYPE` | uint | Type of GPS module, eather 1 for u-blox MAX-M10M, 2 for XM1110 module. For identifying the module see [GPS](https://github.com/sq2ips/m20-custom-firmware?tab=readme-ov-file#gps) section. |
| `QRG_FSK4` | uint (in Hz) | Frequency of 4FSK Horus transmition. |
| `PA_FSK4` | uint | Number from 0 to 63. TODO power levels measurements. |
| `RF_BOOST_ACTIVE` | bool | State of RF TX boost, amplifies signal by around 15dB. (In off state the boost cricut is attenuating the signal, when less output power is needed it's better to decrease `PA_FSK4` than turning it off.) |
| `ADF_FREQ_CORRECTION` | uint (multiples of 244Hz) | Frequency correction for transmitted signal |
| `LED_MODE` | uint | 0 - disabled, 1 - LED on while getting data before transmission, 2 - Fix type indication (1 flash for no fix, 2 flashes for 2D fix, 3 flashed 3D fix) |
| `LED_PERIOD` | uint | only for `LED_MODE` 3, time between fix indication |
| `LED_DISABLE_ALT` | uint (in meters) | only for `LED_MODE` 3, disables LED when prompted altitude is reached. |
# Building the firmware
Before flashing the firmware you need to build it first, there are a few ways you can do it depending on the platform:
## Building directly on Linux
To build directly on linux you need the arm-none-eabi toolchain, you can install it from your package manager depending on the linux distro.
For example, on Debian it will look like this:
```bash
sudo apt install gcc-arm-none-eabi
```
### WARNING: in different distros version of the toolchain can varry and result in a too big result file
A way to avoids this problem is downloading the toolchain from [here](https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz).
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
First you need to get Docker, you can install it from your package manager depending on the linux distro.
For example, on Debian it will look like this:
```bash
sudo apt install docker
```
Then you need to add your user into the docker group like so:
```bash
sudo groupadd docker && sudo usermod -aG docker $USER
```
Now restart:
```bash
sudo reboot
```

Now you need to start the docker daemon:
```bash
sudo systemctl start docker
```
But you will need to do it again after reboot of your computer, you can set it to autostart:
```bash
sudo systemctl enable docker
```
Now you should cd into the directory of the downloaded code, then:
```bash
cd m20
```
And now build the Docker image:
```bash
docker build -t m20 .
```
It will need some time to download an install the packages, after it finishes run:
```bash
docker run --rm -v $(pwd):/opt/m20 m20:latest
```
It will build the code and after finishing you should see a memory usage table, the compiled binaries should be in the `build/` directory.
If you did the steps next time building you will need to only run the last command.
## Building with MinGW on Windows
First [download MSYS2](https://www.msys2.org/), then install it, after finishing the setup you should now see a terminal.
Now install `make`:
```bash
pacman -Sy make
```
Confirm the instalation and wait for the packages to download and install.
TODO
## Building with Docker on Windows
First [download Docker Desktop](https://www.docker.com/products/docker-desktop/) and install it, then run the Docker engine.
Open Windows PowerShell and go into the directory of the project then `m20/`, next run:
```bash
docker build -t m20 .
```
It will need some time to download an install the packages, after it finishes run:
```bash
docker run --rm -v .:/opt/m20 m20:latest
```
It will build the code and after finishing you should see a memory usage table, the compiled binaries should be in the `build/` directory. 

# Flashing the firmware
## Connecting
Before flashing you first need to connect the sonde to your computer through the ST-LINK programmer.
You can do it using 5 goldpin cables. This is the sonde pinout:

![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/pinout.jpg?raw=true)

Follow it and the pinout of the programmer printed on the case.
After connecting it and the programmer to your USB port you are now ready to flash the firmware.
You don't need to solder the wires, instead you can just put them into the connector, it should make enough contact for the firmware to flash.

## Flashing on Linux
For flashing you will need the OpenOCD you can install it from your package manager depending on the linux distro.
For example, on Debian it will look like this:
```bash
sudo apt install openocd
```
After installing it and ensuring that you are in the `m20` directory. Then check if ST-Link is connected and then you can remove the write protection (only before the first flash):
```bash
make protect
```
or if you don't have `make` or want to run it directly:
```bash
openocd -s ./openocd/ -f ./openocd/openocd_m20.cfg -c "init; halt; flash protect 0 0 7 reset; exit"
```
After it finishes you can flash the built firmware:
```bash
make flash
```
or directly
```bash
openocd -s ./openocd/ -f ./openocd/openocd_m20.cfg -c "program build/m20.elf verify reset exit"
```
After it finishes your sonde should now work with the new firmware.

## Flashing on Windows
First [download OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/latest) select the file with ending `win32-x64.zip`, then extract it.
Go to the path of the project then to `m20/`. Ensure that ST-Link is connected and then remove the write protection (only before the first flash):
```cmd
<path\to\openocd>\bin\openocd.exe -s openocd -f openocd\openocd_m20.cfg -c "init; halt; flash protect 0 0 7 reset; exit"
```
replace `<path\to\openocd>` with path of the extracted program.
After it finishes you can flash the built firmware:
```cmd
<path\to\openocd>\bin\openocd.exe -s openocd -f openocd\openocd_m20.cfg -c "program build/m20.elf verify reset exit"
```
After it finishes your sonde should now work with the new firmware.
## Flashing on Windows with ST-Link utility
Fisrt [download the utility](https://www.st.com/en/development-tools/stsw-link004.html#get-software), extract, install and run it. Connect to the ST-Link and open the binary compiled file with .bin extension. Then go to Target -> Flash & Verify, check that the adress is set to 0x80000000 and the flash.

# Debuging (TODO)
