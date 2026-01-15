# m20-custom-firmware
The goal of this project is to reverse engineer the [Meteomodem M20](https://www.meteomodem.com/m20) radiosonde and build custom free and open-source firmware for its usage in ham radio balloons.

# Code
The code is writen in C using STM32CubeMX (not to be confused with STM32CubeIDE) generated project and Low Layer (LL) libraries, compiled using arm-none-eabi toolchain (version 15.1.0 is used). Now it fits into the original STM32L051R6T6 chip.
Building and flashing instructions are placed further in this file.

# Stage
In this stage the code works quite well and was tested on many flights. Thanks to SP9AOB and SP6MPL for conducting test flights. It was used as a tracker for some HAB missions and floater balloons. I also made a few flights with the SP2ZIE amateur radio club.
Still, there can be errors and bugs in it, it is distributed "AS-IS", there is no waranty, see [the License](#LICENSE).
If you are making a flight with this firmware let me know, it really helps with finding bugs and testing the code.

# Contributing
Any code ideas, questions, bug reports and code contributions are welcome.
If you are having an idea for the project or found a bug you can create an issue.
If you want to contribute code to the project you can create a pull request.
if you are having any other questions, ideas or issues you can write me an e-mail: pawel.sq2ips@gmail.com.

# What works
- GPS (NMEA): :heavy_check_mark:
- GPS (XM1110) :heavy_check_mark:
- Radio (Horus V2): :heavy_check_mark:
- Radio (Horus V3): :heavy_check_mark:
- Radio (APRS): :heavy_check_mark:
- LPS22 barometer + temp sensor: :heavy_check_mark:
- Uart: :heavy_check_mark:
- Outside temperature sensor: :heavy_check_mark:
- humidity sensor: :x:

# Features list
The currently implemented features are:
- GPS time, position, altitude, speed (not working in XM1110), ascent rate and number of satellites (NMEA and XM1110)
- changing u-blox module mode to airborne to allow higher altitude flights
- Sending data over radio using Horus Binary V2/V3 protocol and/or APRS protocol
- Getting battery voltage
- Getting temperature and pressure
- Getting external temperature
- Watchdog timer
- GPS watchdog for no fix (useful in GPS jamming/spoofing areas)
- transmission frequency switching
- Additional ADC for payload / PV voltage

# Planned work
- making use of STM32 energy saving states
- Implementing XM1110 GPS speed data
- implementing humidity sensors

# Known issues
- High frequency instability (no TCXO)
- Transmitted frequency "jumps"
- High RAM consumption caused by staticly alocated memory for NMEA GPS parsing.

# Horus 4FSK tone spacing
Due to hardware limitations (system clock PLL setting options) it is not possible to generate a clock signal for the radio module whose frequency is divisible by 9. That results in no possibility of having a 270Hz tone spacing standardized by the RS41ng project. The tone spacing is set to 244Hz acquired by an 8MHz clock signal. The limitation is directly connected with the method of implementing FSK and there seems to be no way to overcome it without hardware intervention. The effect is that receiving stations must set a different from standard tone spacing or the SNR will be very low, for comparison:

![snr270](./img/snr270.png?raw=true)
![snr244](./img/snr244.png?raw=true)

# Authors
- Paweł SQ2IPS
- Jędrzej SQ2DK

Big thanks to:
- SP9AOB and SP6MPL for testing, making flights and finding new ideas for the project
- SP2IML for code contribution, writing tests, and debuging the code

# License
See LICENSE

# Images

![alt text](./img/side.jpg?raw=true)
![alt text](./img/pcb.jpg?raw=true)
![alt text](./img/pcb2.jpg?raw=true)

# Schematics
Great pcb reverse enginering work was made by [joyel24](https://github.com/joyel24/M20-radiosonde-firmware-alt), [PDF link](https://www.egimoto.com/dwld/17528ed1858138.pdf) (although there are some errors in it)

# Used libraries
- NMEA parsed based on https://github.com/sztvka/stm32-nmea-gps-hal
- LPS22 implementation based on https://github.com/KitSprout/KSDK/tree/master/firmwareSTM32/KSSTM_Module_LPS22HB/Program/modules
- Radio modules implementation based on https://github.com/adamgreig/wombat
- Horus Binary V2 encoder based on https://github.com/projecthorus/horusdemodlib/blob/master/src%2Fhorus_l2.c
- Horus Binary V3 encoder based on Mark VK5QI's fork of RS41-nfw https://github.com/darksidelemm/rs41-nfw/tree/main/fw/fw-files/rs41-nfw

# GPS
There are 2 variants of GPS modules, both of them are supported.
## New GPS (NMEA)
In newer M20 sondes u-blox [MAX-M10M](https://content.u-blox.com/sites/default/files/documents/MAX-M10M_DataSheet_UBX-22028884.pdf) that uses NMEA protocol is used.

The module has normal altitude limit to 12000m, that's why mode change is needed for stratospheric flights (to around 30000m). After startup a command is sent to the module to change its mode. Then the altitude limit is 80000m in exchange for lower max acceleration that is not needed anyway.

![alt text](./img/gps_new.jpg?raw=true)

## Old GPS (XM1110)
In older M20 sondes XM1110 GPS module is used. It transmits data over UART but with custom firmware that transmits only binary protocol data.

The module doesn't seem to have any altitude limit lower than 30000m.

Vertical speed from this module is not implemented yet due to weird frame format.

![alt text](./img/gps_old.jpg?raw=true)

Data format:

![alt text](./img/GPS.png?raw=true)

## GPS Watchdog
This is a feature meant to reset the GPS module when it stops working because of jamming/spoofling. https://gpsjam.org/ It is mainly implemented because of high level of interference present at north part of Poland.
It restarts the module if after getting an initial fix it will dissapear after set time.

# Barometer and temperature sensor
LPS22HB sensor is used with SPI interface, it sends pressure data, and additionaly temperature.

# External temperature sensor
A NTC is used for external temperature measuring with addable resistors, the schematic looks like this:

![alt text](./img/NTC.jpg?raw=true)

# Battery voltage reading
Battery is directly connected to one of the ADC pins, without any resistor divider, the voltage reference of ADC is 3.3V so it can't mesure voltages higher than that.

# Radio
The [ADF7012B](https://www.analog.com/media/en/technical-documentation/data-sheets/ADF7012.pdf) radio module is used.

## Horus Binary V2
[Horus Binary V2](https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet) is implemented with default custom format to be used with the [amateur Somdehub](https://amateur.sondehub.org/) infrastructure.
Sent data (implemented in [`horus.h`](./m20/Core/Inc/horus.h)):
| Byte No. | Data Type | Description |
|-|-|-|
| 0-1 | uint16 | Payload ID (0-65535) |
| 2-3 |	uint16 |	Sequence Number |
| 4 |	uint8 |	Time-of-day (Hours) |
| 5 |	uint8 |	Time-of-day (Minutes) |
| 6 |	uint8 |	Time-of-day (Seconds) |
| 7-10 |	float |	Latitude |
| 11-14 |	float |	Longitude |
| 15-16 |	uint16 |	Altitude (m) |
| 17 |	uint8 |	Speed (kph) |
| 18 |	uint8 |	Satellites |
| 19 |	int8 |	Temperature (deg C) from [LPS22HB](#barometer-and-temperature-sensor) sensor |
| 20 |	uint8 |	Battery Voltage from [battery ADC](#battery-voltage-reading) |
| 21-22 | int16 | Ascent rate (speed of changes in altitude) |
| 23-24 | int16 | External temperature from [NTC](#external-temperature-sensor) sensor |
| 25 | uint8 | Humidity, not implemented yet |
| 26-27 | uint16 | Pressure data from [LPS22HB](#barometer-and-temperature-sensor) sensor |
| 28 | uint8 | GPS watchdog restart count |
| 29 | uint8 | Additional ADC measurement, see [PV / payload voltage ADC](#pv--payload-voltage-adc) |
| 30-31 |	uint16 | CRC16-CCITT Checksum |
## Horus Binary V3
[Horus Binary V3](https://github.com/xssfox/horusbinaryv3) is a new radio protocol designed for HAB flights. It's based on protocol version V2 but additionaly uses Abstract Syntax Notation 1 (ASN.1) to describe the data format. It is highly customizable, and doesn't need a payload ID request like V2.
Note that since in V3 payload callsign is directly encoded into the frame, it's length depends on the callsign length so using any sufixes is not recommended if not really necessary.
The Fields in the packet are as follows:
| Field name | Constraint | Description |
|-|-|-|
| `payloadCallsign` | 1 to 15 characters : `a-z`,`A-Z`,`0-9`,`-` | Payload callsign |
| `sequenceNumber` | 0 - 65535 | Frame sequence number. |
| `timeOfDaySeconds` | -1 - 86400 | This is the time since midnight UTC. Received from the GPS module. |
| `latitude` | -9000000 - 9000000 | Current GPS latitude (fixt point multiplied by 100000).  |
| `longitude` | -18000000..18000000 | Current GPS longitude (fixt point multiplied by 100000). |
| `altitudeMeters` | -1000 - 50000 | GPS altitude. |
| `extraSensor`: `type` | string | Just sonde identifier, defaults to `M20`. |
| `extraSensor`: `gps` | uint8 | Number of GPS watchdog restarts. |
| `velocityHorizontalKilometersPerHour` | 0-512 | Horizontal velocity in km/h from GPS module. |
| `ascentRateCentimetersPerSecond` | -32767 - 32767 | Ascent rate in centimeters per second. |
| `gnssSatellitesVisible` | 0 - 31 | Number of satellites the GPS module can see. |
| `humidityPercentage` | 0 - 100 | Humidity in percentage, not implemented. |
| `pressurehPa` | 0 - 1200 | Atmospheric pressure in hPa from [LPS22HB](#barometer-and-temperature-sensor). |
| `temperatureCelsius`: `internal` | -1023 - 1023 | Internal temperature sensor value in Celsius (multiplied by 10), from [LPS22HB](#barometer-and-temperature-sensor) sensor. |
| `temperatureCelsius`: `external` | -1023 - 1023 | External temperature sensor value in Celsius (multiplied by 10), from [NTC](#external-temperature-sensor) sensor. |
| `milliVolts`: `battery` | 0 - 16383 | [battery ADC](#battery-voltage-reading) voltage in millivolts. |
| `milliVolts`: `solar` | 0 - 16383 | Voltage in millivolts from additional ADC measurement, see [PV / payload voltage ADC](#pv--payload-voltage-adc) |

Some sensor fileds are enabled with corresponding sensors enable value from `config.h`.

## Note about Horus Binary
From [horusdemodlib wiki](https://github.com/projecthorus/horusdemodlib/wiki):
> Starting from horusdemodlib 0.6.0 it is recommended to use Horus Binary V3 for new designs, projects and launches. This format is more flexible and does not require pull requests to be submitted for payload IDs and custom data types

## APRS
The Automatic Packet Reporting System (APRS) is also commonly used for ham balloon flights because of the big network of internet forwarding ground stations together with the [amateur Sondehub](https://amateur.sondehub.org/) infrastructure. Frames are collected from the APRS-IS, parsed by [sondehub-aprs-gateway](https://github.com/projecthorus/sondehub-aprs-gateway) and sent to the map.

The AFSK modulation used by APRS is implemented just like in [Trackduino](https://github.com/trackuino/trackuino/blob/1.52/trackuino/afsk.cpp).
The AFSK Bell 202 tones are not supported in the radio module, so another method needed to be developed. It works by using direct mode, where the TX_DATA pin is fed by a high-frequency PWM signal of a sine wave, where phase is increased by values corresponding to the Bell 202 tones, creating a 1-bit DAC. That method also allows continuous phase change, which is critical for AFSK. More on the implementation is in the comments of [`afsk.c`](./m20/Core/Src/afsk.c). On top of AFSK AX.25 the APRS standard is used for routing packages, formatting and compressing position, and adding a comment field. In it telemetry is inserted to be parsed by [sondehub-aprs-gateway](https://github.com/projecthorus/sondehub-aprs-gateway). The implementation is in [`aprs.c`](./m20/Core/Src/aprs.c).

## Note about modes
APRS is not very eficient due to the modulation characteristics and no forward error correction, also the number of automatic receiving stations is way smaller than Horus.
That's why if you are making a flight it's good to use Horus or both modes, not only APRS.

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
If you have a sonde with new GPS module, there is a additional parallel 62 Ohm resistor added at the output of the voltage converter, all it does it drawing ~53mA from the line and converting it to heat. I have no idea why it was added, removing it does not make the power supply unstable or anything like that, maybe it was added for draining the battery quicker or heating up the board. You can safely remove this resistor to save some energy from the battery.
This is the resistor:

![alt text](./img/rezystor.jpg?raw=true)

Thermal camera image:

![alt text](./img/termo.jpg?raw=true)

(Image from SP9AOB)

## PV / payload voltage ADC
Pin PA0 allows to measure additional voltage, for example from another payload batteries or from a solar panel (PV). It uses a resistor divider to allow voltages bigger than the ADC reference 3.3V. To enable it set `PV_ADC_ENABLE` to 1.
This is the connection schematic, green is PA0 pin, black is ground (other GND can be used as well):

![alt text](./img/pcb_adc.png?raw=true)

Values of R1 and R2 need to be set in config (`PV_ADC_R1`, `PV_ADC_R2`).
For example for R1 = 1k and R2 = 2k the max voltage is 4.95V.
When setting values in config you can reduce the values fraction if possible (for 1000R and 2000R you can set 1 and 2 in config).

~~Note that Horus voltage field is an unsigned byte of values corresponding to voltage range from 0 to 5V, so higher values can't be represented.
For this to work with Horus decoders you need to make a PR to add your call assigned to payload ID to [Horus custom field list](https://github.com/projecthorus/horusdemodlib/blob/master/custom_field_list.json), in the section of `"SP2ZIE"` in `"other_payloads"` add your call in the list after a `,` in `""`, for example when the table looks like this: `["SQ2IPS"]` adding a call SP0ABC is: `["SQ2IPS", "SP0ABC"]`.~~ Since Horus Binary V3 none of this is necessary.

## Downloading code
First you need to obtain the code, you can do it with `git`:
```bash
git clone https://github.com/sq2ips/m20-custom-firmware.git
```
From github website "code" button, or directly from [here](https://github.com/sq2ips/m20-custom-firmware/archive/refs/heads/main.zip), and then unzip the file.
# Configuration
Before building the firmware you fist need to configure parameters located in the [`config.h`](./m20/Core/Inc/config.h) file. The most important parameters to change are in bold.
Parameters list:
| parameter | type (and unit) | description |
|-----------|------|-------------|
| **`TIME_PERIOD`** | uint (in seconds) | Time between transmition of frames. Should not be lower than 4. |
| `HORUS_ENABLE` | uint | 0 for disabled, 2 for Horus Binary V2, 3 for Horus Binary V3.
| **`QRG_FSK4`** | float[] (in Hz) | Transmitted frequencies array of Horus 4-FSK, switched in a loop, add new frequencies after a comma in braces. Refer to [Horus wiki](https://github.com/projecthorus/horusdemodlib/wiki#commonly-used-frequencies) for commonly used frequencies. |
| **`FSK4_POWER`** | uint | Number from 0 to 63. See the table bellow. |
| **`HORUS_V2_PAYLOAD_ID`** | uint16 | Payload ID transmitted in Horus Binary V2 frame, in order to conduct a flight you need to request one for your callsign, more information in the [Protocol documentation](https://github.com/projecthorus/horusdemodlib/wiki#how-do-i-transmit-it). For testing ID 256 is used. |
| **`HORUS_V3_PAYLOAD_CALLSIGN`** | string | Payload callsign string for Horus Binary V3, 1 to 15 characters : `a-z`,`A-Z`,`0-9`,`-`. |
| `FSK4_BAUD` | uint | Baudrate of the 4-FSK transmission. |
| `FSK4_SPACE_MULTIPLIER` | uint | Tone spacing multiplier - 1 for 244Hz, 2 for 488, etc. |
| `FSK4_HEADER_LENGTH` | uint | Length in bytes of 4FSK sync header. |
| `TX_PAUSE` | uint (ms) | Delay between HORUS and APRS when both are enabled. |
| `APRS_ENABLE` | bool | Enables the APRS AFSK transmission. |
| **`QRG_AFSK`** | float[] (in Hz) | Just like `QRG_4FSK`, commonly used in europe is 432.500MHz. |
| **`AFSK_POWER`** | uint | Just like `FSK4_POWER`. |
| **`APRS_CALLSIGN`** | string | Callsign of the sonde, put your callsign here (max 6 digits). |
| `APRS_SSID` | uint | Sonde callsign SSID, 11 is "balloons, aircraft, spacecraft, etc", refer to https://www.aprs.org/aprs11/SSIDs.txt. |
| `APRS_DESTINATION` | string | Destination adress, characterizing a M20 transmitter (max 6 digits). |
| `APRS_DESTINATION_SSID` | uint | Destination adress SSID. |
| `APRS_PATH` | string | APRS path 1, refer to https://blog.aprs.fi/2020/02/how-aprs-paths-work.html, (max 6 digits). |
| `APRS_PATH_SSID` | string | APRS path 1 SSID, note example WIDE2-1, will be "WIDE2" as `APRS_PATH` and 1 as `APRS_PATH_SSID`. [Balloons recomended path](https://www.aprs.org/balloons.html). |
| `APRS_SYMBOL` | string (2 characters) | First character is symbol table ID, eather / or \\, second character is the symbol. /O is balloon, [all symbols](https://www.aprs.org/symbols.html). Needs to be /O for showing on Sondehub. |
| `APRS_COMMENT_TELEMETRY` | bool | Enable telemetry in APRS comment field. |
| **`APRS_COMMENT_TEXT`** | string | Additional text in comment field. |
| `LED_MODE` | uint | 0 - disabled, 1 - LED on while getting data before transmission, 2 - Fix type indication (1 flash for no fix, 2 flashes for 2D fix, 3 flashed 3D fix). |
| `LED_PERIOD` | uint | only for `LED_MODE` 2, time between fix indication. |
| `LED_DISABLE_ALT` | uint (in meters) | only for `LED_MODE` 2, disables LED when prompted altitude is reached. |
| `RF_BOOST` | bool | State of RF TX boost, amplifies signal by around 15dB. (In off state the boost cricut is attenuating the signal, when less output power is needed it's better to decrease `PA_FSK4` than turning it off.) |
| `ADF_FREQ_CORRECTION` | uint (multiples of 244Hz) | Frequency correction for transmitted signal. |
| `ADF_FSK_DEVIATION` | uint (multiples of 244Hz) | Deviation parameter used in AFSK modem, don't change it without a reason. |
| `ADF_CLOCK` | uint | Clock speed of adf7012 chip coming from STM32 (in Hz) (set to HSE 8MHz oscilator). |
| **`GPS_TYPE`** | uint | Type of GPS module, eather 1 for u-blox MAX-M10M, 2 for XM1110 module. For identifying the module see [GPS](#gps) section. |
| `GPS_WATCHDOG` | bool | Enable [GPS Watchdog](#gps-watchdog). |
| `GPS_WATCHDOG_ARC` | uint | Number of main loop iterations without GPS fix that will trigger restart (Only if there was fix before). |
| `AscentRateTime` | uint | Time of ascent rate measure. |
| `LPS22_ENABLE` | bool | Enable LPS22 sensor. |
| `NTC_ENABLE` | bool | Enable NTC external temperature sensor. |
| `BAT_ADC_ENABLE` | bool | Enable battery voltage measure. |
| `PV_ADC_ENABLE` | bool | Enable additional ADC on pin PA0. |
| `PV_ADC_R1` | uint | R1 resistor value for divider. See [PV / payload voltage ADC](#pv--payload-voltage-adc) |
| `PV_ADC_R2` | uint | R2 resistor value for divider. |
| `DEBUG` | bool | Enable debug info over UART. |
| `GPS_DEBUG` | bool | Enable GPS data debug. |
| `GPS_RAW_DEBUG` | bool | Enable GPS raw frames debug. |

## Power setting
Power setting, `RF_BOOST` is on, mesured at 437.600MHz, directly at output.
| power setting | power |
| ------------- | ----- |
| 8  | 0,005W |
| 10 | 0,008W |
| 15 | 0,016W |
| 20 | 0,027W |
| 25 | 0,038W |
| 30 | 0,052W |
| 35 | 0,062W |
| 40 | 0,075W |
| 45 | 0,088W |
| 50 | 0,101W |
| 55 | 0,112W |
| 60 | 0,122W |
| 63 | 0,130W |

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

![alt text](./img/memory.png?raw=true)

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

![alt text](./img/pinout.jpg?raw=true)

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
openocd -s ./openocd/ -f ./openocd/openocd_m20.cfg -c "init; halt; flash protect 0 0 7 off; exit"
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
<path\to\openocd>\bin\openocd.exe -s openocd -f openocd\openocd_m20.cfg -c "init; halt; flash protect 0 0 7 off; exit"
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
