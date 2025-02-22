# m20-custom-firmware
The goal of the project is to reverse engineer the Meteomodem M20 radiosonde and build custom firmware for it for use in ham radio baloons and the [Horus Binary](https://github.com/projecthorus/horusdemodlib/wiki) V2 radio protocol. 

# Code
The code is writen in c using STM32CubeMX and Low Layer (LL) libraries and compiled using arm-none-eabi toolchain. Now fits into the original STM32L051R6T6 chip.

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
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/bottom.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/sticker.jpg?raw=true)

# Schematics
Great pcb reverse enginering work was made by [joyel24](https://github.com/joyel24/M20-radiosonde-firmware-alt), [PDF link](https://www.egimoto.com/dwld/17528ed1858138.pdf) (although there are some errors in it)

# Used libraries
- NMEA parsed based on https://github.com/sztvka/stm32-nmea-gps-hal
- LPS22 implementation based on https://github.com/KitSprout/KSDK/tree/master/firmwareSTM32/KSSTM_Module_LPS22HB/Program/modules
- Radio modules implementation based on https://github.com/adamgreig/wombat
- Horus Binary encoder based on https://github.com/whallmann/RS41HUP_V2/blob/master/horus_l2.c

# GPS
There are 2 variants of GPS modules, both of them are supported.

## GPS (NMEA)
In newer M20 sondes u-blox MAX-M10M that uses NMEA protocol.
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_new.png?raw=true)

## GPS (old)
In older M20 sondes xm1110 GPS module was used. It transmits data over UART but with custom firmware that transmits only binary protocol data.
Data format:
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/GPS.png?raw=true)

# Barometer and temp sensor
LPS22HB sensor is used with SPI interface. File lps22hb.c and lps22hb.c are a library for this sensor.

# Radio
TODO

# Build and Flash instruction
TODO (Soon)
