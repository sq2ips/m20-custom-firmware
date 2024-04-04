# m20-custom-firmware
The goal of the project is to reverse engineer the Meteomodem M20 radiosonde and build custom firmware for it for use in ham radio baloons. Work is currently underway to make individual modules work such as GPS, radio etc. Once this is achieved, the full firmware will be built. In this state, the code only testing individual elements.
# What works
- GPS: :heavy_check_mark:
- Radio: only CW
- Uart: :heavy_check_mark:
- temperature sensor: :x:
- barometer: :heavy_check_mark:
- humidity sensor: :x:

# Images
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/side.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/pcb.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/bottom.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/sticker.jpg?raw=true)

# Schematics
Great pcb reverse enginering work was made by [joyel24](https://github.com/joyel24/M20-radiosonde-firmware-alt), [PDF link](https://www.egimoto.com/dwld/17528ed1858138.pdf)

# Code
Code is writen in C using stm32CubeIDE.

# GPS
M20 uses xm1110 gps module that transmits data over UART but with custom firmware that transmits only binary protocol data.
Data format:
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/GPS.png?raw=true)
It has been observed that in newest models of M20 other module: u-blox MAX-M10M that is using standard NMEA is used.
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/gps_new.png?raw=true)

# Barometer
LPS22HB sensor is used with SPI interface. File lps22hb.c and lps22hb.c are a library for this sensor.

# Radio
TODO
