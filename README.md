# m20-custom-firmware
The goal of the project is to reverse engineer the Meteomodem M20 radiosonde and build custom firmware for it for use in ham radio baloons. Work is currently underway to make individual modules work such as GPS, radio etc. Once this is achieved, the full firmware will be built. In this state, the code only tests individual elements.
# What works
- GPS: :heavy_check_mark:
- Radio: :x:
- Uart: :heavy_check_mark:
- temperature sensor: :x:
- barometer: :x:
- humidity sensor: :x:

# Images
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/bottom.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/pcb.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/side.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/side.jpg?raw=true)
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/sticker.jpg?raw=true)

# GPS
M20 uses xm1110 gps module that transmits data over UART but with custom firmware that transmits only binary protocol data.
Data format:
![alt text](https://github.com/sq2ips/m20-custom-firmware/blob/main/img/GPS.png?raw=true)

