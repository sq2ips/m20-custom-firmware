cmake_minimum_required(VERSION 4.0)
project(m20_custom_firmware C)

set(CMAKE_C_STANDARD 11)

include_directories(m20/Core/Inc)

add_executable(m20_custom_firmware
        m20/Core/Inc/adf.h
        m20/Core/Inc/afsk.h
        m20/Core/Inc/aprs.h
        m20/Core/Inc/asn1crt.h
        m20/Core/Inc/asn1crt_encoding.h
        m20/Core/Inc/config.h
        m20/Core/Inc/fsk4.h
        m20/Core/Inc/gps.h
        m20/Core/Inc/horus_l2.h
        m20/Core/Inc/horus_v3.h
        m20/Core/Inc/lps22hb.h
        m20/Core/Inc/main.h
        m20/Core/Inc/nmea.h
        m20/Core/Inc/stm32_assert.h
        m20/Core/Inc/stm32l0xx_it.h
        m20/Core/Inc/utils.h
        m20/Core/Inc/xm_gps.h
        m20/Core/Src/adf.c
        m20/Core/Src/afsk.c
        m20/Core/Src/aprs.c
        m20/Core/Src/asn1crt.c
        m20/Core/Src/asn1crt_encoding.c
        m20/Core/Src/fsk4.c
        m20/Core/Src/gps.c
        m20/Core/Src/horus_l2.c
        m20/Core/Src/horus_v3.c
        m20/Core/Src/lps22hb.c
        m20/Core/Src/main.c
        m20/Core/Src/nmea.c
        m20/Core/Src/stm32l0xx_it.c
        m20/Core/Src/syscalls.c
        m20/Core/Src/sysmem.c
        m20/Core/Src/system_stm32l0xx.c
        m20/Core/Src/utils.c
        m20/Core/Src/xm_gps.c
        m20/tests/test.c)
