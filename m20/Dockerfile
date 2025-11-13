FROM alpine:3.22

RUN apk add --no-cache make gcc-arm-none-eabi newlib-arm-none-eabi

WORKDIR /opt/m20

CMD set -e && make clean && make && make test
