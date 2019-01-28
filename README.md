# ntpriddle

NTP server implementation for STM32F746ZG

# hardware
* NUCLEO-F746ZG
* GPS module
    * require NMEA 0183 and 1PPS signal output
    * http://akizukidenshi.com/catalog/g/gK-13849/

# how to build

```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt update && sudo apt install gcc-arm-embedded
make
```

