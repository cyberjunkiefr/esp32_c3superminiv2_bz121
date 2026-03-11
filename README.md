<!--#### Project -->
### Overview
This project implements a French Law compliant UAV Wi-Fi Beacon (Remote ID) using an ESP32-C3 SuperMini. It utilizes a custom MicroPython module `esp_wifi_ext` for raw 802.11 frame injection. (read .\docs\arrete_dispositif.txt)

### Hardware Configuration
- **Platform**: MicroPython v1.28 with `esp_wifi_ext`(.\docs\esp_wifi_ext.md)
- **MCU**: ESP32-C3 SuperMini V2
- **GPS**: BZ-121 (connected to UART1)
- **LED**: WS2812B Neopixel on GPIO 8
- **Wiring**:
    - GPS TX -> GPIO 3 (UART1 RX) PASSTROUGH TO FLIGHTCOMONTROLLER (FC)
    - GPS RX -> GPIO 4 (UART1 TX)
    - FC RX  <- (UART1 RX - GPIO 3 
    PASSTROUGH)
    - FC TX  -> (UART1-TX - PASSTROUGH)

### Technical Specifications

Platform : Micropython v1.28 with special usermod (esp_wifi_ext .\docs\esp_wifi_ext.md)

Module : esp32-c3 supermini v2

gps module : BZ-121

The module must collect gps data of a UAV and transmit that data over a wifi beacon on accordance to french law (see document .\docs\arrete_dispositif.txt)

The GPS module is connected to UART1 RX  =  gpio pin3; tx = gpio pin4

The output of the module is connected UART0 rx = gpio pin 20; tx = gpio pi 21

The ouput is supposed to go to a flightcontroller and is a passtrough of the UART1.

At the boot of the module the software must check the baudrate of uart1-rx.

Then it should set the in and out uart  of the gps-module to UBX protocol only.

Then it should put the baudrate of the gps-module to 57600 baud and the rate to 10hz

Only the UBX Nav_PVT messages must be used

if after 5 sec there are less then 10 characters received   the status must be "No gps module"

Else if numSV = 0 the status is "NO satelites found"

when numSV <  5  or  pdoP >= 2.0 then status  "home position not determinated"

Else set home_latitude, home_longitude , home_altitude home_position_set = True

As sone as the home position is determinated the module must sent the beacon mangement message as described in .\docs\arrete_dispositif.rtf, at least every 3sec or when the UAV travelled more then 30 meters sinc e the last message

The SSID of the Module must be "RJA_"+ lsb 3 bytes of the MAC addresse of the module in HEX format

the UAV-ID must be "RJAB08000000000000"+ the 6 bytes of the MAC addresse of the module in HEX format

On gpio pin 8 is a neopixel led (WS2812b) that should blink like this:

| Status                           | Frequency | Duty cyle | Color on | Color off |
| -------------------------------- | --------- | --------- | -------- | --------- |
| "No gps module"                  | 100hz     | 50%       | red      | black     |
| "NO satelites found"             | 500hz     | 50%       | blue     | black     |
| "home position not determinated" | 1000hz    | 50%       | blue     | red       |
| home_position_set = True         | 3000hz    | 25%       | green    | purple    |











