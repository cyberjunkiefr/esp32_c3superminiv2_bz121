# esp_wifi_ext - MicroPython Wi-Fi Extension Module

This module provides low-level Wi-Fi capabilities to MicroPython on ESP32, including:
- Promiscuous mode (sniffing)
- Packet filtering
- Raw 802.11 packet injection

## Features

- **Safe Sniffing**: Uses a FreeRTOS queue to safely pass packets from the Wi-Fi stack to the MicroPython thread.
- **Filtering**: Support for Management and Data frame filters.
- **Injection**: `tx_80211` function for sending raw frames.

## How to Build

To include this module in your MicroPython firmware, you need to specify the path to its `micropython.cmake` file during the build process.

### Prerequisites

- ESP-IDF installed and configured.
- MicroPython source code cloned.

### Build Commands

Replace `~/micropython` with your actual MicroPython path.

#### For ESP32-C3:
```bash
cd ~/micropython/ports/esp32/
idf.py set-target esp32c3
idf.py -DUSER_C_MODULES="/home/cyberjunkie/projects/esp-idf/usermods/esp_wifi_ext/micropython.cmake" \
       -DBOARD=ESP32_GENERIC_C3 \
       reconfigure
idf.py build
```

#### For ESP32-S3:
```bash
cd ~/micropython/ports/esp32/
idf.py set-target esp32s3
idf.py -DUSER_C_MODULES="/home/cyberjunkie/projects/esp-idf/usermods/esp_wifi_ext/micropython.cmake" \
       -DBOARD=ESP32_GENERIC_S3 \
       reconfigure
idf.py build
```

## How to Use

### Basic Sniffing Example

```python
import esp_wifi_ext
import network
import time

# 1. Activate Wi-Fi interface
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# 2. Initialize the internal packet queue
esp_wifi_ext.init_queue()

# 3. Define a callback function
def my_callback(buf, rssi, channel, frame_type, subtype):
    print(f"RX: type={frame_type} sub={subtype} rssi={rssi} len={len(buf)}")

esp_wifi_ext.set_rx_callback(my_callback)

# 4. Set filters (Management and/or Data)
esp_wifi_ext.set_filter(esp_wifi_ext.FILTER_MGMT | esp_wifi_ext.FILTER_DATA)

# 5. Enable promiscuous mode
esp_wifi_ext.promiscuous(True)

# 6. Poll for packets in your main loop
print("Sniffing started...")
try:
    while True:
        esp_wifi_ext.poll_queue()
        time.sleep_ms(1)
except KeyboardInterrupt:
    esp_wifi_ext.promiscuous(False)
    print("Stopped.")
```

### Raw Packet Injection

```python
# Send a raw 802.11 frame
# The interface (STA/AP) is automatically selected based on what is active.
packet = b'\x80\x00\x00\x00\xff\xff\xff\xff\xff\xff...' # Example beacon frame
esp_wifi_ext.tx_80211(packet)
```

## API Reference

- `promiscuous(enable: bool)`: Enable or disable promiscuous mode.
- `set_rx_callback(cb)`: Set the function to be called for each received packet. Callback args: `(buffer, rssi, channel, type, subtype)`.
- `set_filter(mask)`: Set the promiscuous filter mask (`FILTER_MGMT`, `FILTER_DATA`).
- `set_channel(channel)`: Set the Wi-Fi channel (1-14).
- `init_queue()`: Initialize the internal FreeRTOS queue for packets.
- `poll_queue()`: Process the queue and trigger callbacks in the MicroPython thread.
- `tx_80211(buffer)`: Transmit a raw 802.11 packet.
