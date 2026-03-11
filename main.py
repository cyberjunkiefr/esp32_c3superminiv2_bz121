# main.py - French Remote ID beacon for ESP32-C3 SuperMini
# Implements GPS passthrough + UBX NAV-PVT parsing + 802.11 beacon injection

import time
import struct
import math
import network
import machine
import sys
import _thread
import binascii

try:
    import select
except ImportError:
    select = None

try:
    import neopixel
    print("Neopixel module available, LED support enabled")
except ImportError:
    print("Neopixel module not available, LED support disabled")
    neopixel = None

try:
    import esp_wifi_ext
    print("esp_wifi_ext module available, beacon transmission enabled")
except ImportError:
    print("esp_wifi_ext module not available, beacon transmission disabled")    
    esp_wifi_ext = None

# --- Configuration ---------------------------------------------------------
GPS_UART_ID = 1
GPS_RX_PIN = 3
GPS_TX_PIN = 4

FC_UART_ID = 0
FC_RX_PIN = 20
FC_TX_PIN = 21

# Beacon requirements
BEACON_CHANNEL = 6
BEACON_MIN_PERIOD_MS = 3000
BEACON_MIN_DISTANCE_M = 30

# Logging configuration (prints to USB REPL)
LOG_LEVEL_DEBUG = 10
LOG_LEVEL_INFO = 20
LOG_LEVEL_WARN = 30

# Current log level (set to LOG_LEVEL_INFO or LOG_LEVEL_WARN to reduce verbosity)
LOG_LEVEL = LOG_LEVEL_DEBUG

# Runtime state (for interactive commands)
CURRENT_STATUS = None
CURRENT_NAV = None
HOME_SET = False
HOME_LAT = 0.0
HOME_LON = 0.0
HOME_ALT = 0.0

# Status enums
STATUS_NO_GPS = "No gps module"
STATUS_NO_SATS = "NO satelites found"
STATUS_NO_HOME = "home position not determinated"
STATUS_HOME_SET = "home position set"

# LED (Neopixel) configuration
LED_PIN = 8
LED_COUNT = 1

# TLV types (from Arrête 27/12/2019)
TLV_PROTOCOL_VERSION = 0x01
TLV_IDENT_FR = 0x02
TLV_LAT = 0x04
TLV_LON = 0x05
TLV_ALT = 0x06
TLV_TAKEOFF_LAT = 0x08
TLV_TAKEOFF_LON = 0x09
TLV_HEIGHT = 0x07
TLV_SPEED = 0x0A
TLV_HEADING = 0x0B

# Vendor Specific IE (802.11) values
OUI = b"\x6A\x5C\x35"
VS_TYPE = b"\x01"

# ---------------------------------------------------------------------------

def _log(level: int, *args, **kwargs):
    if level < LOG_LEVEL:
        return
    ts = time.ticks_ms()
    level_str = {
        LOG_LEVEL_DEBUG: "DEBUG",
        LOG_LEVEL_INFO: "INFO",
        LOG_LEVEL_WARN: "WARN",
    }.get(level, str(level))
    print("[%d][%s]" % (ts, level_str), *args, **kwargs)


def _debug(*args, **kwargs):
    _log(LOG_LEVEL_DEBUG, *args, **kwargs)


def _info(*args, **kwargs):
    _log(LOG_LEVEL_INFO, *args, **kwargs)


def _warn(*args, **kwargs):
    _log(LOG_LEVEL_WARN, *args, **kwargs)


def _set_log_level(level: int):
    global LOG_LEVEL
    if level in (LOG_LEVEL_DEBUG, LOG_LEVEL_INFO, LOG_LEVEL_WARN):
        LOG_LEVEL = level
        _info("Log level set to", level)
        return True
    return False


def _process_console_commands():
    """Non-blocking read from REPL/USB for simple runtime commands."""
    if select is None:
        return
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return
        line = sys.stdin.readline().strip()
        if not line:
            return
        parts = line.split()
        if len(parts) == 0:
            return
        cmd = parts[0].lower()
        if cmd == "log" and len(parts) >= 2:
            lvl = parts[1].lower()
            if lvl == "debug":
                _set_log_level(LOG_LEVEL_DEBUG)
            elif lvl == "info":
                _set_log_level(LOG_LEVEL_INFO)
            elif lvl == "warn" or lvl == "warning":
                _set_log_level(LOG_LEVEL_WARN)
            else:
                _info("Unknown log level", lvl)
        elif cmd == "log" and len(parts) == 1:
            _info("Current log level", LOG_LEVEL)
        elif cmd in ("help", "?"):
            _info("Commands:")
            _info("  log [debug|info|warn]  - set log level")
            _info("  log                   - show current level")
            _info("  status                - show GPS + home status")
            _info("  beacon                - force a beacon transmit now")
        elif cmd == "status":
            _report_status()
        elif cmd == "beacon":
            _force_beacon()
        else:
            _info("Unknown command:", line)
    except Exception as e:
        # ignore if stdin isn't available or select isn't supported
        pass


def _report_status():
    _info("Status:", CURRENT_STATUS)
    #_info(f"Baudrate = {rate}")
    if CURRENT_NAV:                    
        _info("  numSV", CURRENT_NAV.get("numSV"), "pDOP", CURRENT_NAV.get("pDOP"))
        _info("  lat", CURRENT_NAV.get("lat"), "lon", CURRENT_NAV.get("lon"), "alt_msl", CURRENT_NAV.get("height_msl"))
        _info("  speed", CURRENT_NAV.get("gSpeed_m_s"), "heading", CURRENT_NAV.get("heading_deg"))
    _info("Home set:", HOME_SET, "home lat", HOME_LAT, "lon", HOME_LON, "alt", HOME_ALT)

def _force_beacon():
    if not HOME_SET or not CURRENT_NAV:
        _warn("Cannot force beacon: home not set or no nav data yet")
        return
    _info("Forcing beacon transmit")

    # Build payload and beacon frame using current nav + home
    wlan = network.WLAN(network.STA_IF)
    mac = wlan.config("mac")
    ssid = _format_ssid(mac)
    uav_id = _format_uav_id(mac)

    payload = _build_vendor_payload(
        protocol_version=1,
        id_fr=uav_id,
        lat=CURRENT_NAV.get("lat"),
        lon=CURRENT_NAV.get("lon"),
        alt_m=CURRENT_NAV.get("height_msl"),
        takeoff_lat=HOME_LAT,
        takeoff_lon=HOME_LON,
        takeoff_alt_m=HOME_ALT,
        speed_m_s=CURRENT_NAV.get("gSpeed_m_s"),
        heading_deg=CURRENT_NAV.get("heading_deg"),
    )

    pkt = _build_beacon_frame(ssid, payload, mac)
    try:
        if esp_wifi_ext:
            esp_wifi_ext.tx_80211(pkt)
    except Exception as e:
        _warn("Beacon tx error", e)


def _ubx_checksum(message: bytes) -> bytes:
    """Calculate UBX checksum for message (class+id+length+payload)."""
    ck_a = 0
    ck_b = 0
    for b in message:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def _ubx_packet(cls: int, msg_id: int, payload: bytes) -> bytes:
    hdr = b"\xB5\x62" + bytes((cls, msg_id)) + struct.pack("<H", len(payload))
    pkt = hdr + payload
    return pkt + _ubx_checksum(pkt[2:])


def _make_cfg_prt_uart1(baud: int) -> bytes:
    # CFG-PRT (0x06 0x00) for UART1
    # portID=1, mode=8N1, inProtoMask=1(UBX), outProtoMask=1(UBX)
    portID = 1
    reserved0 = 0
    txReady = 0
    mode = 0x000008D0  # 8N1, no parity
    inProtoMask = 0x0001
    outProtoMask = 0x0001
    flags = 0
    reserved5 = 0
    payload = struct.pack("<BBHIIHHHH",
                          portID,
                          reserved0,
                          txReady,
                          mode,
                          baud,
                          inProtoMask,
                          outProtoMask,
                          flags,
                          reserved5)
    return _ubx_packet(0x06, 0x00, payload)


def _make_cfg_rate(meas_rate_ms: int, nav_rate: int = 1, time_ref: int = 0) -> bytes:
    payload = struct.pack("<HHH", meas_rate_ms, nav_rate, time_ref)
    return _ubx_packet(0x06, 0x08, payload)


def _parse_nav_pvt(payload: bytes) -> dict | None:
    # Expect payload len 92C
    if len(payload) < 92:
        return None

    # See UBX-NAV-PVT (0x01 0x07)
    #   lon, lat in 1e-7 deg
    #   height in mm (above ellipsoid, height in msl)
    #   gSpeed in cm/s, heading in 1e-5 deg
    numSV = payload[23]
    lon = struct.unpack_from("<i", payload, 24)[0]
    lat = struct.unpack_from("<i", payload, 28)[0]
    height = struct.unpack_from("<i", payload, 32)[0]
    hMSL = struct.unpack_from("<i", payload, 36)[0]
    gSpeed = struct.unpack_from("<i", payload, 60)[0]
    heading = struct.unpack_from("<i", payload, 64)[0]
    pDOP = struct.unpack_from("<H", payload, 76)[0]

    return {
        "numSV": numSV,
        "pDOP": pDOP / 100.0,
        "lat": lat / 1e7,
        "lon": lon / 1e7,
        "height_msl": hMSL / 1000.0,
        "height_ellipsoid": height / 1000.0,
        "gSpeed_m_s": gSpeed / 100.0,
        "heading_deg": heading / 1e5,
    }


def _signed_int(value: int, size: int) -> int:
    """Encode signed integer to two's complement of size bytes."""
    # size in bytes
    bits = size * 8
    if value < 0:
        value = (1 << bits) + value
    return value & ((1 << bits) - 1)


def _tlv_uint(value: int, size: int) -> bytes:
    return _signed_int(int(value), size).to_bytes(size, "big")


def _coord_to_tlv(value_deg: float) -> bytes:
    """Convert degrees to fixed 5 decimal places and pack into 4 bytes signed."""
    v = int(round(value_deg * 1e5))
    return _tlv_uint(v, 4)


def _build_vendor_payload(protocol_version: int, id_fr: str, lat: float, lon: float, alt_m: float,
                           takeoff_lat: float, takeoff_lon: float, takeoff_alt_m: float,
                           speed_m_s: float, heading_deg: float) -> bytes:
    parts = []

    # Version
    parts.append(bytes((TLV_PROTOCOL_VERSION, 1, protocol_version)))

    # UAV ID (ID FR)
    id_bytes = id_fr.encode("utf-8")
    if len(id_bytes) != 30:
        # pad or truncate
        id_bytes = id_bytes[:30].ljust(30, b"0")
    parts.append(bytes((TLV_IDENT_FR, 30)) + id_bytes)

    # Current position
    parts.append(bytes((TLV_LAT, 4)) + _coord_to_tlv(lat))
    parts.append(bytes((TLV_LON, 4)) + _coord_to_tlv(lon))

    # Altitude (m) signed 2 bytes (MSL)
    parts.append(bytes((TLV_ALT, 2)) + _tlv_uint(round(alt_m), 2))

    # Height relative to takeoff (m) signed 2 bytes
    rel_height = round(alt_m - takeoff_alt_m)
    parts.append(bytes((TLV_HEIGHT, 2)) + _tlv_uint(rel_height, 2))

    # Takeoff position
    parts.append(bytes((TLV_TAKEOFF_LAT, 4)) + _coord_to_tlv(takeoff_lat))
    parts.append(bytes((TLV_TAKEOFF_LON, 4)) + _coord_to_tlv(takeoff_lon))

    # Speed (m/s) 1 byte
    speed_byte = max(0, min(255, int(round(speed_m_s))))
    parts.append(bytes((TLV_SPEED, 1)) + bytes((speed_byte,)))

    # Heading 2 bytes unsigned
    heading = int(round(heading_deg)) % 360
    parts.append(bytes((TLV_HEADING, 2)) + heading.to_bytes(2, "big"))

    return b"".join(parts)


def _build_beacon_frame(ssid: str, vendor_payload: bytes, src_mac: bytes) -> bytes:
    # Minimal 802.11 beacon
    frame_ctrl = b"\x80\x00"  # Beacon
    duration = b"\x00\x00"
    addr1 = b"\xff\xff\xff\xff\xff\xff"  # broadcast
    addr2 = src_mac
    addr3 = src_mac
    seq_ctrl = b"\x00\x00"

    # Fixed parameters
    timestamp = b"\x00" * 8
    beacon_interval = struct.pack("<H", 0x0064)  # 100 TU
    cap_info = struct.pack("<H", 0x0431)

    # SSID IE
    ssid_bytes = ssid.encode("utf-8")
    ssid_ie = b"\x00" + bytes((len(ssid_bytes),)) + ssid_bytes

    # Vendor Specific IE
    vs_payload = OUI + VS_TYPE + vendor_payload
    vs_ie = b"\xDD" + bytes((len(vs_payload),)) + vs_payload

    body = timestamp + beacon_interval + cap_info + ssid_ie + vs_ie

    return frame_ctrl + duration + addr1 + addr2 + addr3 + seq_ctrl + body


def _haversine(lat1, lon1, lat2, lon2):
    # meters
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


class LED:
    def __init__(self):
        if neopixel is None:
            print("neopixel is None:")
            self._np = None
            return
        pin = machine.Pin(LED_PIN, machine.Pin.OUT)
        self._np = neopixel.NeoPixel(pin, LED_COUNT)
        self._period = 100              # In ms 
        self._dutycycle = 50            # In % of the period
        self._color_on = (255, 0, 0)    # Red
        self._color_off = (0, 0, 0)     # Off

    def blink(self):
        if self._np is None:
            print("LED not available (neopixel module missing)")
            return
        try:
            while True:
                self._on_ms = self._period * self._dutycycle // 100
                self._off_ms = self._period - self._on_ms                               
                self._np[0] = self._color_on
                self._np.write()
                time.sleep_ms(self._on_ms)
                self._np[0] = self._color_off
                self._np.write()
                time.sleep_ms(self._off_ms)
        except Exception as error:
            print("ERROR %s\n%s" % (error,err))

    def update(self, period_ms: int, dutycycle: int, color_on, color_off):
            if self._np is None:
                print("LED not available (neopixel module missing)")    
                return
            self._period = period_ms
            self._dutycycle = dutycycle
            self._color_on = color_on
            self._color_off = color_off

def _format_ssid(mac: bytes) -> str:
    # SSID: "RJA_" + lsb 3 bytes of MAC in hex uppercase
    return "RJA_" + "".join(f"{b:02X}" for b in mac[-3:])


def _format_uav_id(mac: bytes) -> str:
    # UAV-ID: "RJAB08000000000000" + 6 bytes of MAC
    return "RJAB08000000000000" + "".join(f"{b:02X}" for b in mac)


def _setup_wifi():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)
    # prefer channel 6 as per spec
    try:
        esp_wifi_ext.set_channel(BEACON_CHANNEL)
    except Exception:
        pass
    return wlan

def _detect_gps_baud(gps_uart) -> int:
    # Try common baud rates and detect UBX sync bytes
    rates = [9600, 19200, 38400, 57600, 115200, 230400, 46000]
    deadline = time.ticks_add(time.ticks_ms(), 5000)
    while time.ticks_ms()-deadline <= 0:
        for rate in rates:
            gps_uart.init(baudrate=rate, rx=GPS_RX_PIN, tx=GPS_TX_PIN, timeout=50)
            gps_uart.read()  # flush
            t0 = time.ticks_ms() + 300
            #seen = 0
            _debug("Selected Baudrate = ", rate)
            while time.ticks_ms()-t0 <= 0:
                data = gps_uart.read()
                if not data:
                    continue
                if b"\xb5\x62" in data:
                    return rate
                '''
                seen += len(data)
                if seen >= 10:
                    return rate
                '''    
    return None


def _send_ubx(gps_uart, packet: bytes):
    # Debug: show what is being sent to GPS module
    if len(packet) >= 6:
        cls = packet[2]
        msg_id = packet[3]
        length = packet[4] | (packet[5] << 8)
        _debug("UBX -> cls=0x%02X id=0x%02X len=%d" % (cls, msg_id, length))
    gps_uart.write(packet)


def _parse_ubx_stream(stream: bytearray) -> tuple[list[dict], bytearray]:
    """Parse UBX frames from stream, return list of parsed msgs and leftover buffer."""
    msgs = []
    i = 0
    while i + 6 <= len(stream):
        if stream[i] != 0xB5 or stream[i + 1] != 0x62:
            i += 1
            continue
        if i + 6 > len(stream):
            break
        length = stream[i + 4] | (stream[i + 5] << 8)
        if i + 6 + length + 2 > len(stream):
            break
        msg = bytes(stream[i:i + 6 + length + 2])
        # verify checksum
        ck = _ubx_checksum(msg[2:-2])
        if ck == msg[-2:]:
            msgs.append({
                "class": msg[2],
                "id": msg[3],
                "payload": msg[6:-2],
            })
            i += 6 + length + 2
        else:
            i += 1
    return msgs, stream[i:]


def main():
    global CURRENT_STATUS, CURRENT_NAV, HOME_SET, HOME_LAT, HOME_LON, HOME_ALT

    led = LED()
    _thread.start_new_thread(led.blink, ())

    status = STATUS_NO_GPS
    CURRENT_STATUS = status

    wlan = _setup_wifi()
    mac = wlan.config("mac")
    ssid = _format_ssid(mac)
    uav_id = _format_uav_id(mac)
    _info("MAC", ":".join(f"{b:02X}" for b in mac))
    _info("SSID", ssid)
    _info("UAV-ID", uav_id)

    gps_uart = machine.UART(GPS_UART_ID, baudrate=9600, rx=GPS_RX_PIN, tx=GPS_TX_PIN, timeout=50)
    fc_uart = machine.UART(FC_UART_ID, baudrate=57600, rx=FC_RX_PIN, tx=FC_TX_PIN, timeout=50)

    # Detect baud rate and set protocol + rate
    detected = _detect_gps_baud(gps_uart)
    if detected is None:
        status = STATUS_NO_GPS
    else:
        # make sure GPS is in UBX-only, 57600 baud, 10Hz
        cfg_prt = _make_cfg_prt_uart1(57600)
        cfg_rate = _make_cfg_rate(100)
        _send_ubx(gps_uart, cfg_prt)
        time.sleep_ms(200)
        _debug("CFG_RATE",":".join(f"{b:02x}" for b in cfg_rate))
        _send_ubx(gps_uart, cfg_rate)
        time.sleep_ms(200)
        gps_uart.init(baudrate=57600, rx=GPS_RX_PIN, tx=GPS_TX_PIN, timeout=50)

    buf = bytearray()
    home_set = False
    home_lat = 0.0
    home_lon = 0.0
    home_alt = 0.0

    last_beacon_time = time.ticks_ms()
    last_beacon_lat = 0.0
    last_beacon_lon = 0.0

    last_status = None

    while True:

        # Allow changing log level via USB REPL commands
        _process_console_commands()

        # Pass-through between GPS and FC
        try:
            data = gps_uart.read()
            if data:
                fc_uart.write(data)
                buf.extend(data)
        except Exception:
            pass

        # Parse potential UBX frames
        msgs, buf = _parse_ubx_stream(buf)
        for msg in msgs:
            if msg["class"] == 0x01 and msg["id"] == 0x07:
                nav = _parse_nav_pvt(msg["payload"])
                if nav is None:
                    continue

                # Determine status
                if nav["numSV"] <= 0:
                    status = STATUS_NO_SATS
                elif nav["numSV"] < 5 or nav["pDOP"] >= 12.0:
                    status = STATUS_NO_HOME
                else:
                    if not home_set:
                        home_set = True
                        home_lat = nav["lat"]
                        home_lon = nav["lon"]
                        home_alt = nav["height_msl"]
                        _debug(f"home_lat = {home_lat}\nhome_lon = {home_lon}\nhome_alt = {home_alt}")
                    status = STATUS_HOME_SET

                # Update global runtime state (for REPL commands)
                CURRENT_STATUS = status
                CURRENT_NAV = nav
                HOME_SET = home_set
                HOME_LAT = home_lat
                HOME_LON = home_lon
                HOME_ALT = home_alt

                if status != last_status:
                    _info("Status ->", status)
                    last_status = status

                # send beacon when required
                now = time.ticks_ms()
                dist = _haversine(last_beacon_lat, last_beacon_lon, nav["lat"], nav["lon"]) if last_beacon_lat else 9999
                if home_set and (time.ticks_diff(now, last_beacon_time) >= BEACON_MIN_PERIOD_MS or dist >= BEACON_MIN_DISTANCE_M):
                    payload = _build_vendor_payload(
                        protocol_version=1,
                        id_fr=uav_id,
                        lat=nav["lat"],
                        lon=nav["lon"],
                        alt_m=nav["height_msl"],
                        takeoff_lat=home_lat,
                        takeoff_lon=home_lon,
                        takeoff_alt_m=home_alt,
                        speed_m_s=nav["gSpeed_m_s"],
                        heading_deg=nav["heading_deg"],
                    )
                    pkt = _build_beacon_frame(ssid, payload, mac)
                    _debug(f"Beacon tx: len={len(pkt)} chan={BEACON_CHANNEL}\nLatitude={nav["lat"]}\nLongitude={nav["lon"]}\nAltitude={nav["height_msl"]}")
                    try:
                        esp_wifi_ext.tx_80211(pkt)
                    except Exception as e:
                        _warn("Beacon tx error", e)
                    last_beacon_time = now
                    last_beacon_lat = nav["lat"]
                    last_beacon_lon = nav["lon"]

        # Update LED pattern
        if status == STATUS_NO_GPS:
            led.update(period_ms=100, dutycycle=50, color_on=(255, 0, 0), color_off=(0, 0, 0))
        elif status == STATUS_NO_SATS:
            led.update(period_ms=500, dutycycle=50, color_on=(0, 0, 255), color_off=(0, 0, 0))
        elif status == STATUS_NO_HOME:
            led.update(period_ms=1000, dutycycle=50, color_on=(0, 0, 255), color_off=(255, 0, 0))
        else:
            led.update(period_ms=3000, dutycycle=25, color_on=(0, 255, 0), color_off=(128, 0, 128))

        # minimal delay
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
