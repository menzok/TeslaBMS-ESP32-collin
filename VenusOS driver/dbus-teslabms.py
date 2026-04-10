#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ─────────────────────────────────────────────────────────────────────────────
#  dbus-teslabms.py  —  Standalone Venus OS battery driver for TeslaBMS-ESP32
#  Version 1.4.0
#
#  Protocol change: the ESP32 is now purely request/reply — it only transmits
#  in response to a CMD 0x03 command and never sends unsolicited packets.
#  The Pi drives all data requests via a periodic poll every POLL_INTERVAL_S.
#
#  Frame format (29 bytes total)
#  ──────────────────────────────
#  [0]      0xAA           start byte
#  [1..26]  payload        26-byte payload (see map below)
#  [27..28] CRC16/MODBUS   little-endian, computed over payload bytes only
#
#  Payload map (all multi-byte fields big-endian)
#  ──────────────────────────────────────────────
#  [0-1]   packV               uint16  × 100   → V   (10 mV res)
#  [2-3]   packI               int16   × 10    → A   (100 mA res, + = charge)
#  [4]     soc                 uint8   0-100   %
#  [5]     temp                int8    °C      (signed)
#  [6-7]   power               int16   W
#  [8-9]   avgCell             uint16  × 100   → V   (10 mV res)
#  [10-11] alarmFlags          uint16  bitmask (bits 5-15 reserved/unimplemented)
#  [12]    overlordState       uint8   0=Normal 1=Fault 2=StorageMode
#  [13]    contactorState      uint8   raw enum
#  [14-15] overVoltage         uint16  × 1000  → V   (1 mV res)
#  [16-17] underVoltage        uint16  × 1000  → V   (1 mV res)
#  [18]    overTemp            int8    °C      (signed)
#  [19]    underTemp           int8    °C      (signed)
#  [20]    numModules          uint8   total modules detected
#  [21]    numStrings          uint8   parallel string count
#  [22-23] overCurrentThresh   uint16  × 10    → A   (0.1A res)
#  [24]    reserved            0x00
#  [25]    reserved            0x00
#
#  Venus OS user-configurable settings (com.victronenergy.settings)
#  ────────────────────────────────────────────────────────────────
#  /Settings/TeslaBMS/MaxChargeCurrent       default = 250A
#  /Settings/TeslaBMS/MaxDischargeCurrent    default = 250A
#  /Settings/TeslaBMS/AbsorptionVoltage      default = 4.15V per cell
#  /Settings/TeslaBMS/FloatVoltage           default = 4.10V per cell
#  /Settings/TeslaBMS/TailCurrent            default = 10A
#  /Settings/TeslaBMS/MaxChargeTemp          default = 45°C
#  /Settings/TeslaBMS/MinChargeTemp          default = 5°C
#
#  The overcurrent threshold from ESP32 EEPROM is used as the hard cap
#  on CCL/DCL — user settings cannot exceed it.
#
# ─────────────────────────────────────────────────────────────────────────────

import os
import sys
import signal
import time
import threading
import struct
import serial
import glob
import logging

from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import dbus

# ── velib_python ──────────────────────────────────────────────────────────────
VELIB_PATH = "/data/apps/dbus-serialbattery/ext/velib_python"
sys.path.insert(1, VELIB_PATH)
from vedbus import VeDbusService           # noqa: E402
from settingsdevice import SettingsDevice  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
#  Constants — Venus OS owns these, not the ESP32
# ─────────────────────────────────────────────────────────────────────────────

AH_PER_MODULE = 232.0   # Ah per Tesla module (6S 74P, 232 Ah)

# Hard cap — user settings are clamped to this regardless of what they enter.
# The ESP32 EEPROM overcurrent threshold is also enforced as an upper bound.
CURRENT_HARD_CAP = 500.0   # A

# Settings defaults — healthy starting points for a Tesla Model 3 pack
DEFAULT_MAX_CHARGE_CURRENT    = 250.0   # A
DEFAULT_MAX_DISCHARGE_CURRENT = 250.0   # A
DEFAULT_ABSORPTION_VOLTAGE    = 4.15    # V per cell
DEFAULT_FLOAT_VOLTAGE         = 4.10    # V per cell
DEFAULT_TAIL_CURRENT          = 10.0    # A  (end-of-charge detection)
DEFAULT_MAX_CHARGE_TEMP       = 45.0    # °C (charge inhibit above)
DEFAULT_MIN_CHARGE_TEMP       = 5.0     # °C (charge inhibit below)

# Timing
SERIAL_TIMEOUT_S    = 3.5    # must exceed worst-case ESP32 response latency (1 s loop + Overlord.update() duration)
POLL_INTERVAL_S     = 2.0    # how often Pi sends CMD 0x03 to request data
CMD_RETRIES         = 3      # number of attempts before declaring a command failed
CMD_RETRY_DELAY_S   = 1.0    # pause between retries — lets the ESP32 drain any converter noise before the next command arrives
STALE_TIMEOUT       = 5.0    # flag data as stale if no frame received within this window
OFFLINE_TIMEOUT     = 15.0   # flag as offline/cable-fault after this long with no frame
BAUD_RATE           = 115200
PUBLISH_INTERVAL_MS = 1000

DBUS_SERVICE_NAME = "com.victronenergy.battery.teslabms"
SETTINGS_SERVICE  = "com.victronenergy.settings"
SETTINGS_BASE     = "/Settings/TeslaBMS"

# ─────────────────────────────────────────────────────────────────────────────
#  Protocol constants
# ─────────────────────────────────────────────────────────────────────────────

EXT_CMD_SHUTDOWN  = 0x01
EXT_CMD_STARTUP   = 0x02
EXT_CMD_SEND_DATA = 0x03

FRAME_START_BYTE  = 0xAA
PAYLOAD_LEN       = 26
FRAME_LEN         = 1 + PAYLOAD_LEN + 2   # = 29

ALARM_OVER_VOLTAGE  = (1 << 0)
ALARM_UNDER_VOLTAGE = (1 << 1)
ALARM_OVER_TEMP     = (1 << 2)
ALARM_UNDER_TEMP    = (1 << 3)
ALARM_OVER_CURRENT  = (1 << 4)
# bits 5-15: reserved, not yet implemented in ESP32 firmware

ALARM_OK      = 0
ALARM_WARNING = 1
ALARM_ALARM   = 2

OVERLORD_NORMAL  = 0
OVERLORD_FAULT   = 1
OVERLORD_STORAGE = 2

# ─────────────────────────────────────────────────────────────────────────────
#  Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("teslabms")

# ─────────────────────────────────────────────────────────────────────────────
#  CRC-16 / MODBUS
# ─────────────────────────────────────────────────────────────────────────────

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc

# ─────────────────────────────────────────────────────────────────────────────
#  VenusSettings — user-configurable charge parameters
# ─────────────────────────────────────────────────────────────────────────────

class VenusSettings:
    """
    Persists user-configurable charge/discharge settings in
    com.victronenergy.settings under /Settings/TeslaBMS/.

    These settings survive reboots and firmware updates.
    They are the user's installation-specific limits and charge targets.

    The ESP32 EEPROM overcurrent threshold is enforced as a hard upper
    bound on MaxChargeCurrent and MaxDischargeCurrent — the user cannot
    set values above what the contactor is rated for.
    """

    _BASE = SETTINGS_BASE

    def __init__(self, bus):
        self._bus = bus
        self._sd  = None
        self._cache = {
            "MaxChargeCurrent":     DEFAULT_MAX_CHARGE_CURRENT,
            "MaxDischargeCurrent":  DEFAULT_MAX_DISCHARGE_CURRENT,
            "AbsorptionVoltage":    DEFAULT_ABSORPTION_VOLTAGE,
            "FloatVoltage":         DEFAULT_FLOAT_VOLTAGE,
            "TailCurrent":          DEFAULT_TAIL_CURRENT,
            "MaxChargeTemp":        DEFAULT_MAX_CHARGE_TEMP,
            "MinChargeTemp":        DEFAULT_MIN_CHARGE_TEMP,
        }

    def setup(self) -> None:
        """Register settings with localsettings. Call once at startup."""
        settings_map = {
            # [dbus_path, default, min, max]
            #
            # Current limits — clamped at runtime to ESP32 overcurrent threshold
            "MaxChargeCurrent": [
                f"{self._BASE}/MaxChargeCurrent",
                DEFAULT_MAX_CHARGE_CURRENT,
                1.0,
                CURRENT_HARD_CAP,
            ],
            "MaxDischargeCurrent": [
                f"{self._BASE}/MaxDischargeCurrent",
                DEFAULT_MAX_DISCHARGE_CURRENT,
                1.0,
                CURRENT_HARD_CAP,
            ],
            # Per-cell voltage targets for the Victron charge algorithm
            # Absorption: held at this voltage until tail current is reached
            "AbsorptionVoltage": [
                f"{self._BASE}/AbsorptionVoltage",
                DEFAULT_ABSORPTION_VOLTAGE,
                3.50,   # V per cell — never go below this for absorption
                4.25,   # V per cell — hard upper limit
            ],
            # Float: maintained after absorption complete
            "FloatVoltage": [
                f"{self._BASE}/FloatVoltage",
                DEFAULT_FLOAT_VOLTAGE,
                3.20,   # V per cell
                4.20,   # V per cell
            ],
            # Tail current: charge considered complete when current drops below this
            "TailCurrent": [
                f"{self._BASE}/TailCurrent",
                DEFAULT_TAIL_CURRENT,
                0.5,    # A
                50.0,   # A
            ],
            # Temperature window for charging
            "MaxChargeTemp": [
                f"{self._BASE}/MaxChargeTemp",
                DEFAULT_MAX_CHARGE_TEMP,
                20.0,   # °C
                60.0,   # °C
            ],
            "MinChargeTemp": [
                f"{self._BASE}/MinChargeTemp",
                DEFAULT_MIN_CHARGE_TEMP,
                -10.0,  # °C
                20.0,   # °C
            ],
        }

        self._sd = SettingsDevice(
            bus=self._bus,
            supportedSettings=settings_map,
            eventCallback=self._on_setting_changed,
            name=SETTINGS_SERVICE,
            timeout=10,
        )

        # Seed cache from whatever localsettings currently holds
        for key in self._cache:
            try:
                self._cache[key] = self._sd[key]
            except Exception:
                pass

        log.info(f"VenusSettings ready: {self._cache}")

    def _on_setting_changed(self, setting: str, old_value, new_value) -> None:
        log.info(f"Setting '{setting}' changed: {old_value} → {new_value}")
        self._cache[setting] = new_value

    # ─── Read accessors ───────────────────────────────────────────────────────

    @property
    def max_charge_current(self) -> float:
        return float(self._cache["MaxChargeCurrent"])

    @property
    def max_discharge_current(self) -> float:
        return float(self._cache["MaxDischargeCurrent"])

    @property
    def absorption_voltage(self) -> float:
        return float(self._cache["AbsorptionVoltage"])

    @property
    def float_voltage(self) -> float:
        return float(self._cache["FloatVoltage"])

    @property
    def tail_current(self) -> float:
        return float(self._cache["TailCurrent"])

    @property
    def max_charge_temp(self) -> float:
        return float(self._cache["MaxChargeTemp"])

    @property
    def min_charge_temp(self) -> float:
        return float(self._cache["MinChargeTemp"])

    def effective_max_charge_current(self, overcurrent_threshold: float) -> float:
        """
        Returns the effective CCL — the lower of:
          - User-configured MaxChargeCurrent
          - ESP32 EEPROM overcurrent threshold (contactor rating)
        Ensures the user can never accidentally set a limit above the
        hardware safety threshold.
        """
        return min(self.max_charge_current, overcurrent_threshold)

    def effective_max_discharge_current(self, overcurrent_threshold: float) -> float:
        """Same enforcement for DCL."""
        return min(self.max_discharge_current, overcurrent_threshold)

    def pack_absorption_voltage(self, cell_count: int) -> float:
        """Absorption CVL for the full pack."""
        return round(self.absorption_voltage * cell_count, 2)

    def pack_float_voltage(self, cell_count: int) -> float:
        """Float CVL for the full pack."""
        return round(self.float_voltage * cell_count, 2)


# ─────────────────────────────────────────────────────────────────────────────
#  TeslaBMSSerial — UART handler + decoded state (background thread)
# ─────────────────────────────────────────────────────────────────────────────

class TeslaBMSSerial:
    """
    Background daemon thread that:
      1. Auto-detects and opens the ESP32 serial port.
      2. Polls the ESP32 for data every POLL_INTERVAL_S seconds via CMD 0x03.
      3. Validates CRC-16/MODBUS on all received frames.
      4. Decodes all 26 payload bytes into public attributes.

    The ESP32 is purely reply-driven — it only transmits in response to a command.
    send_command() retries up to CMD_RETRIES times before disconnecting.
    """

    def __init__(self):
        # ── Live telemetry ────────────────────────────────────────────────────
        self.voltage         = 0.0
        self.current         = 0.0
        self.soc             = 50
        self.temperature     = 25.0
        self.power           = 0
        self.avg_cell_volt   = 3.30
        self.alarm_flags     = 0
        self.overlord_state  = 0
        self.contactor_state = 0

        # ── EEPROM config (updated every frame) ───────────────────────────────
        self.eep_overvoltage        = 4.25
        self.eep_undervoltage       = 2.90
        self.eep_overtemp           = 60.0
        self.eep_undertemp          = -10.0
        self.eep_num_modules        = 2
        self.eep_num_strings        = 1
        self.eep_overcurrent_thresh = 350.0   # A — from EEPROM OVERCURRENT_THRESHOLD_A

        # ── Derived ───────────────────────────────────────────────────────────
        self.cell_count  = 6
        self.capacity_ah = 232.0

        # ── Connection ────────────────────────────────────────────────────────
        self.connected        = False
        self.last_frame_time  = 0.0
        self.frame_count      = 0

        # ── Internal ──────────────────────────────────────────────────────────
        self._ser                  = None
        self._port                 = None
        self._lock                 = threading.Lock()
        self._stop_event           = threading.Event()
        self._thread               = None
        self._last_connect_attempt = 0.0
        self._last_poll_time       = 0.0

    # ���── Derived pack limits ──────────────────────────────────────────────────

    @property
    def pack_overvoltage(self) -> float:
        return round(self.eep_overvoltage * self.cell_count, 2)

    @property
    def pack_undervoltage(self) -> float:
        return round(self.eep_undervoltage * self.cell_count, 2)

    @property
    def charge_fet(self) -> bool:
        return self.overlord_state != OVERLORD_FAULT

    @property
    def discharge_fet(self) -> bool:
        return self.overlord_state != OVERLORD_FAULT

    @property
    def is_stale(self) -> bool:
        if self.last_frame_time == 0:
            return True
        return (time.time() - self.last_frame_time) > OFFLINE_TIMEOUT

    def alarm_level(self, bit: int) -> int:
        return ALARM_ALARM if (self.alarm_flags & bit) else ALARM_OK

    # ─── Port detection ───────────────────────────────────────────────────────

    def _probe_port(self, port: str) -> bool:
        try:
            # dsrdtr=False / rtscts=False: keep DTR and RTS de-asserted so the
            # USB-UART bridge does not trigger the ESP32 auto-reset circuit.
            ser = serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT_S,
                                dsrdtr=False, rtscts=False)
            ser.dtr = False   # explicit: do not pull EN low via auto-reset RC
            ser.rts = False
            log.info(f"Probing {port} …")
            # Brief settle so line stabilises after open without a device reset
            time.sleep(0.1)
            cmd   = bytes([EXT_CMD_SEND_DATA])
            frame = bytes([FRAME_START_BYTE]) + cmd + struct.pack("<H", crc16_modbus(cmd))
            for attempt in range(1, CMD_RETRIES + 1):
                ser.reset_input_buffer()          # flush stale bytes before each send
                ser.write(frame)
                time.sleep(2.5)                   # wait for ESP32 loop tick + reply margin
                data = ser.read(FRAME_LEN)
                if len(data) >= FRAME_LEN and data[0] == FRAME_START_BYTE:
                    payload = data[1: 1 + PAYLOAD_LEN]
                    crc_rx  = struct.unpack("<H", data[1 + PAYLOAD_LEN: FRAME_LEN])[0]
                    if crc16_modbus(payload) == crc_rx:
                        ser.close()
                        log.info(f"✅ TeslaBMS-ESP32 confirmed on {port}")
                        return True
                log.debug(f"  {port}: no valid reply (attempt {attempt}/{CMD_RETRIES})")
            ser.close()
        except Exception as exc:
            log.debug(f"  {port}: {exc}")
        return False

    def _find_port(self) -> str | None:
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if not candidates:
            log.warning("No USB serial ports found.")
            return None
        for port in candidates:
            if self._probe_port(port):
                return port
        log.info("No TeslaBMS-ESP32 found.")
        return None

    # ─── Connection ───────────────────────────────────────────────────────────

    def _connect(self) -> bool:
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
            now = time.time()
            if now - self._last_connect_attempt < 5.0:
                return False
            self._last_connect_attempt = now

        port = self._find_port()
        if not port:
            return False

        try:
            with self._lock:
                self._ser = serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT_S,
                                          dsrdtr=False, rtscts=False)
                self._ser.dtr = False   # keep DTR low — do not trigger ESP32 auto-reset
                self._ser.rts = False
                self._port     = port
                self.connected = True
            log.info(f"✅ Connected on {port}")
            return True
        except Exception as exc:
            log.error(f"Failed to open {port}: {exc}")
            return False

    def _disconnect(self) -> None:
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser      = None
                self._port     = None
                self.connected = False

    # ─── Command send ─────────────────────────────────────────────────────────

    def send_command(self, cmd: int) -> bool:
        """Transmit a command frame and wait for a reply. Retries up to CMD_RETRIES times."""
        cmd_byte  = bytes([cmd])
        frame_out = (bytes([FRAME_START_BYTE])
                     + cmd_byte
                     + struct.pack("<H", crc16_modbus(cmd_byte)))

        for attempt in range(1, CMD_RETRIES + 1):
            with self._lock:
                if not self._ser or not self._ser.is_open:
                    log.warning(f"send_command(0x{cmd:02X}): not connected")
                    return False
                try:
                    self._ser.reset_input_buffer()   # discard any stale reply from a previous timed-out attempt
                    self._ser.write(frame_out)
                    log.info(f"→ CMD 0x{cmd:02X} (attempt {attempt}/{CMD_RETRIES})")
                except Exception as exc:
                    log.error(f"send_command write error: {exc}")
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser      = None
                    self._port     = None
                    self.connected = False
                    return False

            # Wait for reply frame
            try:
                with self._lock:
                    if not self._ser or not self._ser.is_open:
                        return False
                    # Hunt for start byte
                    raw = self._ser.read(1)

                if raw and raw[0] == FRAME_START_BYTE:
                    with self._lock:
                        if not self._ser or not self._ser.is_open:
                            return False
                        rest = self._ser.read(PAYLOAD_LEN + 2)

                    if len(rest) == PAYLOAD_LEN + 2:
                        payload  = rest[:PAYLOAD_LEN]
                        crc_rx   = struct.unpack("<H", rest[PAYLOAD_LEN:])[0]
                        if crc16_modbus(payload) == crc_rx:
                            self._parse_frame(payload)
                            return True

                log.warning(f"send_command(0x{cmd:02X}): no valid reply on attempt {attempt}/{CMD_RETRIES}")
                if attempt < CMD_RETRIES:
                    time.sleep(CMD_RETRY_DELAY_S)   # let ESP32 drain garbage before next attempt

            except Exception as exc:
                log.error(f"send_command read error (attempt {attempt}): {exc}")

        # All retries exhausted
        log.error(f"send_command(0x{cmd:02X}): failed after {CMD_RETRIES} attempts — disconnecting")
        self._disconnect()
        return False

    # ─── Frame parser ─────────────────────────────────────────────────────────

    def _parse_frame(self, payload: bytes) -> bool:
        """Decode all 26 payload bytes and update instance attributes."""
        try:
            # ── Live telemetry ─────────────────────────────────────────────
            pack_v_raw,   = struct.unpack_from(">H", payload, 0)
            pack_i_raw,   = struct.unpack_from(">h", payload, 2)
            soc           = payload[4]
            temp_c,       = struct.unpack_from("b",  payload, 5)
            power_w,      = struct.unpack_from(">h", payload, 6)
            avg_cell_raw, = struct.unpack_from(">H", payload, 8)
            alarm_flags,  = struct.unpack_from(">H", payload, 10)
            overlord      = payload[12]
            contactor_st  = payload[13]

            self.voltage         = pack_v_raw  / 100.0
            self.current         = pack_i_raw  / 10.0
            self.soc             = int(soc)
            self.temperature     = float(temp_c)
            self.power           = int(power_w)
            self.avg_cell_volt   = avg_cell_raw / 100.0
            self.alarm_flags     = alarm_flags
            self.overlord_state  = overlord
            self.contactor_state = contactor_st

            # ── EEPROM config ──────────────────────────────────────────────
            over_v_raw,  = struct.unpack_from(">H", payload, 14)
            under_v_raw, = struct.unpack_from(">H", payload, 16)
            over_t,      = struct.unpack_from("b",  payload, 18)
            under_t,     = struct.unpack_from("b",  payload, 19)
            num_modules  = payload[20]
            num_strings  = payload[21]

            # Overcurrent threshold — 0.1A resolution
            over_i_raw,  = struct.unpack_from(">H", payload, 22)

            self.eep_overvoltage        = over_v_raw  / 1000.0
            self.eep_undervoltage       = under_v_raw / 1000.0
            self.eep_overtemp           = float(over_t)
            self.eep_undertemp          = float(under_t)
            self.eep_num_modules        = num_modules
            self.eep_num_strings        = max(num_strings, 1)
            self.eep_overcurrent_thresh = over_i_raw  / 10.0   # 0.1A → A

            # bytes 24-25 reserved — ignored

            # ── Derived topology ───────────────────────────────────────────
            self.cell_count  = self.eep_num_modules // self.eep_num_strings
            self.capacity_ah = round(
                (self.eep_num_modules / self.eep_num_strings) * AH_PER_MODULE, 1
            )

            # ── Bookkeeping ───────────────────────────────────────��────────
            self.last_frame_time = time.time()
            self.frame_count    += 1

            log.debug(
                f"#{self.frame_count}: "
                f"{self.voltage:.2f}V {self.current:+.1f}A "
                f"SOC={self.soc}% {self.temperature}°C "
                f"OV={self.eep_overvoltage:.3f}V UV={self.eep_undervoltage:.3f}V "
                f"OC={self.eep_overcurrent_thresh:.1f}A "
                f"mod={num_modules} str={num_strings} "
                f"cells={self.cell_count}S cap={self.capacity_ah}Ah "
                f"alarms=0x{alarm_flags:04X}"
            )
            return True

        except Exception as exc:
            log.error(f"_parse_frame: {exc}")
            return False

    # ─── Background reader loop ───────────────────────────────────────────────

    def _reader_loop(self) -> None:
        log.info("Reader thread started.")

        while not self._stop_event.is_set():

            if not self._connect():
                time.sleep(1.0)
                continue

            try:
                # Hunt for start byte
                found = False
                for _ in range(FRAME_LEN * 4):
                    with self._lock:
                        raw = self._ser.read(1) if self._ser and self._ser.is_open else b""
                    if not raw:
                        break
                    if raw[0] == FRAME_START_BYTE:
                        found = True
                        break

                if not found:
                    now = time.time()
                    if now - self._last_poll_time >= POLL_INTERVAL_S:
                        self._last_poll_time = now
                        log.debug("Polling ESP32 for data …")
                        self.send_command(EXT_CMD_SEND_DATA)
                    continue

                # Read payload + CRC
                remainder = PAYLOAD_LEN + 2   # 28
                with self._lock:
                    rest = self._ser.read(remainder) if self._ser and self._ser.is_open else b""

                if len(rest) != remainder:
                    log.warning(f"Short frame: got {len(rest)}/{remainder} bytes")
                    continue

                # Validate CRC
                payload  = rest[:PAYLOAD_LEN]
                crc_rx   = struct.unpack("<H", rest[PAYLOAD_LEN:])[0]
                crc_calc = crc16_modbus(payload)

                if crc_calc != crc_rx:
                    log.warning(f"CRC mismatch: calc=0x{crc_calc:04X} rx=0x{crc_rx:04X}")
                    continue

                self._parse_frame(payload)

            except serial.SerialException as exc:
                log.error(f"Serial error: {exc} — reconnecting …")
                self._disconnect()
                time.sleep(2.0)

            except Exception as exc:
                log.error(f"Reader error: {exc}")
                self._disconnect()
                time.sleep(2.0)

        log.info("Reader thread stopped.")

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._reader_loop, name="teslabms-reader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._disconnect()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)


# ─────────────────────────────────────────────────────────────────────────────
#  D-Bus service builder
# ─────────────────────────────────────────────────────────────────────────────

def build_dbus_service(bus) -> VeDbusService:
    svc = VeDbusService(DBUS_SERVICE_NAME, bus=bus, register=False)

    # ── Management / identity ─────────────────────────────────────────────────
    svc.add_path("/Mgmt/ProcessName",    __file__)
    svc.add_path("/Mgmt/ProcessVersion", "1.4.0")
    svc.add_path("/Mgmt/Connection",     "USB Serial (auto-detect)")
    svc.add_path("/DeviceInstance",  1)
    svc.add_path("/ProductId",       0xBA77)
    svc.add_path("/ProductName",     "TeslaBMS-ESP32")
    svc.add_path("/FirmwareVersion", "1.3")
    svc.add_path("/HardwareVersion", "ESP32 HEX v1.0")
    svc.add_path("/Connected",       0)
    svc.add_path("/CustomName",      "TeslaBMS", writeable=True)
    svc.add_path("/Serial",          "TESLABMS001")

    # ── State ─────────────────────────────────────────────────────────────────
    svc.add_path("/State",     0)
    svc.add_path("/ErrorCode", None)

    # ── SOC / SOH ─────────────────────────────────────────────────────────────
    svc.add_path("/Soc", None, writeable=True)
    svc.add_path("/Soh", None, writeable=True)

    # ── DC measurements ───────────────────────────────────────────────────────
    svc.add_path("/Dc/0/Voltage",     None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}V")
    svc.add_path("/Dc/0/Current",     None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Dc/0/Power",       None, writeable=True, gettextcallback=lambda p, v: f"{v:.0f}W")
    svc.add_path("/Dc/0/Temperature", None, writeable=True)

    # ── Capacity ──────────────────────────────────────────────────────────────
    svc.add_path("/Capacity",          None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}Ah")
    svc.add_path("/InstalledCapacity", None, writeable=True, gettextcallback=lambda p, v: f"{v:.0f}Ah")
    svc.add_path("/ConsumedAmphours",  None, writeable=True, gettextcallback=lambda p, v: f"{v:.0f}Ah")

    # ── CVL / CCL / DCL ───────────────────────────────────────────────────────
    svc.add_path("/Info/MaxChargeVoltage",    None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}V")
    svc.add_path("/Info/BatteryLowVoltage",   None, writeable=True)
    svc.add_path("/Info/MaxChargeCurrent",    None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/MaxDischargeCurrent", None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/ChargeMode",          None, writeable=True)
    svc.add_path("/Info/TailCurrent",         None, writeable=True, gettextcallback=lambda p, v: f"{v:.1f}A")

    # ── System / cell data ────────────────────────────────────────────────────
    svc.add_path("/System/NrOfCellsPerBattery",         None, writeable=True)
    svc.add_path("/System/NrOfModulesOnline",              0, writeable=True)
    svc.add_path("/System/NrOfModulesOffline",             1, writeable=True)
    svc.add_path("/System/NrOfModulesBlockingCharge",   None, writeable=True)
    svc.add_path("/System/NrOfModulesBlockingDischarge",None, writeable=True)
    svc.add_path("/System/MinCellVoltage",  None, writeable=True, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/System/MaxCellVoltage",  None, writeable=True, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/System/MinVoltageCellId",None, writeable=True)
    svc.add_path("/System/MaxVoltageCellId",None, writeable=True)
    svc.add_path("/System/MinCellTemperature", None, writeable=True)
    svc.add_path("/System/MaxCellTemperature", None, writeable=True)
    svc.add_path("/System/Temperature1",       None, writeable=True)
    svc.add_path("/System/Temperature1Name",   "Battery", writeable=True)

    # ── IO / FET flags ────────────────────────────────────────────────────────
    svc.add_path("/Io/AllowToCharge",    0, writeable=True)
    svc.add_path("/Io/AllowToDischarge", 0, writeable=True)
    svc.add_path("/Io/AllowToBalance",   None, writeable=True)

    # ── Alarms ────────────────────────────────────────────────────────────────
    svc.add_path("/Alarms/LowVoltage",           None, writeable=True)
    svc.add_path("/Alarms/HighVoltage",          None, writeable=True)
    svc.add_path("/Alarms/LowCellVoltage",       None, writeable=True)
    svc.add_path("/Alarms/HighCellVoltage",      None, writeable=True)
    svc.add_path("/Alarms/LowSoc",               None, writeable=True)
    svc.add_path("/Alarms/HighTemperature",      None, writeable=True)
    svc.add_path("/Alarms/LowTemperature",       None, writeable=True)
    svc.add_path("/Alarms/HighChargeCurrent",    None, writeable=True)
    svc.add_path("/Alarms/HighDischargeCurrent", None, writeable=True)
    svc.add_path("/Alarms/InternalFailure",      None, writeable=True)
    svc.add_path("/Alarms/BmsCable",             None, writeable=True)

    # ── History ───────────────────────────────────────────────────────────────
    svc.add_path("/History/MinimumVoltage",     None, writeable=True)
    svc.add_path("/History/MaximumVoltage",     None, writeable=True)
    svc.add_path("/History/MinimumTemperature", None, writeable=True)
    svc.add_path("/History/MaximumTemperature", None, writeable=True)

    # ── Live EEPROM config mirror (read-only) ─────────────────────────────────
    svc.add_path("/Config/CellOverVoltage",      None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/CellUnderVoltage",     None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/OverTemp",             None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/UnderTemp",            None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/NumModules",           None, writeable=False)
    svc.add_path("/Config/NumStrings",           None, writeable=False)
    svc.add_path("/Config/CellCount",            None, writeable=False)
    svc.add_path("/Config/CapacityAh",           None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}Ah")
    svc.add_path("/Config/OverCurrentThreshold", None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}A")

    # ── User settings mirror (read-only display of what localsettings holds) ──
    svc.add_path("/Settings/MaxChargeCurrent",    None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}A")
    svc.add_path("/Settings/MaxDischargeCurrent", None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}A")
    svc.add_path("/Settings/AbsorptionVoltage",   None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Settings/FloatVoltage",        None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Settings/TailCurrent",         None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}A")
    svc.add_path("/Settings/MaxChargeTemp",       None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Settings/MinChargeTemp",       None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")

    # ── Commands ──────────────────────────────────────────────────────────────
    svc.add_path("/Control/Shutdown", 0, writeable=True)
    svc.add_path("/Control/Startup",  0, writeable=True)

    svc.register()
    return svc


# ─────────────────────────────────────────────────────────────────────────────
#  Publish callback
# ─────────────────────────────────────────────────────────────────────────────

_history = {"min_v": None, "max_v": None, "min_t": None, "max_t": None}


def publish(bms: TeslaBMSSerial, svc: VeDbusService, vs: VenusSettings) -> bool:

    # ── 1. Commands ───────────────────────────────────────────────────────────
    if svc["/Control/Shutdown"] == 1:
        log.info("D-Bus → SHUTDOWN")
        bms.send_command(EXT_CMD_SHUTDOWN)
        svc["/Control/Shutdown"] = 0

    if svc["/Control/Startup"] == 1:
        log.info("D-Bus → STARTUP")
        bms.send_command(EXT_CMD_STARTUP)
        svc["/Control/Startup"] = 0

    # ── 2. Connection state ───────────────────────────────────────────────────
    online = not bms.is_stale
    svc["/Connected"]                 = 1 if online else 0
    svc["/System/NrOfModulesOnline"]  = 1 if online else 0
    svc["/System/NrOfModulesOffline"] = 0 if online else 1
    svc["/Alarms/BmsCable"]           = ALARM_OK if online else ALARM_WARNING

    if not online:
        svc["/State"] = 0
        return True

    # ── 3. Core telemetry ─────────────────────────────────────────────────────
    svc["/Soc"]              = bms.soc
    svc["/Dc/0/Voltage"]     = round(bms.voltage,     2)
    svc["/Dc/0/Current"]     = round(bms.current,     2)
    svc["/Dc/0/Power"]       = round(bms.power,       0)
    svc["/Dc/0/Temperature"] = round(bms.temperature, 1)

    # ── 4. Cell / temperature ─────────────────────────────────────────────────
    svc["/System/NrOfCellsPerBattery"]  = bms.cell_count
    svc["/System/MinCellVoltage"]       = round(bms.avg_cell_volt, 3)
    svc["/System/MaxCellVoltage"]       = round(bms.avg_cell_volt, 3)
    svc["/System/MinVoltageCellId"]     = "avg"
    svc["/System/MaxVoltageCellId"]     = "avg"
    svc["/System/Temperature1"]         = round(bms.temperature, 1)
    svc["/System/MinCellTemperature"]   = round(bms.temperature, 1)
    svc["/System/MaxCellTemperature"]   = round(bms.temperature, 1)

    # ── 5. Capacity ───────────────────────────────────────────────────────────
    cap_remain   = round(bms.capacity_ah * bms.soc / 100.0, 2)
    cap_consumed = round(bms.capacity_ah - cap_remain, 2)
    svc["/InstalledCapacity"]  = bms.capacity_ah
    svc["/Capacity"]           = cap_remain
    svc["/ConsumedAmphours"]   = -cap_consumed

    # ── 6. CVL / CCL / DCL ───────────────────────────────────────────────────
    # CVL: use absorption voltage during bulk/absorption, float voltage after.
    # For now we publish absorption voltage as MaxChargeVoltage — the Victron
    # DVCC system manages the bulk→absorption→float transition using
    # TailCurrent as the end-of-absorption trigger.
    #
    # CCL / DCL: user setting clamped by ESP32 overcurrent threshold.
    # Neither can exceed the contactor's hardware rating.
    #
    # Charge inhibit: if temperature is outside the charge window, CCL = 0.
    temp_ok_for_charge = (
        bms.temperature >= vs.min_charge_temp and
        bms.temperature <= vs.max_charge_temp
    )

    ccl = vs.effective_max_charge_current(bms.eep_overcurrent_thresh)
    dcl = vs.effective_max_discharge_current(bms.eep_overcurrent_thresh)

    svc["/Info/MaxChargeVoltage"]    = vs.pack_absorption_voltage(bms.cell_count)
    svc["/Info/BatteryLowVoltage"]   = bms.pack_undervoltage
    svc["/Info/MaxChargeCurrent"]    = round(ccl, 1) if (bms.charge_fet and temp_ok_for_charge) else 0.0
    svc["/Info/MaxDischargeCurrent"] = round(dcl, 1) if bms.discharge_fet else 0.0
    svc["/Info/TailCurrent"]         = vs.tail_current
    svc["/Info/ChargeMode"]          = (
        "Normal"  if bms.overlord_state == OVERLORD_NORMAL  else
        "Storage" if bms.overlord_state == OVERLORD_STORAGE else
        "Fault"
    )

    # ── 7. FET / IO flags ─────────────────────────────────────────────────────
    svc["/Io/AllowToCharge"]                     = 1 if (bms.charge_fet and temp_ok_for_charge) else 0
    svc["/Io/AllowToDischarge"]                  = 1 if bms.discharge_fet else 0
    svc["/System/NrOfModulesBlockingCharge"]     = 0 if (bms.charge_fet and temp_ok_for_charge) else 1
    svc["/System/NrOfModulesBlockingDischarge"]  = 0 if bms.discharge_fet else 1

    # ── 8. State machine ──────────────────────────────────────────────────────
    if bms.overlord_state == OVERLORD_FAULT:
        svc["/State"] = 10
    elif not bms.charge_fet and not bms.discharge_fet:
        svc["/State"] = 14
    else:
        svc["/State"] = 9

    # ── 9. Alarms ─────────────────────────────────────────────────────────────
    svc["/Alarms/LowVoltage"]            = bms.alarm_level(ALARM_UNDER_VOLTAGE)
    svc["/Alarms/HighVoltage"]           = bms.alarm_level(ALARM_OVER_VOLTAGE)
    svc["/Alarms/LowCellVoltage"]        = bms.alarm_level(ALARM_UNDER_VOLTAGE)
    svc["/Alarms/HighCellVoltage"]       = bms.alarm_level(ALARM_OVER_VOLTAGE)
    svc["/Alarms/HighTemperature"]       = bms.alarm_level(ALARM_OVER_TEMP)
    svc["/Alarms/LowTemperature"]        = bms.alarm_level(ALARM_UNDER_TEMP)
    svc["/Alarms/HighChargeCurrent"]     = bms.alarm_level(ALARM_OVER_CURRENT)
    svc["/Alarms/HighDischargeCurrent"]  = bms.alarm_level(ALARM_OVER_CURRENT)
    svc["/Alarms/InternalFailure"]       = ALARM_ALARM if bms.overlord_state == OVERLORD_FAULT else ALARM_OK
    svc["/Alarms/LowSoc"] = (
        ALARM_ALARM   if bms.soc < 5  else
        ALARM_WARNING if bms.soc < 10 else
        ALARM_OK
    )
    # Temperature charge inhibit warning
    if not temp_ok_for_charge and online:
        log.debug(f"Charge inhibited: temp {bms.temperature}°C outside [{vs.min_charge_temp}, {vs.max_charge_temp}]°C")

    # ── 10. EEPROM config mirror ───────────────────────────────────────────────
    svc["/Config/CellOverVoltage"]      = round(bms.eep_overvoltage,        3)
    svc["/Config/CellUnderVoltage"]     = round(bms.eep_undervoltage,       3)
    svc["/Config/OverTemp"]             = bms.eep_overtemp
    svc["/Config/UnderTemp"]            = bms.eep_undertemp
    svc["/Config/NumModules"]           = bms.eep_num_modules
    svc["/Config/NumStrings"]           = bms.eep_num_strings
    svc["/Config/CellCount"]            = bms.cell_count
    svc["/Config/CapacityAh"]           = bms.capacity_ah
    svc["/Config/OverCurrentThreshold"] = round(bms.eep_overcurrent_thresh, 1)

    # ── 11. User settings mirror ───────────────────────────────────────────────
    svc["/Settings/MaxChargeCurrent"]    = vs.max_charge_current
    svc["/Settings/MaxDischargeCurrent"] = vs.max_discharge_current
    svc["/Settings/AbsorptionVoltage"]   = vs.absorption_voltage
    svc["/Settings/FloatVoltage"]        = vs.float_voltage
    svc["/Settings/TailCurrent"]         = vs.tail_current
    svc["/Settings/MaxChargeTemp"]       = vs.max_charge_temp
    svc["/Settings/MinChargeTemp"]       = vs.min_charge_temp

    # ── 12. History ───────────────────────────────────────────────────────────
    if _history["min_v"] is None or bms.voltage     < _history["min_v"]: _history["min_v"] = bms.voltage
    if _history["max_v"] is None or bms.voltage     > _history["max_v"]: _history["max_v"] = bms.voltage
    if _history["min_t"] is None or bms.temperature < _history["min_t"]: _history["min_t"] = bms.temperature
    if _history["max_t"] is None or bms.temperature > _history["max_t"]: _history["max_t"] = bms.temperature

    svc["/History/MinimumVoltage"]     = _history["min_v"]
    svc["/History/MaximumVoltage"]     = _history["max_v"]
    svc["/History/MinimumTemperature"] = _history["min_t"]
    svc["/History/MaximumTemperature"] = _history["max_t"]

    return True


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 62)
    log.info("  TeslaBMS-ESP32 Venus OS driver v1.4 starting …")
    log.info("=" * 62)

    DBusGMainLoop(set_as_default=True)

    bus = (
        dbus.SessionBus()
        if "DBUS_SESSION_BUS_ADDRESS" in os.environ
        else dbus.SystemBus()
    )

    # ── User settings (persistent, localsettings) ─────────────────────────────
    vs = VenusSettings(bus)
    vs.setup()

    # ── Serial BMS handler ────────────────────────────────────────────────────
    bms = TeslaBMSSerial()
    bms.start()

    # ── D-Bus service ─────────────────────────────────────────────────────────
    svc = build_dbus_service(bus)
    log.info(f"D-Bus service '{DBUS_SERVICE_NAME}' registered.")

    GLib.timeout_add(PUBLISH_INTERVAL_MS, lambda: publish(bms, svc, vs))

    mainloop = GLib.MainLoop()

    def _shutdown(sig, frame):
        log.info(f"Signal {sig} — shutting down …")
        bms.stop()
        mainloop.quit()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT,  _shutdown)

    log.info("Waiting for ESP32 data …")
    mainloop.run()
    log.info("Driver stopped.")


if __name__ == "__main__":
    main()