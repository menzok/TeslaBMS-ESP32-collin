#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ─────────────────────────────────────────────────────────────────────────────
#  dbus-teslabms.py  —  Standalone Venus OS battery driver for TeslaBMS-ESP32
#
#  Single frame design
#  ───────────────────
#  The ESP32 sends one 27-byte frame every ~1000 ms (unsolicited heartbeat).
#  Every frame carries BOTH live telemetry AND the current EEPROM config
#  values.  There is no separate config request or second frame type.
#
#  If no frame arrives within STALE_TIMEOUT seconds the driver sends
#  EXT_CMD_SEND_DATA (0x03) to demand an immediate reply.
#
#  Frame format (27 bytes total)
#  ──────────────────────────────
#  [0]      0xAA           start byte
#  [1..24]  payload        24-byte payload (see map below)
#  [25..26] CRC16/MODBUS   little-endian, computed over payload bytes only
#
#  Payload map (all multi-byte fields big-endian)
#  ──────────────────────────────────────────────
#  [0-1]   packV          uint16  × 100   → V   (10 mV resolution)
#  [2-3]   packI          int16   × 10    → A   (100 mA resolution, + = charge)
#  [4]     soc            uint8   0-100   %
#  [5]     temp           int8    °C      (signed)
#  [6-7]   power          int16   W
#  [8-9]   avgCell        uint16  × 100   → V   (10 mV resolution)
#  [10-11] alarmFlags     uint16  bitmask (ALARM_* constants)
#  [12]    overlordState  uint8   0=Normal 1=Fault 2=StorageMode
#  [13]    contactorState uint8   raw contactor enum
#  [14-15] overVoltage    uint16  × 1000  → V   (1 mV resolution)
#  [16-17] underVoltage   uint16  × 1000  → V   (1 mV resolution)
#  [18]    overTemp       int8    °C      (signed, whole degrees)
#  [19]    underTemp      int8    °C      (signed, whole degrees)
#  [20]    numModules     uint8   total modules detected by BMS
#  [21]    numStrings     uint8   parallel string count (from EEPROM)
#  [22]    reserved       0x00
#  [23]    reserved       0x00
#
#  Capacity formula:  Ah = (numModules / numStrings) × AH_PER_MODULE
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

# ── velib_python — ships with every Venus OS installation ────────────────────
VELIB_PATH = "/opt/victronenergy/dbus-serialbattery/ext/velib_python"
sys.path.insert(1, VELIB_PATH)
from vedbus import VeDbusService   # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
#  Constants — only things Venus OS owns (not mirrored from ESP32 EEPROM)
# ─────────────────────────────────────────────────────���───────────────────────

# Nominal Ah per series module in a Tesla pack
AH_PER_MODULE = 232.0

# Float voltage offset below the EEPROM overvoltage threshold.
# e.g. OV=4.25 V → float target = 4.25 - 0.10 = 4.15 V per cell.
# Venus OS sends this as CVL to the charger so it does not continuously
# push all the way to the absolute cell maximum.
CELL_FLOAT_OFFSET = 0.10   # V

# Maximum continuous charge / discharge current limits (A)
# Adjust to match your pack's BMS/fuse rating.
MAX_CHARGE_CURRENT    = 200.0
MAX_DISCHARGE_CURRENT = 200.0

# Timing
STALE_TIMEOUT       = 2.5    # s — demand a frame if nothing received
OFFLINE_TIMEOUT     = 15.0   # s — declare BMS offline
BAUD_RATE           = 115200
PUBLISH_INTERVAL_MS = 1000   # ms — GLib publish timer

# D-Bus identity
DBUS_SERVICE_NAME = "com.victronenergy.battery.teslabms"

# ─────────────────────────────────────────────────────────────────────────────
#  Protocol constants  (must match ExternalCommsLayer.h)
# ─────────────────────────────────────────────────────────────────────────────

EXT_CMD_SHUTDOWN  = 0x01
EXT_CMD_STARTUP   = 0x02
EXT_CMD_SEND_DATA = 0x03

FRAME_START_BYTE  = 0xAA
PAYLOAD_LEN       = 24
FRAME_LEN         = 1 + PAYLOAD_LEN + 2   # = 27

ALARM_OVER_VOLTAGE  = (1 << 0)
ALARM_UNDER_VOLTAGE = (1 << 1)
ALARM_OVER_TEMP     = (1 << 2)
ALARM_UNDER_TEMP    = (1 << 3)
ALARM_OVER_CURRENT  = (1 << 4)

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
#  CRC-16 / MODBUS  (identical to ExternalCommsLayer::calculateCRC16)
# ─────────────────────────────────────────────────────────────────────────────

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc

# ─────────────────────────────────────────────────────────────────────────────
#  TeslaBMSSerial  —  UART handler + decoded state (background thread)
# ─────────────────────────────────────────────────────────────────────────────

class TeslaBMSSerial:
    """
    Runs a background daemon thread that:
      1. Auto-detects and opens the ESP32 serial port.
      2. Hunts for 0xAA start bytes and reads 27-byte frames.
      3. Validates CRC-16/MODBUS.
      4. Decodes all 24 payload bytes into public attributes.

    The main GLib thread reads these attributes every publish cycle.
    send_command() is thread-safe via a lock on the serial port.
    """

    def __init__(self):
        # ── Live telemetry (from payload bytes 0-13) ──────────────────────────
        self.voltage         = 0.0    # V
        self.current         = 0.0    # A  (positive = charging)
        self.soc             = 50     # %
        self.temperature     = 25.0   # °C
        self.power           = 0      # W
        self.avg_cell_volt   = 3.30   # V
        self.alarm_flags     = 0      # raw bitmask
        self.overlord_state  = 0      # 0=Normal 1=Fault 2=StorageMode
        self.contactor_state = 0      # raw enum

        # ── EEPROM config (from payload bytes 14-21) ──────────────────────────
        # These are updated on EVERY frame — always current, no staleness risk.
        self.eep_overvoltage  = 4.25   # V per cell
        self.eep_undervoltage = 2.90   # V per cell
        self.eep_overtemp     = 60.0   # °C
        self.eep_undertemp    = -10.0  # °C
        self.eep_num_modules  = 28     # total modules detected
        self.eep_num_strings  = 2      # parallel strings

        # ── Derived pack values (recalculated on every frame) ─────────────────
        self.cell_count   = 14         # series cells per string
        self.capacity_ah  = 232.0      # Ah

        # ── Connection bookkeeping ────────────────────────────────────────────
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

    # ─── Derived pack limits (always computed live from EEPROM fields) ────────

    @property
    def pack_overvoltage(self) -> float:
        """Absolute pack OV limit = cell OV × series cells."""
        return round(self.eep_overvoltage * self.cell_count, 2)

    @property
    def pack_undervoltage(self) -> float:
        """Absolute pack UV limit = cell UV × series cells."""
        return round(self.eep_undervoltage * self.cell_count, 2)

    @property
    def pack_float_voltage(self) -> float:
        """
        CVL sent to the charger = (cell OV - CELL_FLOAT_OFFSET) × series cells.
        Keeps the charger from perpetually pushing to the absolute cell max.
        The ESP32 still opens the contactor independently if OV is breached.
        """
        return round((self.eep_overvoltage - CELL_FLOAT_OFFSET) * self.cell_count, 2)

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
        """
        Open port, send EXT_CMD_SEND_DATA, verify we get a valid 27-byte reply.
        Returns True if the port is confirmed as a TeslaBMS-ESP32.
        """
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.8)
            log.info(f"Probing {port} …")
            cmd   = bytes([EXT_CMD_SEND_DATA])
            frame = bytes([FRAME_START_BYTE]) + cmd + struct.pack("<H", crc16_modbus(cmd))
            ser.write(frame)
            time.sleep(0.3)
            data = ser.read(FRAME_LEN)
            ser.close()
            if len(data) >= FRAME_LEN and data[0] == FRAME_START_BYTE:
                payload = data[1: 1 + PAYLOAD_LEN]
                crc_rx  = struct.unpack("<H", data[1 + PAYLOAD_LEN: FRAME_LEN])[0]
                if crc16_modbus(payload) == crc_rx:
                    log.info(f"✅ TeslaBMS-ESP32 confirmed on {port}")
                    return True
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

    # ─── Connection management ────────────────────────────────────────────────

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
                self._ser      = serial.Serial(port, BAUD_RATE, timeout=1.2)
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

    # ─── Command send (thread-safe) ───────────────────────────────────────────

    def send_command(self, cmd: int) -> bool:
        """
        Transmit a 4-byte command frame: [0xAA][cmd][CRC_lo][CRC_hi]
        CRC is computed over the single command byte only.
        Safe to call from any thread.
        """
        cmd_byte  = bytes([cmd])
        frame_out = (bytes([FRAME_START_BYTE])
                     + cmd_byte
                     + struct.pack("<H", crc16_modbus(cmd_byte)))
        with self._lock:
            if not self._ser or not self._ser.is_open:
                log.warning(f"send_command(0x{cmd:02X}): not connected")
                return False
            try:
                self._ser.write(frame_out)
                log.info(f"→ CMD 0x{cmd:02X}")
                return True
            except Exception as exc:
                log.error(f"send_command error: {exc}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser      = None
                self._port     = None
                self.connected = False
                return False

    # ─── Frame parser ─────────────────────────────────────────────���───────────

    def _parse_frame(self, payload: bytes) -> bool:
        """
        Decode all 24 payload bytes and update instance attributes.

        Called from the reader thread on every validated frame.
        Attribute writes are atomic for Python basic types, so the main
        thread can read them safely without an explicit lock.
        """
        try:
            # ── Live telemetry ─────────────────────────────────────────────
            pack_v_raw,   = struct.unpack_from(">H", payload, 0)
            pack_i_raw,   = struct.unpack_from(">h", payload, 2)   # signed
            soc           = payload[4]
            temp_c,       = struct.unpack_from("b",  payload, 5)   # signed
            power_w,      = struct.unpack_from(">h", payload, 6)   # signed
            avg_cell_raw, = struct.unpack_from(">H", payload, 8)
            alarm_flags,  = struct.unpack_from(">H", payload, 10)
            overlord      = payload[12]
            contactor_st  = payload[13]

            self.voltage         = pack_v_raw  / 100.0   # 10 mV → V
            self.current         = pack_i_raw  / 10.0    # 100 mA → A
            self.soc             = int(soc)
            self.temperature     = float(temp_c)
            self.power           = int(power_w)
            self.avg_cell_volt   = avg_cell_raw / 100.0  # 10 mV → V
            self.alarm_flags     = alarm_flags
            self.overlord_state  = overlord
            self.contactor_state = contactor_st

            # ── EEPROM config ──────────────────────────────────────────────
            # These are transmitted in every frame — always authoritative,
            # always current.  No staleness, no separate config request.
            over_v_raw,  = struct.unpack_from(">H", payload, 14)
            under_v_raw, = struct.unpack_from(">H", payload, 16)
            over_t,      = struct.unpack_from("b",  payload, 18)   # signed
            under_t,     = struct.unpack_from("b",  payload, 19)   # signed
            num_modules  = payload[20]
            num_strings  = payload[21]

            self.eep_overvoltage  = over_v_raw  / 1000.0   # 1 mV → V
            self.eep_undervoltage = under_v_raw / 1000.0
            self.eep_overtemp     = float(over_t)
            self.eep_undertemp    = float(under_t)
            self.eep_num_modules  = num_modules
            self.eep_num_strings  = max(num_strings, 1)   # guard against /0

            # ── Derived topology ───────────────────────────────────────────
            self.cell_count  = self.eep_num_modules // self.eep_num_strings
            self.capacity_ah = round(
                (self.eep_num_modules / self.eep_num_strings) * AH_PER_MODULE, 1
            )

            # ── Bookkeeping ────────────────────────────────────────────────
            self.last_frame_time = time.time()
            self.frame_count    += 1

            log.debug(
                f"#{self.frame_count}: "
                f"{self.voltage:.2f}V {self.current:+.1f}A "
                f"SOC={self.soc}% {self.temperature}°C "
                f"OV={self.eep_overvoltage:.3f}V UV={self.eep_undervoltage:.3f}V "
                f"OT={self.eep_overtemp}°C UT={self.eep_undertemp}°C "
                f"mod={num_modules} str={num_strings} "
                f"cells={self.cell_count}S cap={self.capacity_ah}Ah"
            )
            return True

        except Exception as exc:
            log.error(f"_parse_frame: {exc}")
            return False

    # ─── Background reader loop ───────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """
        Daemon thread main loop.

        1. Ensures the serial port is open (retries every 5 s).
        2. Scans the byte stream for 0xAA start bytes.
        3. Reads the remaining 26 bytes (payload + CRC).
        4. Validates CRC; on success calls _parse_frame().
        5. If no valid frame has arrived within STALE_TIMEOUT, demands one.
        """
        log.info("Reader thread started.")

        while not self._stop_event.is_set():

            # ── Ensure connected ───────────────────────────────────────────
            if not self._connect():
                time.sleep(1.0)
                continue

            try:
                # ── Hunt for start byte ────────────────────────────────────
                # Read one byte at a time; cap iterations to avoid an
                # infinite spin on a stream of garbage bytes.
                found = False
                for _ in range(FRAME_LEN * 4):
                    with self._lock:
                        raw = self._ser.read(1) if self._ser and self._ser.is_open else b""
                    if not raw:
                        # Serial read timeout — no data in the window
                        break
                    if raw[0] == FRAME_START_BYTE:
                        found = True
                        break

                if not found:
                    # Check if we've gone stale and need to demand a frame
                    age = time.time() - self.last_frame_time
                    if self.last_frame_time > 0 and age > STALE_TIMEOUT:
                        log.debug(f"No frame for {age:.1f}s — requesting data")
                        self.send_command(EXT_CMD_SEND_DATA)
                    continue

                # ── Read payload + CRC (26 bytes after the start byte) ─────
                remainder = PAYLOAD_LEN + 2   # 26
                with self._lock:
                    rest = self._ser.read(remainder) if self._ser and self._ser.is_open else b""

                if len(rest) != remainder:
                    log.warning(f"Short frame: got {len(rest)}/{remainder} bytes")
                    continue

                # ── Validate CRC ───────────────────────────────────────────
                payload  = rest[:PAYLOAD_LEN]
                crc_rx   = struct.unpack("<H", rest[PAYLOAD_LEN:])[0]
                crc_calc = crc16_modbus(payload)

                if crc_calc != crc_rx:
                    log.warning(
                        f"CRC mismatch: calc=0x{crc_calc:04X} rx=0x{crc_rx:04X}"
                    )
                    continue

                # ── Good frame ─────────────────────────────────────────────
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

    # ─── Thread lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._reader_loop,
            name="teslabms-reader",
            daemon=True,
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
    """
    Register com.victronenergy.battery.teslabms on the D-Bus.

    /Config/* paths are read-only mirrors of the ESP32 EEPROM values.
    They are updated every publish cycle so dbus-spy / VRM / Node-RED
    always show the current hardware configuration.

    /Control/* paths are writable command triggers.
    """
    svc = VeDbusService(DBUS_SERVICE_NAME, bus=bus, register=False)

    # ── Management ────────────────────────────────────────────────────────────
    svc.add_path("/Mgmt/ProcessName",    __file__)
    svc.add_path("/Mgmt/ProcessVersion", "1.2.0")
    svc.add_path("/Mgmt/Connection",     "USB Serial (auto-detect)")

    # ── Identity ──────────────────────────────────────────────────────────────
    svc.add_path("/DeviceInstance",  1)
    svc.add_path("/ProductId",       0xBA77)
    svc.add_path("/ProductName",     "TeslaBMS-ESP32")
    svc.add_path("/FirmwareVersion", "1.2")
    svc.add_path("/HardwareVersion", "ESP32 HEX v1.0")
    svc.add_path("/Connected",       0)
    svc.add_path("/CustomName",      "TeslaBMS", writeable=True)
    svc.add_path("/Serial",          "TESLABMS001")

    # ── State ─────────��───────────────────────────────────────────────────────
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

    # ── CVL / CCL / DCL (DVCC) ────────────────────────────────────────────────
    svc.add_path("/Info/MaxChargeVoltage",    None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}V")
    svc.add_path("/Info/BatteryLowVoltage",   None, writeable=True)
    svc.add_path("/Info/MaxChargeCurrent",    None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/MaxDischargeCurrent", None, writeable=True, gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/ChargeMode",          None, writeable=True)

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

    # ── Alarms (0=OK 1=Warning 2=Alarm) ──────────────────────────────────────
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

    # ── Live EEPROM config mirror (read-only, updated every frame) ────────────
    # These paths let Node-RED / dbus-spy / VRM see exactly what the ESP32
    # currently has in EEPROM without any separate config request.
    svc.add_path("/Config/CellOverVoltage",  None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/CellUnderVoltage", None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/CellFloatVoltage", None, writeable=False, gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/OverTemp",         None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/UnderTemp",        None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/NumModules",       None, writeable=False)
    svc.add_path("/Config/NumStrings",       None, writeable=False)
    svc.add_path("/Config/CellCount",        None, writeable=False)
    svc.add_path("/Config/CapacityAh",       None, writeable=False, gettextcallback=lambda p, v: f"{v:.1f}Ah")

    # ── Command triggers (write 1 to fire, auto-clears to 0) ─────────────────
    svc.add_path("/Control/Shutdown", 0, writeable=True)
    svc.add_path("/Control/Startup",  0, writeable=True)

    svc.register()
    return svc


# ─────────────────────────────────────────────────────────────────────────────
#  Publish callback — fires every PUBLISH_INTERVAL_MS via GLib timer
# ─────────────────────────────────────────────────────────────────────────────

_history = {"min_v": None, "max_v": None, "min_t": None, "max_t": None}


def publish(bms: TeslaBMSSerial, svc: VeDbusService) -> bool:
    """
    Reads the latest decoded state from *bms* and writes it to D-Bus.
    Returns True to keep the GLib timer running.
    """

    # ── 1. Command control ────────────────────────────────────────────────────
    if svc["/Control/Shutdown"] == 1:
        log.info("D-Bus → SHUTDOWN")
        bms.send_command(EXT_CMD_SHUTDOWN)
        svc["/Control/Shutdown"] = 0

    if svc["/Control/Startup"] == 1:
        log.info("D-Bus → STARTUP")
        bms.send_command(EXT_CMD_STARTUP)
        svc["/Control/Startup"] = 0

    # ── 2. Connection / online state ──────────────────────────────────────────
    online = not bms.is_stale
    svc["/Connected"]                    = 1 if online else 0
    svc["/System/NrOfModulesOnline"]     = 1 if online else 0
    svc["/System/NrOfModulesOffline"]    = 0 if online else 1
    svc["/Alarms/BmsCable"]              = ALARM_OK if online else ALARM_WARNING

    if not online:
        svc["/State"] = 0
        return True

    # ── 3. Core telemetry ──────────────────────────────────────────────────────
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

    # ── 5. Capacity ────────────────────────────────────────────────────────────
    cap_remain   = round(bms.capacity_ah * bms.soc / 100.0, 2)
    cap_consumed = round(bms.capacity_ah - cap_remain, 2)
    svc["/InstalledCapacity"]  = bms.capacity_ah
    svc["/Capacity"]           = cap_remain
    svc["/ConsumedAmphours"]   = -cap_consumed   # Victron convention: negative

    # ── 6. CVL / CCL / DCL ───────────────────────────────────────────────────
    # pack_float_voltage = (EEPROM OV - CELL_FLOAT_OFFSET) × cell_count
    # This is recalculated from the live EEPROM values in every frame,
    # so a user changing thresholds in the BMS configurator is reflected
    # in Venus OS within one heartbeat interval (~1 s).
    svc["/Info/MaxChargeVoltage"]    = bms.pack_float_voltage
    svc["/Info/BatteryLowVoltage"]   = bms.pack_undervoltage
    svc["/Info/MaxChargeCurrent"]    = MAX_CHARGE_CURRENT    if bms.charge_fet    else 0.0
    svc["/Info/MaxDischargeCurrent"] = MAX_DISCHARGE_CURRENT if bms.discharge_fet else 0.0
    svc["/Info/ChargeMode"]          = (
        "Normal"  if bms.overlord_state == OVERLORD_NORMAL
        else "Storage" if bms.overlord_state == OVERLORD_STORAGE
        else "Fault"
    )

    # ── 7. FET / IO flags ─────────────────────────────────────────────────────
    svc["/Io/AllowToCharge"]                     = 1 if bms.charge_fet    else 0
    svc["/Io/AllowToDischarge"]                  = 1 if bms.discharge_fet else 0
    svc["/System/NrOfModulesBlockingCharge"]     = 0 if bms.charge_fet    else 1
    svc["/System/NrOfModulesBlockingDischarge"]  = 0 if bms.discharge_fet else 1

    # ── 8. State machine ──────────────────────────────────────────────────────
    if bms.overlord_state == OVERLORD_FAULT:
        svc["/State"] = 10
    elif not bms.charge_fet and not bms.discharge_fet:
        svc["/State"] = 14
    else:
        svc["/State"] = 9

    # ── 9. Alarms ───────────────────────���─────────────────────────────────────
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

    # ── 10. Live EEPROM config mirror ─────────────────────────────────────────
    # Reflects the exact values currently in ESP32 EEPROM.
    # Updated every cycle — no staleness possible.
    svc["/Config/CellOverVoltage"]  = round(bms.eep_overvoltage,  3)
    svc["/Config/CellUnderVoltage"] = round(bms.eep_undervoltage, 3)
    svc["/Config/CellFloatVoltage"] = round(bms.eep_overvoltage - CELL_FLOAT_OFFSET, 3)
    svc["/Config/OverTemp"]         = bms.eep_overtemp
    svc["/Config/UnderTemp"]        = bms.eep_undertemp
    svc["/Config/NumModules"]       = bms.eep_num_modules
    svc["/Config/NumStrings"]       = bms.eep_num_strings
    svc["/Config/CellCount"]        = bms.cell_count
    svc["/Config/CapacityAh"]       = bms.capacity_ah

    # ── 11. History (in-memory lifetime min/max) ──────────────────────────────
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
    log.info("  TeslaBMS-ESP32 Venus OS driver v1.2 starting …")
    log.info("=" * 62)

    DBusGMainLoop(set_as_default=True)

    bus = (
        dbus.SessionBus()
        if "DBUS_SESSION_BUS_ADDRESS" in os.environ
        else dbus.SystemBus()
    )

    bms = TeslaBMSSerial()
    bms.start()

    svc = build_dbus_service(bus)
    log.info(f"D-Bus service '{DBUS_SERVICE_NAME}' registered.")

    GLib.timeout_add(PUBLISH_INTERVAL_MS, lambda: publish(bms, svc))

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