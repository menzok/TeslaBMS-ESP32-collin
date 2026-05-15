#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ─────────────────────────────────────────────────────────────────────────────
#  dbus-anenji.py — Standalone Venus OS vebus driver for the
#  ANENJI ANJ-3KW-24V-LV-WIFI inverter/charger.
#
#  Version 1.0.0
#
#  The device speaks Modbus RTU over RS-485 at 9600 8N1.  This driver mirrors
#  the same operational shape as the staged TeslaBMS Venus OS driver in this
#  repository: serial I/O happens in a background thread, D-Bus publication is
#  handled from the GLib main loop, serial-starter is stopped while the port is
#  owned, and reconnects target the same discovered port for the lifetime of
#  the process.
#
#  Important behavioural choices:
#    • The Modbus address is probed from the configured address first, then
#      1-10, then 1-247, and the discovered address is locked in RAM until the
#      process exits.
#    • Port discovery happens only during the first successful connect.  After a
#      disconnect the driver retries the same port only; it never starts
#      touching unrelated USB serial ports again until the process is restarted.
#    • No register defaults are assumed.  Settings and device metadata are read
#      from the inverter before the published D-Bus values become authoritative.
#
#  D-Bus services:
#    • com.victronenergy.vebus.anenji        — primary inverter/charger service
#    • com.victronenergy.solarcharger.anenji — always-on companion PV service
#
#  Writable controls:
#    • /Mode                                 → register 420 (best-effort on/off)
#    • /Ac/ActiveIn/CurrentLimit             → register 333
#    • /BatteryOperationalLimits/*           → registers 324 / 332 / 351
#    • /Settings/*                           → matching inverter setting registers
#    • /Control/RemoteOn, /Control/RemoteOff, /Control/ClearFaults
#
#  All write-backs are queued and executed on the serial thread so the GLib
#  main loop never blocks on RS-485 I/O.
# ─────────────────────────────────────────────────────────────────────────────

from __future__ import annotations

import glob
import logging
from logging.handlers import RotatingFileHandler
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import serial

# ── velib_python — use native Venus OS library (no external dependencies) ───
_VELIB_PATHS = [
    "/opt/victronenergy/dbus-systemcalc-py/ext/velib_python",
    "/opt/victronenergy/dbus-battery/ext/velib_python",
    "/data/apps/dbus-serialbattery/ext/velib_python",
]
for _candidate in _VELIB_PATHS:
    if os.path.isdir(_candidate):
        sys.path.insert(1, _candidate)
        break

from vedbus import VeDbusService  # noqa: E402
from settingsdevice import SettingsDevice  # noqa: E402

VERSION = "1.0.0"
# Staged, locally unique product id used for this non-official Venus OS driver.
PRODUCT_ID = 0xA3E1
PRODUCT_NAME = "ANENJI ANJ-3KW-24V-LV-WIFI"
FIRMWARE_VERSION = VERSION
DBUS_SERVICE_VEBUS = "com.victronenergy.vebus.anenji"
DBUS_SERVICE_SOLAR = "com.victronenergy.solarcharger.anenji"
LOCALSETTINGS_ROOT = "/Settings/Anenji"

BAUD_RATE = 9600
BYTE_SIZE = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE
SERIAL_TIMEOUT_S = 1.2
RETRY_COUNT = 3
RETRY_DELAY_S = 1.0
RECONNECT_BACKOFF_S = 5.0
PUBLISH_INTERVAL_MS = 1000
REALTIME_INTERVAL_S = 2.0
FAULT_INTERVAL_S = 5.0
SETTINGS_INTERVAL_S = 30.0
INFO_INTERVAL_S = 60.0
SOLAR_DISCOVERY_INTERVAL_S = 15.0
STALE_TIMEOUT_S = 6.0
PV_PRESENT_THRESHOLD_V = 5.0

MODBUS_FC_READ_HOLDING = 0x03
MODBUS_FC_WRITE_MULTIPLE = 0x10

REALTIME_START = 201
REALTIME_COUNT = 34
FAULT_REG = 100
WARNING_REG = 104
WARNING_MASKED_REG = 108
DEVICE_TYPE_REG = 171
DEVICE_NAME_REG = 172
DEVICE_NAME_LEN = 12
PROTOCOL_NUMBER_REG = 184
SERIAL_NUMBER_REG = 186
SERIAL_NUMBER_LEN = 12
FIRMWARE_REG = 626
FIRMWARE_LEN = 8
RATED_POWER_REG = 643
RATED_CELL_COUNT_REG = 644
SETTINGS_START = 300
SETTINGS_COUNT = 44
MAX_DISCHARGE_CURRENT_REG = 351
GRID_RECOGNITION_TIME_REG = 355
REMOTE_SWITCH_REG = 420
CLEAR_FAULTS_REG = 426
MAX_CHARGE_VOLTAGE_REG = 324
MAX_CHARGE_CURRENT_REG = 332
MAX_GRID_CHARGE_CURRENT_REG = 333
CHARGE_PRIORITY_REG = 331
BATTERY_TYPE_REG = 322
OFFGRID_LOW_PROTECTION_REG = 329
GRID_LOW_PROTECTION_REG = 327

MODE_BOOT = 0
MODE_STANDBY = 1
MODE_GRID = 2
MODE_OFFGRID = 3
MODE_BYPASS = 4
MODE_CHARGE = 5
MODE_FAULT = 6

POWER_FLOW_PV_PRESENT = 0x0001
POWER_FLOW_GRID_PRESENT = 0x0004
POWER_FLOW_BATTERY_CHARGING = 0x0010
POWER_FLOW_BATTERY_DISCHARGING = 0x0020
POWER_FLOW_OUTPUT_HAS_LOAD = 0x0040
POWER_FLOW_GRID_CHARGING = 0x0100
POWER_FLOW_PV_CHARGING = 0x0200

FAULT_BIT_NAMES = {
    1: "InverterOverTemp",
    2: "DCDCOverTemp",
    3: "BattOverVolt",
    4: "PVOverTemp",
    5: "OutputShort",
    6: "InverterOverVolt",
    7: "OutputOverload",
    8: "BusOverVolt",
    9: "BusSoftStartTimeout",
    10: "PVOverCurrent",
    11: "PVOverVolt",
    12: "BattOverCurrent",
    13: "InverterOverCurrent",
    14: "BusUnderVolt",
    16: "InverterDCComponentHigh",
    18: "OutputCurrentZeroOffset",
    19: "InverterCurrentZeroOffset",
    20: "BattCurrentZeroOffset",
    21: "PVCurrentZeroOffset",
    22: "InverterUnderVolt",
    23: "InverterNegativePower",
    24: "ParallelMasterLost",
    25: "ParallelSyncSignalAbnormal",
    26: "BattTypeIncompatible",
    27: "ParallelVersionIncompatible",
}

WARNING_BIT_NAMES = {
    0: "GridZeroCrossLost",
    1: "GridWaveformAbnormal",
    2: "GridOverVolt",
    3: "GridUnderVolt",
    4: "GridOverFreq",
    5: "GridUnderFreq",
    6: "PVUnderVolt",
    7: "OverTemp",
    8: "BattLow",
    9: "BattNotConnected",
    10: "Overload",
    11: "BattEqCharging",
    12: "BattDischargedNotRecovered",
    13: "OutputDerating",
    14: "FanStuck",
    15: "PVEnergyTooLow",
    16: "ParallelCommInterrupt",
    17: "ParallelOutputModeMismatch",
    18: "ParallelBattVoltageDiff",
    19: "LithiumCommError",
    20: "BattDischargeCurrentExceeded",
}

LOG_PATH_CANDIDATES = [
    "/var/log/dbus-anenji/driver.log",
    "/data/etc/dbus-anenji/dbus-anenji.log",
    "/tmp/dbus-anenji.log",
]
LOG_MAX_BYTES = 512_000

ALARM_OK = 0
ALARM_WARNING = 1
ALARM_ALARM = 2


class RegisterConversionError(ValueError):
    """Raised when a D-Bus write cannot be converted to a Modbus value."""


def setup_logging() -> logging.Logger:
    logger = logging.getLogger("anenji")
    if logger.handlers:
        return logger

    logger.setLevel(logging.INFO)
    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)-8s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    for candidate in LOG_PATH_CANDIDATES:
        directory = os.path.dirname(candidate)
        try:
            if directory:
                os.makedirs(directory, exist_ok=True)
            file_handler = RotatingFileHandler(candidate, maxBytes=LOG_MAX_BYTES, backupCount=4)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
            logger.info("Logging to %s", candidate)
            break
        except Exception:
            continue

    return logger


log = setup_logging()


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def to_signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def decode_ascii(registers: Iterable[int]) -> str:
    payload = bytearray()
    for reg in registers:
        payload.extend(struct.pack(">H", reg & 0xFFFF))
    return payload.decode("ascii", errors="ignore").strip("\x00 ")


def decode_u32(registers: List[int]) -> int:
    if len(registers) != 2:
        raise ValueError("Expected exactly two registers for a 32-bit value")
    return ((registers[0] & 0xFFFF) << 16) | (registers[1] & 0xFFFF)


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def dbus_to_native(value: Any) -> Any:
    if isinstance(value, (dbus.Int16, dbus.Int32, dbus.Int64, dbus.UInt16, dbus.UInt32, dbus.UInt64, dbus.Byte)):
        return int(value)
    if isinstance(value, dbus.Double):
        return float(value)
    if isinstance(value, dbus.Boolean):
        return bool(value)
    if isinstance(value, dbus.String):
        return str(value)
    return value


def working_mode_name(mode: int) -> str:
    return {
        MODE_BOOT: "Boot",
        MODE_STANDBY: "Standby",
        MODE_GRID: "Grid",
        MODE_OFFGRID: "OffGrid",
        MODE_BYPASS: "Bypass",
        MODE_CHARGE: "Charge",
        MODE_FAULT: "Fault",
    }.get(mode, f"Unknown({mode})")


def venus_state_from_mode(mode: int, battery_voltage: float, cv_voltage: float) -> int:
    if mode == MODE_BOOT:
        return 0
    if mode == MODE_STANDBY:
        return 1
    if mode == MODE_GRID:
        return 8
    if mode == MODE_OFFGRID:
        return 9
    if mode == MODE_BYPASS:
        return 8
    if mode == MODE_CHARGE:
        return 4 if cv_voltage > 0 and battery_voltage >= max(cv_voltage - 0.2, 0.0) else 3
    if mode == MODE_FAULT:
        return 2
    return 0


def first_set_bit_name(value: int, mapping: Dict[int, str]) -> Optional[str]:
    for bit, name in mapping.items():
        mask = 1 << bit if bit < 32 else 0
        if value & mask:
            return name
    return None


@dataclass
class DriverData:
    connected: bool = False
    last_success: float = 0.0
    port: Optional[str] = None
    modbus_address: Optional[int] = None
    working_mode: int = MODE_BOOT
    grid_voltage: float = 0.0
    grid_frequency: float = 0.0
    grid_power: int = 0
    inverter_voltage: float = 0.0
    inverter_current: float = 0.0
    inverter_frequency: float = 0.0
    inverter_power: int = 0
    inverter_charge_power: int = 0
    output_voltage: float = 0.0
    output_current: float = 0.0
    output_frequency: float = 0.0
    output_power: int = 0
    output_apparent_power: int = 0
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_power: int = 0
    pv_voltage: float = 0.0
    pv_current: float = 0.0
    pv_power: int = 0
    pv_charge_power: int = 0
    load_percent: int = 0
    dcdc_temp: int = 0
    inverter_temp: int = 0
    soc: int = 0
    power_flow_status: int = 0
    battery_current_filtered: float = 0.0
    inverter_charge_current: float = 0.0
    pv_charge_current: float = 0.0
    fault_bits: int = 0
    warning_bits: int = 0
    warning_masked_bits: int = 0
    device_type: int = 0
    device_name: str = ""
    protocol_number: int = 0
    serial_number: str = ""
    firmware_string: str = ""
    rated_power: int = 0
    rated_cell_count: int = 0
    settings: Dict[int, int] = field(default_factory=dict)
    info_ready: bool = False
    settings_ready: bool = False
    realtime_ready: bool = False
    faults_ready: bool = False

    @property
    def is_stale(self) -> bool:
        return self.last_success == 0 or (time.time() - self.last_success) > STALE_TIMEOUT_S

    @property
    def grid_connected(self) -> bool:
        return bool(self.power_flow_status & POWER_FLOW_GRID_PRESENT)

    @property
    def pv_present(self) -> bool:
        return self.pv_voltage > PV_PRESENT_THRESHOLD_V or bool(self.power_flow_status & POWER_FLOW_PV_PRESENT)

    @property
    def low_voltage_limit(self) -> float:
        raw = self.settings.get(OFFGRID_LOW_PROTECTION_REG) or self.settings.get(GRID_LOW_PROTECTION_REG) or 0
        return raw / 10.0

    @property
    def cv_voltage(self) -> float:
        return self.settings.get(MAX_CHARGE_VOLTAGE_REG, 0) / 10.0

    @property
    def max_charge_current_setting(self) -> float:
        return self.settings.get(MAX_CHARGE_CURRENT_REG, 0) / 10.0

    @property
    def max_grid_charge_current_setting(self) -> float:
        return self.settings.get(MAX_GRID_CHARGE_CURRENT_REG, 0) / 10.0

    @property
    def max_discharge_current_setting(self) -> float:
        return float(self.settings.get(MAX_DISCHARGE_CURRENT_REG, 0))

    @property
    def remote_switch(self) -> int:
        return self.settings.get(REMOTE_SWITCH_REG, 0)

    @property
    def dc_temperature(self) -> float:
        return float(self.inverter_temp if self.inverter_temp else self.dcdc_temp)


@dataclass
class PendingWrite:
    register: int
    values: List[int]
    reason: str


class SettingsCallbackDispatcher:
    def __init__(self) -> None:
        self.target: Optional[Callable[..., bool]] = None

    def __call__(self, *args: Any) -> bool:
        if self.target is None:
            return True
        return bool(self.target(*args))


class DbusValueReader:
    def __init__(self, bus: dbus.bus.BusConnection):
        self._bus = bus

    def read(self, service: str, path: str) -> Any:
        try:
            obj = self._bus.get_object(service, path)
            iface = dbus.Interface(obj, "com.victronenergy.BusItem")
            return dbus_to_native(iface.GetValue())
        except Exception:
            return None


class ExternalSolarObserver:
    def __init__(self, bus: dbus.bus.BusConnection):
        self._bus = bus
        self._reader = DbusValueReader(bus)
        self.last_scan = 0.0
        self.service_name: Optional[str] = None
        self.available_power = 0.0

    def refresh(self) -> None:
        now = time.time()
        if (now - self.last_scan) < SOLAR_DISCOVERY_INTERVAL_S:
            return
        self.last_scan = now

        try:
            dbus_obj = self._bus.get_object("org.freedesktop.DBus", "/org/freedesktop/DBus")
            dbus_iface = dbus.Interface(dbus_obj, "org.freedesktop.DBus")
            names = dbus_iface.ListNames()
        except Exception as exc:
            log.debug("External solar scan failed: %s", exc)
            return

        best_service = None
        best_power = 0.0
        for name in sorted(names):
            if not name.startswith("com.victronenergy.solarcharger"):
                continue
            if name == DBUS_SERVICE_SOLAR:
                continue
            power = self._reader.read(name, "/Yield/Power")
            if power in (None, ""):
                power = self._reader.read(name, "/Dc/0/Power")
            if power in (None, ""):
                continue
            power_f = safe_float(power, 0.0)
            if power_f >= best_power:
                best_power = power_f
                best_service = name

        if best_service:
            if best_service != self.service_name or abs(best_power - self.available_power) >= 1.0:
                log.info("External Victron solar service %s reports %.1fW", best_service, best_power)
            self.service_name = best_service
            self.available_power = best_power
        else:
            if self.service_name is not None:
                log.info("No external Victron solarcharger service currently reporting power")
            self.service_name = None
            self.available_power = 0.0


class AnenjiModbusClient:
    def __init__(self, configured_address: int) -> None:
        self.data = DriverData()
        self._configured_address = int(configured_address)
        self._ser: Optional[serial.Serial] = None
        self._locked_port: Optional[str] = None
        self._locked_address: Optional[int] = None
        self._lock = threading.Lock()
        self._queue_lock = threading.Lock()
        self._pending_writes: "OrderedDict[Tuple[int, Tuple[int, ...]], PendingWrite]" = OrderedDict()
        self._last_connect_attempt = 0.0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_realtime_poll = 0.0
        self._last_fault_poll = 0.0
        self._last_info_poll = 0.0
        self._last_settings_poll = 0.0
        self._dvcc_seen = False
        self._last_priority_bias: Optional[int] = None
        self._serial_hint_logged = False

    # ── Serial-starter helpers ───────────────────────────────────────────────
    @staticmethod
    def _serial_starter_stop(port: str) -> None:
        tty = os.path.basename(port)
        script = "/opt/victronenergy/serial-starter/stop-tty.sh"
        try:
            result = subprocess.run([script, tty], timeout=5, capture_output=True, text=True)
            if result.returncode == 0:
                log.info("serial-starter: stopped %s", tty)
            else:
                log.warning("serial-starter stop-tty.sh returned %s: %s", result.returncode, result.stderr.strip())
        except FileNotFoundError:
            log.debug("serial-starter stop helper not found")
        except Exception as exc:
            log.warning("serial-starter stop helper failed: %s", exc)

    @staticmethod
    def _serial_starter_start(port: str) -> None:
        tty = os.path.basename(port)
        script = "/opt/victronenergy/serial-starter/start-tty.sh"
        try:
            result = subprocess.run([script, tty], timeout=5, capture_output=True, text=True)
            if result.returncode == 0:
                log.info("serial-starter: started %s", tty)
            else:
                log.warning("serial-starter start-tty.sh returned %s: %s", result.returncode, result.stderr.strip())
        except FileNotFoundError:
            log.debug("serial-starter start helper not found")
        except Exception as exc:
            log.warning("serial-starter start helper failed: %s", exc)

    # ── Low-level Modbus framing ─────────────────────────────────────────────
    @staticmethod
    def _build_read_frame(address: int, start_register: int, register_count: int) -> bytes:
        payload = struct.pack(">B B H H", address & 0xFF, MODBUS_FC_READ_HOLDING, start_register & 0xFFFF, register_count & 0xFFFF)
        crc = crc16_modbus(payload)
        return payload + struct.pack("<H", crc)

    @staticmethod
    def _build_write_frame(address: int, start_register: int, values: List[int]) -> bytes:
        register_count = len(values)
        body = bytearray(struct.pack(">B B H H B", address & 0xFF, MODBUS_FC_WRITE_MULTIPLE, start_register & 0xFFFF, register_count & 0xFFFF, register_count * 2))
        for value in values:
            body.extend(struct.pack(">H", value & 0xFFFF))
        crc = crc16_modbus(body)
        return bytes(body) + struct.pack("<H", crc)

    def _read_once(self, address: int, start_register: int, register_count: int) -> List[int]:
        if not self._ser or not self._ser.is_open:
            raise IOError("Serial port is not open")
        frame = self._build_read_frame(address, start_register, register_count)
        expected_len = 5 + (register_count * 2)
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        response = self._ser.read(expected_len)
        if len(response) != expected_len:
            raise IOError(f"Expected {expected_len} bytes, received {len(response)}")
        if response[0] != address:
            raise IOError(f"Unexpected Modbus address {response[0]}")
        if response[1] == (MODBUS_FC_READ_HOLDING | 0x80):
            raise IOError(f"Modbus exception 0x{response[2]:02X}")
        if response[1] != MODBUS_FC_READ_HOLDING:
            raise IOError(f"Unexpected function code 0x{response[1]:02X}")
        if response[2] != register_count * 2:
            raise IOError(f"Unexpected byte count {response[2]}")
        crc_rx = struct.unpack("<H", response[-2:])[0]
        if crc16_modbus(response[:-2]) != crc_rx:
            raise IOError("CRC mismatch")
        payload = response[3:-2]
        return list(struct.unpack(">" + ("H" * register_count), payload))

    def _write_once(self, address: int, start_register: int, values: List[int]) -> None:
        if not self._ser or not self._ser.is_open:
            raise IOError("Serial port is not open")
        frame = self._build_write_frame(address, start_register, values)
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        response = self._ser.read(8)
        if len(response) != 8:
            raise IOError(f"Expected 8 bytes, received {len(response)}")
        if response[0] != address:
            raise IOError(f"Unexpected Modbus address {response[0]}")
        if response[1] == (MODBUS_FC_WRITE_MULTIPLE | 0x80):
            raise IOError(f"Modbus exception 0x{response[2]:02X}")
        if response[1] != MODBUS_FC_WRITE_MULTIPLE:
            raise IOError(f"Unexpected function code 0x{response[1]:02X}")
        crc_rx = struct.unpack("<H", response[-2:])[0]
        if crc16_modbus(response[:-2]) != crc_rx:
            raise IOError("CRC mismatch")
        echoed_register, echoed_count = struct.unpack(">H H", response[2:6])
        if echoed_register != start_register or echoed_count != len(values):
            raise IOError("Write acknowledgement mismatch")

    # ── Discovery / connection ───────────────────────────────────────────────
    def _address_probe_order(self) -> List[int]:
        seen = set()
        order: List[int] = []
        for address in [self._configured_address] + list(range(1, 11)) + list(range(1, 248)):
            if 1 <= int(address) <= 247 and int(address) not in seen:
                seen.add(int(address))
                order.append(int(address))
        return order

    def _open_serial(self, port: str) -> serial.Serial:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=BYTE_SIZE,
            parity=PARITY,
            stopbits=STOP_BITS,
            timeout=SERIAL_TIMEOUT_S,
            dsrdtr=False,
            rtscts=False,
            exclusive=True,
        )
        ser.dtr = False
        ser.rts = False
        return ser

    def _probe_port(self, port: str) -> Optional[Tuple[serial.Serial, int]]:
        ser: Optional[serial.Serial] = None
        try:
            ser = self._open_serial(port)
            time.sleep(0.1)
            for address in self._address_probe_order():
                try:
                    registers = self._read_once_with_serial(ser, address, REALTIME_START, 1)
                    if registers:
                        mode = registers[0]
                        log.info("Confirmed ANENJI inverter on %s at address %s (mode=%s)", port, address, working_mode_name(mode))
                        return ser, address
                except Exception:
                    continue
        except Exception as exc:
            log.debug("Probe failed on %s: %s", port, exc)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        return None

    def _read_once_with_serial(self, ser: serial.Serial, address: int, start_register: int, register_count: int) -> List[int]:
        frame = self._build_read_frame(address, start_register, register_count)
        expected_len = 5 + (register_count * 2)
        ser.reset_input_buffer()
        ser.write(frame)
        response = ser.read(expected_len)
        if len(response) != expected_len:
            raise IOError("Short reply during probe")
        if response[0] != address or response[1] != MODBUS_FC_READ_HOLDING or response[2] != register_count * 2:
            raise IOError("Unexpected probe reply")
        crc_rx = struct.unpack("<H", response[-2:])[0]
        if crc16_modbus(response[:-2]) != crc_rx:
            raise IOError("Probe CRC mismatch")
        payload = response[3:-2]
        return list(struct.unpack(">" + ("H" * register_count), payload))

    def _find_initial_port(self) -> Optional[Tuple[serial.Serial, str, int]]:
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if not ports:
            log.warning("No USB serial ports found while scanning for the ANENJI inverter")
            return None
        for port in ports:
            log.info("Probing %s for Modbus address candidates", port)
            result = self._probe_port(port)
            if result is not None:
                ser, address = result
                return ser, port, address
        log.warning("No responding ANENJI inverter found on candidate USB serial ports")
        return None

    def _connect(self) -> bool:
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
            now = time.time()
            if (now - self._last_connect_attempt) < RECONNECT_BACKOFF_S:
                return False
            self._last_connect_attempt = now

        if self._locked_port and self._locked_address:
            try:
                ser = self._open_serial(self._locked_port)
                self._serial_starter_stop(self._locked_port)
                with self._lock:
                    self._ser = ser
                    self.data.connected = True
                    self.data.port = self._locked_port
                    self.data.modbus_address = self._locked_address
                log.info("Reconnected to locked port %s at address %s", self._locked_port, self._locked_address)
                return True
            except Exception as exc:
                log.warning("Reconnect to locked port %s failed: %s", self._locked_port, exc)
                return False

        discovered = self._find_initial_port()
        if discovered is None:
            return False

        ser, port, address = discovered
        self._serial_starter_stop(port)
        with self._lock:
            self._ser = ser
            self._locked_port = port
            self._locked_address = address
            self.data.connected = True
            self.data.port = port
            self.data.modbus_address = address
        log.info("Connected to %s at Modbus address %s", port, address)
        return True

    def _disconnect(self) -> None:
        with self._lock:
            ser = self._ser
            port = self.data.port
            self._ser = None
            self.data.connected = False
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if port:
            self._serial_starter_start(port)

    # ── Public state / control helpers ───────────────────────────────────────
    def queue_write(self, register: int, values: List[int], reason: str) -> None:
        key = (int(register), tuple(int(v) for v in values))
        with self._queue_lock:
            self._pending_writes[key] = PendingWrite(register=int(register), values=[int(v) for v in values], reason=reason)
        log.info("Queued write r%s=%s (%s)", register, values, reason)

    def mark_dvcc_seen(self) -> None:
        self._dvcc_seen = True

    def maybe_bias_charge_priority(self, external_pv_power: float) -> None:
        if not self._dvcc_seen or external_pv_power <= 0.0:
            return
        current_priority = self.data.settings.get(CHARGE_PRIORITY_REG)
        desired_priority = 1  # PV first
        if current_priority == desired_priority and self._last_priority_bias == desired_priority:
            return
        self.queue_write(CHARGE_PRIORITY_REG, [desired_priority], f"DVCC external solar bias ({external_pv_power:.1f}W)")
        self._last_priority_bias = desired_priority
        log.info(
            "Best-effort DVCC solar bias requested: external Victron solar %.1fW, writing charge priority=%s",
            external_pv_power,
            desired_priority,
        )

    def snapshot(self) -> DriverData:
        with self._lock:
            data = DriverData(**{field_name: getattr(self.data, field_name) for field_name in DriverData.__dataclass_fields__})
            data.settings = dict(self.data.settings)
            return data

    def update_configured_address(self, address: int) -> None:
        self._configured_address = int(address)
        if self._locked_address and self._locked_address != int(address):
            log.info(
                "Localsettings Modbus address changed to %s, but the active address is locked in RAM at %s until restart",
                address,
                self._locked_address,
            )

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, name="anenji-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._disconnect()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)

    # ── Polling loop ─────────────────────────────────────────────────────────
    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._connect():
                time.sleep(1.0)
                continue

            try:
                now = time.time()
                self._flush_pending_writes()

                if not self.data.info_ready or (now - self._last_info_poll) >= INFO_INTERVAL_S:
                    self._poll_device_info()
                    self._last_info_poll = now

                if not self.data.settings_ready or (now - self._last_settings_poll) >= SETTINGS_INTERVAL_S:
                    self._poll_settings()
                    self._last_settings_poll = now

                if not self.data.faults_ready or (now - self._last_fault_poll) >= FAULT_INTERVAL_S:
                    self._poll_faults()
                    self._last_fault_poll = now

                if (now - self._last_realtime_poll) >= REALTIME_INTERVAL_S:
                    self._poll_realtime()
                    self._last_realtime_poll = now

                time.sleep(0.1)
            except Exception as exc:
                log.warning("Polling loop error: %s", exc)
                self._disconnect()
                time.sleep(RETRY_DELAY_S)

    def _transact_read(self, start_register: int, register_count: int, label: str) -> List[int]:
        address = self._locked_address
        if not address:
            raise IOError("No locked Modbus address available")
        for attempt in range(1, RETRY_COUNT + 1):
            try:
                with self._lock:
                    return self._read_once(address, start_register, register_count)
            except Exception as exc:
                log.warning("Read %s failed on attempt %s/%s: %s", label, attempt, RETRY_COUNT, exc)
                if attempt < RETRY_COUNT:
                    time.sleep(RETRY_DELAY_S)
        self._disconnect()
        raise IOError(f"Read {label} failed after {RETRY_COUNT} attempts")

    def _transact_write(self, register: int, values: List[int], reason: str) -> None:
        address = self._locked_address
        if not address:
            raise IOError("No locked Modbus address available")
        for attempt in range(1, RETRY_COUNT + 1):
            try:
                with self._lock:
                    self._write_once(address, register, values)
                log.info("Write success r%s=%s (%s)", register, values, reason)
                if len(values) == 1:
                    self.data.settings[register] = values[0]
                return
            except Exception as exc:
                log.warning("Write r%s failed on attempt %s/%s (%s): %s", register, attempt, RETRY_COUNT, reason, exc)
                if attempt < RETRY_COUNT:
                    time.sleep(RETRY_DELAY_S)
        self._disconnect()
        raise IOError(f"Write r{register} failed after {RETRY_COUNT} attempts")

    def _flush_pending_writes(self) -> None:
        with self._queue_lock:
            pending = list(self._pending_writes.values())
            self._pending_writes.clear()
        for item in pending:
            try:
                self._transact_write(item.register, item.values, item.reason)
            except Exception as exc:
                log.warning("Ignoring write-back failure for %s: %s", item.reason, exc)

    def _poll_realtime(self) -> None:
        regs = self._transact_read(REALTIME_START, REALTIME_COUNT, "realtime")
        data = self.data
        data.working_mode = regs[0]
        data.grid_voltage = to_signed16(regs[1]) / 10.0
        data.grid_frequency = to_signed16(regs[2]) / 100.0
        data.grid_power = to_signed16(regs[3])
        data.inverter_voltage = to_signed16(regs[4]) / 10.0
        data.inverter_current = to_signed16(regs[5]) / 10.0
        data.inverter_frequency = to_signed16(regs[6]) / 100.0
        data.inverter_power = to_signed16(regs[7])
        data.inverter_charge_power = to_signed16(regs[8])
        data.output_voltage = to_signed16(regs[9]) / 10.0
        data.output_current = to_signed16(regs[10]) / 10.0
        data.output_frequency = to_signed16(regs[11]) / 100.0
        data.output_power = to_signed16(regs[12])
        data.output_apparent_power = to_signed16(regs[13])
        data.battery_voltage = to_signed16(regs[14]) / 10.0
        data.battery_current = to_signed16(regs[15]) / 10.0
        data.battery_power = to_signed16(regs[16])
        data.pv_voltage = to_signed16(regs[18]) / 10.0
        data.pv_current = to_signed16(regs[19]) / 10.0
        data.pv_power = to_signed16(regs[22])
        data.pv_charge_power = to_signed16(regs[23])
        data.load_percent = to_signed16(regs[24])
        data.dcdc_temp = to_signed16(regs[25])
        data.inverter_temp = to_signed16(regs[26])
        data.soc = max(0, min(100, regs[28]))
        data.power_flow_status = regs[30]
        data.battery_current_filtered = to_signed16(regs[31]) / 10.0
        data.inverter_charge_current = to_signed16(regs[32]) / 10.0
        data.pv_charge_current = to_signed16(regs[33]) / 10.0
        data.last_success = time.time()
        data.realtime_ready = True
        data.connected = True

    def _poll_faults(self) -> None:
        regs_fault = self._transact_read(FAULT_REG, 2, "faults")
        regs_warn = self._transact_read(WARNING_REG, 2, "warnings")
        regs_warn_masked = self._transact_read(WARNING_MASKED_REG, 2, "masked warnings")
        self.data.fault_bits = decode_u32(regs_fault)
        self.data.warning_bits = decode_u32(regs_warn)
        self.data.warning_masked_bits = decode_u32(regs_warn_masked)
        self.data.faults_ready = True

    def _poll_device_info(self) -> None:
        self.data.device_type = self._transact_read(DEVICE_TYPE_REG, 1, "device type")[0]
        self.data.device_name = decode_ascii(self._transact_read(DEVICE_NAME_REG, DEVICE_NAME_LEN, "device name"))
        self.data.protocol_number = self._transact_read(PROTOCOL_NUMBER_REG, 1, "protocol number")[0]
        self.data.serial_number = decode_ascii(self._transact_read(SERIAL_NUMBER_REG, SERIAL_NUMBER_LEN, "serial number"))
        self.data.firmware_string = decode_ascii(self._transact_read(FIRMWARE_REG, FIRMWARE_LEN, "firmware"))
        self.data.rated_power = self._transact_read(RATED_POWER_REG, 1, "rated power")[0]
        self.data.rated_cell_count = self._transact_read(RATED_CELL_COUNT_REG, 1, "rated cell count")[0]
        self.data.info_ready = True

    def _poll_settings(self) -> None:
        regs = self._transact_read(SETTINGS_START, SETTINGS_COUNT, "settings block")
        for index, value in enumerate(regs):
            self.data.settings[SETTINGS_START + index] = value
        self.data.settings[MAX_DISCHARGE_CURRENT_REG] = self._transact_read(MAX_DISCHARGE_CURRENT_REG, 1, "max discharge current")[0]
        self.data.settings[GRID_RECOGNITION_TIME_REG] = self._transact_read(GRID_RECOGNITION_TIME_REG, 1, "grid recognition time")[0]
        self.data.settings[REMOTE_SWITCH_REG] = self._transact_read(REMOTE_SWITCH_REG, 1, "remote switch")[0]
        self.data.settings_ready = True


class ServiceController:
    def __init__(self, bus: dbus.bus.BusConnection, client: AnenjiModbusClient, settings_device: SettingsDevice) -> None:
        self.bus = bus
        self.client = client
        self.settings_device = settings_device
        self.external_solar = ExternalSolarObserver(bus)
        self.vebus = self._build_vebus_service()
        self.solar_service = self._build_solar_service()

    # ── D-Bus helpers ────────────────────────────────────────────────────────
    def _reset_trigger(self, service: VeDbusService, path: str) -> bool:
        try:
            service[path] = 0
        except Exception:
            pass
        return False

    def _handle_local_setting_change(self, *args: Any) -> bool:
        setting = args[0] if len(args) > 0 else None
        old_value = args[1] if len(args) > 1 else None
        new_value = args[2] if len(args) > 2 else None
        log.info("Local setting changed: %s %s -> %s", setting, old_value, new_value)
        if setting == "ModbusAddress" and new_value is not None:
            self.client.update_configured_address(int(new_value))
        if setting == "DeviceInstance" and new_value is not None:
            self.vebus["/DeviceInstance"] = int(new_value)
        if setting == "SolarChargerInstance" and new_value is not None and self.solar_service is not None:
            self.solar_service["/DeviceInstance"] = int(new_value)
        return True

    def register_localsettings_callback(self) -> None:
        # SettingsDevice calls the callback provided during construction, but the
        # controller still exposes this method so main() keeps the wiring obvious.
        pass

    def _queue_setting_write(self, register: int, value: int, label: str) -> None:
        self.client.queue_write(register, [value], label)

    def _write_numeric_setting(self, path: str, value: Any, register: int, converter: Callable[[Any], int], label: str) -> bool:
        try:
            modbus_value = converter(value)
        except RegisterConversionError as exc:
            log.warning("Rejected %s write on %s: %s", label, path, exc)
            return False
        self._queue_setting_write(register, modbus_value, label)
        return True

    def _build_vebus_service(self) -> VeDbusService:
        service = VeDbusService(DBUS_SERVICE_VEBUS, bus=self.bus, register=False)
        service.add_path("/Mgmt/ProcessName", __file__)
        service.add_path("/Mgmt/ProcessVersion", VERSION)
        service.add_path("/Mgmt/Connection", "USB RS-485 (FT232RL/SP485EEN auto-detect)")
        service.add_path("/DeviceInstance", int(self.settings_device["DeviceInstance"]))
        service.add_path("/ProductId", PRODUCT_ID)
        service.add_path("/ProductName", PRODUCT_NAME)
        service.add_path("/FirmwareVersion", VERSION)
        service.add_path("/Connected", 0)

        service.add_path("/Ac/ActiveIn/L1/V", None)
        service.add_path("/Ac/ActiveIn/L1/F", None)
        service.add_path("/Ac/ActiveIn/L1/P", None)
        service.add_path("/Ac/ActiveIn/L1/I", None)
        service.add_path("/Ac/ActiveIn/ActiveInputVoltage", None)
        service.add_path("/Ac/Out/L1/V", None)
        service.add_path("/Ac/Out/L1/F", None)
        service.add_path("/Ac/Out/L1/P", None)
        service.add_path("/Ac/Out/L1/I", None)
        service.add_path("/Ac/Out/L1/S", None)
        service.add_path("/Ac/ActiveIn/Connected", 0)
        service.add_path(
            "/Ac/ActiveIn/CurrentLimit",
            None,
            writeable=True,
            onchangecallback=lambda path, value: self._write_numeric_setting(path, value, MAX_GRID_CHARGE_CURRENT_REG, amps_tenth, "AC input current limit"),
        )

        service.add_path("/Dc/0/Voltage", None)
        service.add_path("/Dc/0/Current", None)
        service.add_path("/Dc/0/Power", None)
        service.add_path("/Dc/0/Temperature", None)
        service.add_path("/Dc/0/MaxChargeCurrent", None, writeable=True,
                         onchangecallback=lambda path, value: self._handle_dc_max_charge_current(path, value))
        service.add_path("/Soc", None)
        service.add_path("/State", 0)
        service.add_path(
            "/Mode",
            4,
            writeable=True,
            onchangecallback=lambda path, value: self._handle_mode_write(path, value),
        )
        service.add_path("/VebusError", 0)

        service.add_path(
            "/BatteryOperationalLimits/MaxChargeVoltage",
            None,
            writeable=True,
            onchangecallback=lambda path, value: self._handle_bol_max_charge_voltage(path, value),
        )
        service.add_path(
            "/BatteryOperationalLimits/MaxChargeCurrent",
            None,
            writeable=True,
            onchangecallback=lambda path, value: self._handle_bol_max_charge_current(path, value),
        )
        service.add_path(
            "/BatteryOperationalLimits/MaxDischargeCurrent",
            None,
            writeable=True,
            onchangecallback=lambda path, value: self._handle_bol_max_discharge_current(path, value),
        )
        service.add_path("/BatteryOperationalLimits/BatteryLowVoltage", None)

        service.add_path("/Alarms/LowVoltage", ALARM_OK)
        service.add_path("/Alarms/HighVoltage", ALARM_OK)
        service.add_path("/Alarms/LowSoc", ALARM_OK)
        service.add_path("/Alarms/HighTemperature", ALARM_OK)
        service.add_path("/Alarms/Overload", ALARM_OK)
        service.add_path("/Alarms/InternalFailure", ALARM_OK)
        service.add_path("/Yield/Power", None)
        service.add_path("/Pv/V", None)

        service.add_path("/Settings/OutputPriority", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 301, enum_value, "Output priority"))
        service.add_path("/Settings/ChargePriority", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, CHARGE_PRIORITY_REG, enum_value, "Charge priority"))
        service.add_path("/Settings/MaxChargeCurrent", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, MAX_CHARGE_CURRENT_REG, amps_tenth, "Max charge current"))
        service.add_path("/Settings/MaxGridChargeCurrent", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, MAX_GRID_CHARGE_CURRENT_REG, amps_tenth, "Max grid charge current"))
        service.add_path("/Settings/BatteryType", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, BATTERY_TYPE_REG, enum_value, "Battery type"))
        service.add_path("/Settings/OutputVoltage", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 320, volts_tenth, "Output voltage"))
        service.add_path("/Settings/OutputFrequency", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 321, hz_hundredths, "Output frequency"))
        service.add_path("/Settings/BuzzerMode", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 303, enum_value, "Buzzer mode"))
        service.add_path("/Settings/EnergySaving", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 307, enum_value, "Energy saving"))
        service.add_path("/Settings/OverloadRestart", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 308, enum_value, "Overload auto restart"))
        service.add_path("/Settings/OverTempRestart", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, 309, enum_value, "Over-temperature auto restart"))
        service.add_path("/Settings/RemoteSwitch", None, writeable=True,
                         onchangecallback=lambda path, value: self._write_numeric_setting(path, value, REMOTE_SWITCH_REG, enum_value, "Remote switch"))

        service.add_path("/Control/RemoteOn", 0, writeable=True,
                         onchangecallback=lambda path, value: self._handle_control_trigger(service, path, value, REMOTE_SWITCH_REG, 1, "Remote on"))
        service.add_path("/Control/RemoteOff", 0, writeable=True,
                         onchangecallback=lambda path, value: self._handle_control_trigger(service, path, value, REMOTE_SWITCH_REG, 0, "Remote off"))
        service.add_path("/Control/ClearFaults", 0, writeable=True,
                         onchangecallback=lambda path, value: self._handle_control_trigger(service, path, value, CLEAR_FAULTS_REG, 1, "Clear faults"))

        service.register()
        return service

    def _build_solar_service(self) -> VeDbusService:
        service = VeDbusService(DBUS_SERVICE_SOLAR, bus=self.bus, register=False)
        service.add_path("/Mgmt/ProcessName", __file__)
        service.add_path("/Mgmt/ProcessVersion", VERSION)
        service.add_path("/Mgmt/Connection", "Internal MPPT via ANENJI Modbus RTU")
        service.add_path("/DeviceInstance", int(self.settings_device["SolarChargerInstance"]))
        service.add_path("/ProductId", PRODUCT_ID)
        service.add_path("/ProductName", f"{PRODUCT_NAME} MPPT")
        service.add_path("/FirmwareVersion", VERSION)
        service.add_path("/Connected", 0)
        service.add_path("/Pv/V", None)
        service.add_path("/Pv/I", None)
        service.add_path("/Yield/Power", None)
        service.add_path("/Dc/0/Voltage", None)
        service.add_path("/Dc/0/Current", None)
        service.add_path("/Dc/0/Power", None)
        service.register()
        log.info("Registered companion solarcharger service %s", DBUS_SERVICE_SOLAR)
        return service

    # ── Writable D-Bus handlers ──────────────────────────────────────────────
    def _handle_control_trigger(self, service: VeDbusService, path: str, value: Any, register: int, register_value: int, label: str) -> bool:
        if enum_value(value) != 1:
            return False
        self.client.queue_write(register, [register_value], label)
        GLib.idle_add(self._reset_trigger, service, path)
        return True

    def _handle_mode_write(self, path: str, value: Any) -> bool:
        try:
            requested = enum_value(value)
        except RegisterConversionError as exc:
            log.warning("Rejected /Mode write on %s: %s", path, exc)
            return False
        register_value = 0 if requested == 4 else 1
        self.client.queue_write(REMOTE_SWITCH_REG, [register_value], f"Mode write {requested}")
        return True

    def _handle_bol_max_charge_voltage(self, path: str, value: Any) -> bool:
        return self._write_numeric_setting(path, value, MAX_CHARGE_VOLTAGE_REG, volts_tenth, "DVCC max charge voltage")

    def _handle_bol_max_charge_current(self, path: str, value: Any) -> bool:
        self.client.mark_dvcc_seen()
        try:
            modbus_value = amps_tenth(value)
        except RegisterConversionError as exc:
            log.warning("Rejected DVCC max charge current write on %s: %s", path, exc)
            return False
        self.client.queue_write(MAX_CHARGE_CURRENT_REG, [modbus_value], "DVCC max charge current")
        return True

    def _handle_dc_max_charge_current(self, path: str, value: Any) -> bool:
        self.client.mark_dvcc_seen()
        return self._write_numeric_setting(path, value, MAX_CHARGE_CURRENT_REG, amps_tenth, "DVCC DC max charge current")

    def _handle_bol_max_discharge_current(self, path: str, value: Any) -> bool:
        return self._write_numeric_setting(path, value, MAX_DISCHARGE_CURRENT_REG, amps_whole, "DVCC max discharge current")

    # ── Publishing ───────────────────────────────────────────────────────────
    def publish(self) -> bool:
        self.external_solar.refresh()
        if self.external_solar.available_power > 0.0:
            self.client.maybe_bias_charge_priority(self.external_solar.available_power)

        data = self.client.snapshot()
        self._publish_vebus(data)
        self._publish_solar(data)
        return True

    def _publish_vebus(self, data: DriverData) -> None:
        svc = self.vebus
        svc["/Connected"] = 1 if data.connected and not data.is_stale and data.realtime_ready else 0

        if not data.realtime_ready:
            return

        active_in_current = round(data.grid_power / data.grid_voltage, 2) if data.grid_voltage else 0.0
        svc["/Ac/ActiveIn/L1/V"] = round(data.grid_voltage, 1)
        svc["/Ac/ActiveIn/L1/F"] = round(data.grid_frequency, 2)
        svc["/Ac/ActiveIn/L1/P"] = int(data.grid_power)
        svc["/Ac/ActiveIn/L1/I"] = active_in_current
        svc["/Ac/ActiveIn/ActiveInputVoltage"] = round(data.grid_voltage, 1)
        svc["/Ac/Out/L1/V"] = round(data.output_voltage, 1)
        svc["/Ac/Out/L1/F"] = round(data.output_frequency, 2)
        svc["/Ac/Out/L1/P"] = int(data.output_power)
        svc["/Ac/Out/L1/I"] = round(data.output_current, 1)
        svc["/Ac/Out/L1/S"] = int(data.output_apparent_power)
        svc["/Ac/ActiveIn/Connected"] = 1 if data.grid_connected else 0
        svc["/Ac/ActiveIn/CurrentLimit"] = round(data.max_grid_charge_current_setting, 1) if data.settings_ready else None

        svc["/Dc/0/Voltage"] = round(data.battery_voltage, 1)
        svc["/Dc/0/Current"] = round(data.battery_current_filtered, 1)
        svc["/Dc/0/Power"] = int(round(data.battery_current_filtered * data.battery_voltage))
        svc["/Dc/0/Temperature"] = round(data.dc_temperature, 1)
        svc["/Dc/0/MaxChargeCurrent"] = round(data.max_charge_current_setting, 1) if data.settings_ready else None
        svc["/Soc"] = int(data.soc)
        svc["/State"] = venus_state_from_mode(data.working_mode, data.battery_voltage, data.cv_voltage)
        svc["/Mode"] = 4 if data.remote_switch == 0 else 3
        svc["/VebusError"] = 1 if data.fault_bits else 0

        svc["/BatteryOperationalLimits/MaxChargeVoltage"] = round(data.cv_voltage, 1) if data.settings_ready else None
        svc["/BatteryOperationalLimits/MaxChargeCurrent"] = round(data.max_charge_current_setting, 1) if data.settings_ready else None
        svc["/BatteryOperationalLimits/MaxDischargeCurrent"] = round(data.max_discharge_current_setting, 1) if data.settings_ready else None
        svc["/BatteryOperationalLimits/BatteryLowVoltage"] = round(data.low_voltage_limit, 1) if data.settings_ready else None

        svc["/Alarms/LowVoltage"] = alarm_low_voltage(data)
        svc["/Alarms/HighVoltage"] = alarm_high_voltage(data)
        svc["/Alarms/LowSoc"] = ALARM_ALARM if data.soc <= 5 else ALARM_WARNING if data.soc <= 10 else ALARM_OK
        svc["/Alarms/HighTemperature"] = alarm_high_temperature(data)
        svc["/Alarms/Overload"] = alarm_overload(data)
        svc["/Alarms/InternalFailure"] = alarm_internal_failure(data)
        svc["/Yield/Power"] = int(data.pv_power)
        svc["/Pv/V"] = round(data.pv_voltage, 1)

        if data.settings_ready:
            svc["/Settings/OutputPriority"] = data.settings.get(301)
            svc["/Settings/ChargePriority"] = data.settings.get(CHARGE_PRIORITY_REG)
            svc["/Settings/MaxChargeCurrent"] = round(data.max_charge_current_setting, 1)
            svc["/Settings/MaxGridChargeCurrent"] = round(data.max_grid_charge_current_setting, 1)
            svc["/Settings/BatteryType"] = data.settings.get(BATTERY_TYPE_REG)
            svc["/Settings/OutputVoltage"] = round(data.settings.get(320, 0) / 10.0, 1)
            svc["/Settings/OutputFrequency"] = round(data.settings.get(321, 0) / 100.0, 2)
            svc["/Settings/BuzzerMode"] = data.settings.get(303)
            svc["/Settings/EnergySaving"] = data.settings.get(307)
            svc["/Settings/OverloadRestart"] = data.settings.get(308)
            svc["/Settings/OverTempRestart"] = data.settings.get(309)
            svc["/Settings/RemoteSwitch"] = data.settings.get(REMOTE_SWITCH_REG)

        svc["/ProductName"] = data.device_name or PRODUCT_NAME
        svc["/FirmwareVersion"] = data.firmware_string or VERSION
        svc["/Mgmt/Connection"] = connection_string(data)

    def _publish_solar(self, data: DriverData) -> None:
        svc = self.solar_service
        svc["/Connected"] = 1 if data.connected and not data.is_stale else 0
        svc["/Pv/V"] = round(data.pv_voltage, 1)
        svc["/Pv/I"] = round(data.pv_current, 1)
        svc["/Yield/Power"] = int(data.pv_power)
        svc["/Dc/0/Voltage"] = round(data.battery_voltage, 1)
        svc["/Dc/0/Current"] = round(data.pv_charge_current, 1)
        svc["/Dc/0/Power"] = int(data.pv_charge_power)
        svc["/ProductName"] = f"{data.device_name or PRODUCT_NAME} MPPT"
        svc["/FirmwareVersion"] = data.firmware_string or VERSION
        svc["/Mgmt/Connection"] = connection_string(data)


def connection_string(data: DriverData) -> str:
    if data.port and data.modbus_address:
        return f"RS-485 {data.port} addr {data.modbus_address}"
    if data.port:
        return f"RS-485 {data.port}"
    return "USB RS-485 (auto-detect)"


def alarm_low_voltage(data: DriverData) -> int:
    if data.fault_bits & (1 << 12):
        return ALARM_ALARM
    if data.warning_bits & (1 << 8):
        return ALARM_WARNING
    if data.low_voltage_limit and data.battery_voltage and data.battery_voltage <= data.low_voltage_limit:
        return ALARM_WARNING
    return ALARM_OK


def alarm_high_voltage(data: DriverData) -> int:
    if data.fault_bits & (1 << 3):
        return ALARM_ALARM
    return ALARM_OK


def alarm_high_temperature(data: DriverData) -> int:
    if data.fault_bits & ((1 << 1) | (1 << 2) | (1 << 4)):
        return ALARM_ALARM
    if data.warning_bits & (1 << 7):
        return ALARM_WARNING
    return ALARM_OK


def alarm_overload(data: DriverData) -> int:
    if data.fault_bits & ((1 << 5) | (1 << 7)):
        return ALARM_ALARM
    if data.warning_bits & (1 << 10):
        return ALARM_WARNING
    return ALARM_OK


def alarm_internal_failure(data: DriverData) -> int:
    if data.fault_bits:
        return ALARM_ALARM
    return ALARM_OK


def enum_value(value: Any) -> int:
    try:
        return int(float(value))
    except Exception as exc:
        raise RegisterConversionError(str(exc))


def volts_tenth(value: Any) -> int:
    try:
        return int(round(float(value) * 10.0))
    except Exception as exc:
        raise RegisterConversionError(str(exc))


def hz_hundredths(value: Any) -> int:
    try:
        return int(round(float(value) * 100.0))
    except Exception as exc:
        raise RegisterConversionError(str(exc))


def amps_tenth(value: Any) -> int:
    try:
        return int(round(float(value) * 10.0))
    except Exception as exc:
        raise RegisterConversionError(str(exc))


def amps_whole(value: Any) -> int:
    try:
        return int(round(float(value)))
    except Exception as exc:
        raise RegisterConversionError(str(exc))


def build_localsettings(bus: dbus.bus.BusConnection, callback: Callable[..., bool]) -> SettingsDevice:
    supported_settings = {
        "ModbusAddress": [f"{LOCALSETTINGS_ROOT}/ModbusAddress", 1, 1, 247],
        "DeviceInstance": [f"{LOCALSETTINGS_ROOT}/DeviceInstance", 276, 0, 9999],
        "SolarChargerInstance": [f"{LOCALSETTINGS_ROOT}/SolarChargerInstance", 10, 0, 9999],
    }
    return SettingsDevice(bus=bus, supportedSettings=supported_settings, eventCallback=callback)


def main() -> None:
    log.info("=" * 72)
    log.info("ANENJI Venus OS driver v%s starting", VERSION)
    log.info("=" * 72)

    DBusGMainLoop(set_as_default=True)
    bus = dbus.SessionBus() if "DBUS_SESSION_BUS_ADDRESS" in os.environ else dbus.SystemBus()

    settings_callback = SettingsCallbackDispatcher()
    settings_device = build_localsettings(bus, settings_callback)
    client = AnenjiModbusClient(configured_address=int(settings_device["ModbusAddress"]))
    controller = ServiceController(bus, client, settings_device)
    settings_callback.target = controller._handle_local_setting_change

    client.start()
    GLib.timeout_add(PUBLISH_INTERVAL_MS, controller.publish)

    mainloop = GLib.MainLoop()

    def _shutdown(sig: int, _frame: Any) -> None:
        log.info("Signal %s received, shutting down", sig)
        client.stop()
        mainloop.quit()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    log.info("Waiting for Modbus data from the ANENJI inverter")
    mainloop.run()
    log.info("Driver stopped")


if __name__ == "__main__":
    main()
