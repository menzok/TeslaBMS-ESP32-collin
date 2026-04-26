#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ─────────────────────────────────────────────────────────────────────────────
#  dbus-teslabms.py  —  Standalone Venus OS battery driver for TeslaBMS-ESP32
#  Version 2.0.0
#
#  Protocol change v2.0: expanded payload (34 bytes, frame 37 bytes) and
#  ping-pong current injection in CMD_SEND_DATA.
#
#  Frame format (37 bytes total)
#  ──────────────────────────────
#  [0]      0xAA           start byte
#  [1..34]  payload        34-byte payload (see map below)
#  [35..36] CRC16/MODBUS   little-endian, computed over payload bytes only
#
#  Payload map (all multi-byte fields big-endian)
#  ──────────────────────────────────────────────
#  [0-1]   packV               uint16  × 100   → V   (10 mV res)
#  [2-3]   packI               int16   × 10    → A   (100 mA res, + = charge)
#  [4]     soc                 uint8   0-100   %
#  [5]     temp                int8    °C      (signed, average)
#  [6-7]   power               int16   W
#  [8-9]   avgCell             uint16  × 100   → V   (10 mV res)
#  [10-11] alarmFlags          uint16  bitmask
#  [12]    overlordState       uint8   0=Normal 1=Fault 2=StorageMode 3=Shutdown
#  [13]    contactorState      uint8   raw enum
#  [14-15] overVoltage         uint16  × 1000  → V   (1 mV res)
#  [16-17] underVoltage        uint16  × 1000  → V   (1 mV res)
#  [18]    overTemp            int8    °C      (signed)
#  [19]    underTemp           int8    °C      (signed)
#  [20]    numModules          uint8   total modules detected
#  [21]    numStrings          uint8   parallel string count
#  [22-23] overCurrentThresh   uint16  × 10    → A   (0.1A res)
#  [24]    statusFlags         uint8   bit0=currentSensorPresent bit1=balancingActive
#  [25]    activeFaultMask     uint8   1<<FaultEntry::Type per active fault
#  [26-27] lowestCellV         uint16  × 1000  → V   (1 mV res)
#  [28-29] highestCellV        uint16  × 1000  → V   (1 mV res)
#  [30]    minTemp             int8    °C      (signed)
#  [31]    maxTemp             int8    °C      (signed)
#  [32-33] reserved
#
#  CMD_SEND_DATA request frame (Venus → ESP32, 7 bytes)
#  ─────────────────────────────────────────────────────
#  [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]
#  curr    = shunt/SCS current × 10 (int16 BE, + = charge), 100 mA res
#  staleness = 0 if shunt data is fresh, increments each poll when unavailable
#  CRC     = CRC16/MODBUS over [0x03][curr_hi][curr_lo][staleness]
#
#  Hard-coded charge/discharge configuration (edit BmsConfig constants to change)
#  ───────────────────────────────────────────────────────────────────────────────
#  MaxChargeCurrent    = 200 A
#  MaxDischargeCurrent = 250 A
#  AbsorptionVoltage   = 4.10 V per cell
#  FloatVoltage        = 4.05 V per cell
#  TailCurrent         = 20 A
#  MaxChargeTemp       = 42 °C  /  MinChargeTemp = 5 °C
#  HighCellVoltage     = 4.10 V per cell (CCL taper onset)
#  ChargeVoltageMargin = 0.05 V per cell (CVL safety margin below OV trip)
#  LowCellVoltage      = 3.10 V per cell (DCL taper onset)
#  LowSocCutoff        = 5.0 %  /  LowSocDerateStart = 15.0 %
#
#  D-Bus command trigger paths (write 1 to activate, auto-clears to 0)
#  ────────────────────────────────────────────────────────────────────
#  /Control/Startup   — closes contactors (EXT_CMD_STARTUP)
#  /Control/Shutdown  — opens contactors  (EXT_CMD_SHUTDOWN)
#
#  Legacy switch path (write 0 or 1 to activate, auto-clears to None)
#  ────────────────────────────────────────────────────────────────────
#  /Switch  — write 1 = STARTUP, write 0 = SHUTDOWN; clears to None after sent
#
#  Contactor status (read-only, updated every publish cycle in all states)
#  ────────────────────────────────────────────────────────────────────────
#  /Contactor/State     — raw integer:  0=Open  1=Precharging  2=Connected  3=Fault
#  /Contactor/StateName — human string: "Open" | "Precharging" | "Connected" | "Fault"
#
#  Example command-line usage:
#    dbus -y com.victronenergy.battery.teslabms /Control/Startup  SetValue %1
#    dbus -y com.victronenergy.battery.teslabms /Control/Shutdown SetValue %1
#    dbus -y com.victronenergy.battery.teslabms /Contactor/StateName GetValue
#
# ─────────────────────────────────────────────────────────────────────────────

import os
import sys
import signal
import subprocess
import time
import threading
import struct
import serial
import glob
import logging

from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import dbus

# ── velib_python — use native Venus OS library (no external dependencies required) ──
_VELIB_PATHS = [
    "/opt/victronenergy/dbus-systemcalc-py/ext/velib_python",   # native Venus OS v3.x (preferred)
    "/opt/victronenergy/dbus-battery/ext/velib_python",          # venus-os-large variant
    "/data/apps/dbus-serialbattery/ext/velib_python",            # legacy fallback only
]
for _p in _VELIB_PATHS:
    if os.path.isdir(_p):
        sys.path.insert(1, _p)
        break
from vedbus import VeDbusService           # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────────────────────

AH_PER_MODULE    = 232.0   # Ah per Tesla module (6S 74P, 232 Ah)
CELLS_PER_MODULE = 6       # Tesla module is 6S — 6 cells in series per module

# ── Current sensor ────────────────────────────────────────────────────────────
# Set to True  if a current sensor is wired to the ESP32.
# Set to False if no current sensor is present (e.g. using a SmartShunt only).
# When False, /Dc/0/Current is NOT published so the SmartShunt reading on
# D-Bus is used by DVCC instead of being overwritten with a zero/stale value.
HAS_CURRENT_SENSOR = False

# ── Charge / discharge limits — hard-coded for this installation ──────────────
# These are the values used at runtime.  To change them, edit here and redeploy.
MAX_CHARGE_CURRENT    = 200.0   # A
MAX_DISCHARGE_CURRENT = 250.0   # A
ABSORPTION_VOLTAGE    = 4.10    # V per cell
FLOAT_VOLTAGE         = 4.05    # V per cell
TAIL_CURRENT          = 20.0    # A  (end-of-charge detection)
MAX_CHARGE_TEMP       = 42.0    # °C (charge inhibit above)
MIN_CHARGE_TEMP       = 5.0     # °C (charge inhibit below)
HIGH_CELL_VOLTAGE     = 4.10    # V per cell (CCL taper onset as highest cell approaches OV trip)
CHARGE_VOLTAGE_MARGIN = 0.05    # V per cell (CVL safety margin subtracted from eep_overvoltage)
LOW_CELL_VOLTAGE      = 3.10    # V per cell (DCL taper onset as lowest cell approaches UV trip)
LOW_SOC_CUTOFF        = 5.0     # % (hard DCL floor — zero discharge below this SOC)
LOW_SOC_DERATE_START  = 15.0    # % (SOC at which DCL begins linearly tapering toward zero)
COMMS_LOSS_CVL_PER_CELL = 4.0   # V/cell — safety CVL ceiling when BMS comms is lost; pack value = this × cell_count

# Timing
SERIAL_TIMEOUT_S    = 3.5    # read timeout inside send_command — generous margin above ESP32 response latency
PROBE_TIMEOUT_S     = 3.5    # per-port read timeout used only during port discovery — match SERIAL_TIMEOUT_S to avoid spurious probe failures on a slow or booting ESP32
POLL_INTERVAL_S     = 2.0    # how often Pi sends CMD 0x03 to request data
CMD_RETRIES         = 3      # number of attempts before declaring a command failed
CMD_RETRY_DELAY_S   = 1.0    # pause between retries — lets the ESP32 drain any converter noise before the next command arrives
STALE_TIMEOUT       = 5.0    # flag data as stale if no frame received within this window
OFFLINE_TIMEOUT     = 15.0   # flag as offline/cable-fault after this long with no frame
BAUD_RATE           = 115200
PUBLISH_INTERVAL_MS = 1000

# Shunt staleness — after this many consecutive polls with no fresh shunt
# data from the D-Bus SmartShunt, Venus sends staleness > 0 so the ESP32
# knows to fall back to OCV blending.
SHUNT_STALE_POLLS = 3   # increments each poll when shunt data is unavailable

DBUS_SERVICE_NAME = "com.victronenergy.battery.teslabms"

# ─────────────────────────────────────────────────────────────────────────────
#  Protocol constants
# ─────────────────────────────────────────────────────────────────────────────

EXT_CMD_SHUTDOWN  = 0x01
EXT_CMD_STARTUP   = 0x02
EXT_CMD_SEND_DATA = 0x03

FRAME_START_BYTE  = 0xAA
PAYLOAD_LEN       = 34
FRAME_LEN         = 1 + PAYLOAD_LEN + 2   # = 37

# CMD_SEND_DATA request carries shunt current (ping-pong)
SEND_DATA_CMD_LEN = 7   # [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]

ALARM_OVER_VOLTAGE  = (1 << 0)
ALARM_UNDER_VOLTAGE = (1 << 1)
ALARM_OVER_TEMP     = (1 << 2)
ALARM_UNDER_TEMP    = (1 << 3)
ALARM_OVER_CURRENT  = (1 << 4)
# bits 5-15: reserved, not yet implemented in ESP32 firmware

ALARM_OK      = 0
ALARM_WARNING = 1
ALARM_ALARM   = 2

OVERLORD_NORMAL   = 0
OVERLORD_FAULT    = 1
OVERLORD_STORAGE  = 2
OVERLORD_SHUTDOWN = 3   # clean shutdown — distinct from fault

# contactorState encoding (payload[13]) — mirrors ContactorController.h ContactorState enum
CONTACTOR_OPEN        = 0
CONTACTOR_PRECHARGING = 1
CONTACTOR_CONNECTED   = 2
CONTACTOR_FAULT       = 3

_CONTACTOR_STATE_NAMES = {
    CONTACTOR_OPEN:        "Open",
    CONTACTOR_PRECHARGING: "Precharging",
    CONTACTOR_CONNECTED:   "Connected",
    CONTACTOR_FAULT:       "Fault",
}

def contactor_state_name(state: int) -> str:
    return _CONTACTOR_STATE_NAMES.get(state, f"Unknown({state})")

# statusFlags bits (payload[24])
STATUS_CURRENT_SENSOR = (1 << 0)
STATUS_BALANCING      = (1 << 1)

# activeFaultMask bits (payload[25]) — 1<<FaultEntry::Type
# Type enum: None=0, OverVoltage=1, UnderVoltage=2, OverTemperature=3,
#            UnderTemperature=4, OverCurrent=5, CommsError=6
FAULT_OVER_VOLTAGE    = (1 << 1)
FAULT_UNDER_VOLTAGE   = (1 << 2)
FAULT_OVER_TEMP       = (1 << 3)
FAULT_UNDER_TEMP      = (1 << 4)
FAULT_OVER_CURRENT    = (1 << 5)
FAULT_COMMS_ERROR     = (1 << 6)

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
#  BmsConfig — hard-coded charge/discharge parameters for this installation
# ─────────────────────────────────────────────────────────────────────────────

class BmsConfig:
    """
    Fixed charge/discharge limits for this installation.
    All values are compiled into the driver — no GUI, no localsettings.
    To change a value, edit the constants above and redeploy the driver.

    The ESP32 EEPROM overcurrent threshold is still enforced as a hard upper
    bound on max_charge_current and max_discharge_current at runtime.
    """

    max_charge_current    = float(MAX_CHARGE_CURRENT)
    max_discharge_current = float(MAX_DISCHARGE_CURRENT)
    absorption_voltage    = float(ABSORPTION_VOLTAGE)
    float_voltage         = float(FLOAT_VOLTAGE)
    tail_current          = float(TAIL_CURRENT)
    max_charge_temp       = float(MAX_CHARGE_TEMP)
    min_charge_temp       = float(MIN_CHARGE_TEMP)
    high_cell_voltage     = float(HIGH_CELL_VOLTAGE)
    charge_voltage_margin = float(CHARGE_VOLTAGE_MARGIN)
    low_cell_voltage      = float(LOW_CELL_VOLTAGE)
    low_soc_cutoff        = float(LOW_SOC_CUTOFF)
    low_soc_derate_start  = float(LOW_SOC_DERATE_START)
    comms_loss_cvl_per_cell = float(COMMS_LOSS_CVL_PER_CELL)

    def effective_max_charge_current(self, overcurrent_threshold: float) -> float:
        return min(self.max_charge_current, overcurrent_threshold)

    def effective_max_discharge_current(self, overcurrent_threshold: float) -> float:
        return min(self.max_discharge_current, overcurrent_threshold)

    def pack_absorption_voltage(self, cell_count: int) -> float:
        return round(self.absorption_voltage * cell_count, 2)

    def pack_float_voltage(self, cell_count: int) -> float:
        return round(self.float_voltage * cell_count, 2)


# ─────────────────────────────────────────────────────────────────────────────
#  ShuntMonitor — reads SmartShunt current from D-Bus
# ─────────────────────────────────────────────────────────────────────────────

class ShuntMonitor:
    """
    Watches D-Bus for a battery monitor service (e.g. SmartShunt) that is
    NOT our own BMS service.  Reads /Dc/0/Current every poll cycle and
    tracks a staleness counter so the ESP32 knows when to ignore the value.
    """

    def __init__(self, bus):
        self._bus              = bus
        self._svc_name         = None   # cached D-Bus service name
        self._current_a        = 0.0
        self._staleness        = 255    # 255 = never received
        self._lock             = threading.Lock()

    def _find_service(self) -> "str | None":
        """Scan D-Bus for a battery monitor that is not our own service."""
        try:
            names = self._bus.list_names()
            for name in names:
                if (name.startswith("com.victronenergy.battery.") and
                        name != DBUS_SERVICE_NAME):
                    return name
        except Exception as exc:
            log.debug(f"ShuntMonitor._find_service: {exc}")
        return None

    def update(self) -> None:
        """Call once per poll cycle to refresh current and staleness."""
        with self._lock:
            # Re-scan for service if we don't have one cached
            if self._svc_name is None:
                self._svc_name = self._find_service()
                if self._svc_name:
                    log.info(f"ShuntMonitor: using {self._svc_name}")

            if self._svc_name is None:
                # No shunt service on D-Bus — increment staleness up to 255
                if self._staleness < 255:
                    self._staleness += 1
                return

            try:
                proxy = self._bus.get_object(self._svc_name, "/Dc/0/Current")
                iface = dbus.Interface(proxy, "com.victronenergy.BusItem")
                val = iface.GetValue()
                if val is None or isinstance(val, dbus.Array):
                    raise ValueError("no value")
                self._current_a = float(val)
                self._staleness = 0   # fresh
            except Exception as exc:
                log.debug(f"ShuntMonitor read failed ({self._svc_name}): {exc}")
                # Service may have disappeared — clear cache and mark stale
                self._svc_name = None
                if self._staleness < 255:
                    self._staleness += 1

    @property
    def current_a(self) -> float:
        with self._lock:
            return self._current_a

    @property
    def staleness(self) -> int:
        """0 = fresh this poll, >0 = number of consecutive failed polls."""
        with self._lock:
            return self._staleness


# ─────────────────────────────────────────────────────────────────────────────
#  TeslaBMSSerial — UART handler + decoded state (background thread)
# ─────────────────────────────────────────────────────────────────────────────

class TeslaBMSSerial:
    """
    Background daemon thread that:
      1. Auto-detects and opens the ESP32 serial port.
      2. Polls the ESP32 every POLL_INTERVAL_S via an extended CMD_SEND_DATA
         frame that carries the Venus shunt current (ping-pong exchange).
      3. Validates CRC-16/MODBUS on all received frames.
      4. Decodes all 34 payload bytes into public attributes.
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

        # ── Extended telemetry (v2.0) ─────────────────────────────────────────
        self.status_flags       = 0      # bit0=currentSensorPresent bit1=balancingActive
        self.active_fault_mask  = 0      # 1<<FaultEntry::Type per active fault
        self.lowest_cell_v      = 0.0
        self.highest_cell_v     = 0.0
        self.min_temp           = 25.0
        self.max_temp           = 25.0

        # ── EEPROM config (updated every frame) ───────────────────────────────
        self.eep_overvoltage        = 4.25
        self.eep_undervoltage       = 2.90
        self.eep_overtemp           = 60.0
        self.eep_undertemp          = -10.0
        self.eep_num_modules        = 2
        self.eep_num_strings        = 1
        self.eep_overcurrent_thresh = 350.0   # A — from EEPROM OVERCURRENT_THRESHOLD_A

        # ── Derived ───────────────────────────────────────────────────────────
        self.cell_count  = CELLS_PER_MODULE   # 1 module = 6 series cells; updated after first frame
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
        self._shunt_monitor        = None   # set by start()

    # ── Derived pack limits ───────────────────────────────────────────────────

    @property
    def pack_overvoltage(self):
        return round(self.eep_overvoltage * self.cell_count, 2)

    @property
    def pack_undervoltage(self):
        return round(self.eep_undervoltage * self.cell_count, 2)

    @property
    def current_sensor_present(self) -> bool:
        return bool(self.status_flags & STATUS_CURRENT_SENSOR)

    @property
    def balancing_active(self) -> bool:
        return bool(self.status_flags & STATUS_BALANCING)

    @property
    def charge_fet(self) -> bool:
        # Allow charge in Normal and StorageMode; inhibit in Fault and Shutdown
        return self.overlord_state not in (OVERLORD_FAULT, OVERLORD_SHUTDOWN)

    @property
    def discharge_fet(self) -> bool:
        return self.overlord_state not in (OVERLORD_FAULT, OVERLORD_SHUTDOWN)

    @property
    def is_stale(self) -> bool:
        if self.last_frame_time == 0:
            return True
        return (time.time() - self.last_frame_time) > OFFLINE_TIMEOUT

    @property
    def data_fresh(self) -> bool:
        """True if a valid frame was received within STALE_TIMEOUT seconds."""
        if self.last_frame_time == 0:
            return False
        return (time.time() - self.last_frame_time) <= STALE_TIMEOUT

    def alarm_level(self, bit: int) -> int:
        return ALARM_ALARM if (self.alarm_flags & bit) else ALARM_OK

    def fault_level(self, bit: int) -> int:
        return ALARM_ALARM if (self.active_fault_mask & bit) else ALARM_OK

    # ─── Serial-starter control ───────────────────────────────────────────────

    @staticmethod
    def _serial_starter_stop(port: str) -> None:
        tty = os.path.basename(port)
        script = "/opt/victronenergy/serial-starter/stop-tty.sh"
        try:
            result = subprocess.run([script, tty], timeout=5,
                                    capture_output=True, text=True)
            if result.returncode == 0:
                log.info(f"serial-starter: stopped {tty}")
            else:
                log.warning(f"serial-starter stop-tty.sh returned {result.returncode}: "
                            f"{result.stderr.strip()}")
        except FileNotFoundError:
            log.debug("serial-starter stop-tty.sh not found — skipping")
        except Exception as exc:
            log.warning(f"serial-starter stop-tty.sh error: {exc}")

    @staticmethod
    def _serial_starter_start(port: str) -> None:
        tty = os.path.basename(port)
        script = "/opt/victronenergy/serial-starter/start-tty.sh"
        try:
            result = subprocess.run([script, tty], timeout=5,
                                    capture_output=True, text=True)
            if result.returncode == 0:
                log.info(f"serial-starter: started {tty}")
            else:
                log.warning(f"serial-starter start-tty.sh returned {result.returncode}: "
                            f"{result.stderr.strip()}")
        except FileNotFoundError:
            log.debug("serial-starter start-tty.sh not found — skipping")
        except Exception as exc:
            log.warning(f"serial-starter start-tty.sh error: {exc}")

    # ─── Port detection ───────────────────────────────────────────────────────

    def _probe_port(self, port: str) -> "serial.Serial | None":
        """
        Open *port*, send an extended CMD_SEND_DATA frame (with zeroed shunt
        fields) and verify the reply.

        Returns the already-open serial handle (with timeout switched to
        SERIAL_TIMEOUT_S) on success so that _connect() can adopt it directly —
        no close/reopen, no further DTR/RTS transitions that would charge the
        ESP32 auto-reset RC circuit and cause a delayed reset.

        On failure the port is closed before returning None.
        """
        ser = None
        try:
            # dsrdtr=False / rtscts=False: keep DTR and RTS de-asserted so the
            # USB-UART bridge does not trigger the ESP32 auto-reset circuit.
            # exclusive=True (TIOCEXCL): lock the port immediately on open so
            # that ModemManager, udev, or any other process cannot open it
            # concurrently and cause a mid-session DTR/RTS glitch.
            ser = serial.Serial(port, BAUD_RATE, timeout=PROBE_TIMEOUT_S,
                                dsrdtr=False, rtscts=False, exclusive=True)
            ser.dtr = False   # explicit: do not pull EN low via auto-reset RC
            ser.rts = False
            log.info(f"Probing {port} …")
            # Brief settle so line stabilises after open without a device reset
            time.sleep(0.1)
            frame = self._build_send_data_frame(current_a=0.0, staleness=255)
            ser.reset_input_buffer()
            ser.write(frame)
            data = ser.read(FRAME_LEN)
            if len(data) >= FRAME_LEN and data[0] == FRAME_START_BYTE:
                payload = data[1: 1 + PAYLOAD_LEN]
                crc_rx  = struct.unpack("<H", data[1 + PAYLOAD_LEN: FRAME_LEN])[0]
                if crc16_modbus(payload) == crc_rx:
                    log.info(f"✅ TeslaBMS-ESP32 confirmed on {port}")
                    # Switch to operational timeout in-place — no close/reopen needed
                    ser.timeout = SERIAL_TIMEOUT_S
                    ser.reset_input_buffer()
                    return ser          # hand the open handle to _connect()
            log.debug(f"  {port}: no valid reply")
        except Exception as exc:
            log.debug(f"  {port}: {exc}")
        # Only close when the port is NOT being handed off to the caller
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        return None

    def _find_port(self) -> "serial.Serial | None":
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if not candidates:
            log.warning("No USB serial ports found.")
            return None
        for port in candidates:
            ser = self._probe_port(port)
            if ser is not None:
                return ser
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

        # _find_port() returns an already-open, confirmed serial handle so that
        # no second open() call (and its associated DTR/RTS transients) occurs.
        ser = self._find_port()
        if not ser:
            return False

        with self._lock:
            self._ser      = ser
            self._port     = ser.port
            self.connected = True
        log.info(f"✅ Connected on {ser.port}")
        # Block serial-starter from polling this port while we own it.
        self._serial_starter_stop(ser.port)
        return True

    def _disconnect(self) -> None:
        with self._lock:
            port = self._port          # capture before clearing
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser      = None
                self._port     = None
                self.connected = False
        # Re-enable serial-starter for this port now that we have released it.
        if port:
            self._serial_starter_start(port)

    # ─── Frame builders ───────────────────────────────────────────────────────

    @staticmethod
    def _build_send_data_frame(current_a: float, staleness: int) -> bytes:
        """
        Build the 7-byte extended CMD_SEND_DATA request that embeds the
        Venus shunt current so the ESP32 can use it for SOC integration.
        """
        cmd       = EXT_CMD_SEND_DATA
        curr_raw  = int(round(current_a * 10.0))
        curr_raw  = max(-32768, min(32767, curr_raw))    # clamp to int16
        stale_b   = max(0, min(255, int(staleness)))
        curr_be   = struct.pack(">h", curr_raw)          # big-endian int16
        crc_data  = bytes([cmd]) + curr_be + bytes([stale_b])
        crc       = crc16_modbus(crc_data)
        return bytes([FRAME_START_BYTE]) + crc_data + struct.pack("<H", crc)

    @staticmethod
    def _build_ctrl_frame(cmd: int) -> bytes:
        """Build the 4-byte control frame (SHUTDOWN / STARTUP)."""
        crc = crc16_modbus(bytes([cmd]))
        return bytes([FRAME_START_BYTE, cmd]) + struct.pack("<H", crc)

    # ─── Command send ─────────────────────────────────────────────────────────

    def send_command(self, cmd: int, current_a: float = 0.0,
                     staleness: int = 255) -> bool:
        """
        Transmit a command frame and, for CMD_SEND_DATA, wait for the telemetry
        reply.  Control commands (SHUTDOWN/STARTUP) do not generate a reply.
        Retries up to CMD_RETRIES times before disconnecting.
        """
        if cmd == EXT_CMD_SEND_DATA:
            frame_out    = self._build_send_data_frame(current_a, staleness)
            expect_reply = True
        else:
            frame_out    = self._build_ctrl_frame(cmd)
            expect_reply = False

        for attempt in range(1, CMD_RETRIES + 1):
            write_error_port = None   # set if a write error forces inline disconnect

            with self._lock:
                if not self._ser or not self._ser.is_open:
                    log.warning(f"send_command(0x{cmd:02X}): not connected")
                    return False
                try:
                    self._ser.reset_input_buffer()
                    # Write
                    self._ser.write(frame_out)
                    log.info(f"→ CMD 0x{cmd:02X} (attempt {attempt}/{CMD_RETRIES})"
                             + (f" shunt={current_a:+.1f}A stale={staleness}"
                                if cmd == EXT_CMD_SEND_DATA else ""))
                except Exception as exc:
                    log.error(f"send_command write error: {exc}")
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    write_error_port = self._port   # capture before clearing
                    self._ser      = None
                    self._port     = None
                    self.connected = False
                    # fall through — lock is released, then we restart serial-starter below

                # Read reply within the same lock — no gap where bytes can be interleaved
                if write_error_port is None and expect_reply:
                    try:
                        raw = self._ser.read(1)
                        if raw and raw[0] == FRAME_START_BYTE:
                            rest = self._ser.read(PAYLOAD_LEN + 2)
                            if len(rest) == PAYLOAD_LEN + 2:
                                payload = rest[:PAYLOAD_LEN]
                                crc_rx  = struct.unpack("<H", rest[PAYLOAD_LEN:])[0]
                                if crc16_modbus(payload) == crc_rx:
                                    self._parse_frame(payload)
                                    return True
                    except Exception as exc:
                        log.error(f"send_command read error (attempt {attempt}): {exc}")
                elif write_error_port is None and not expect_reply:
                    return True   # control commands need no reply

            # Re-enable serial-starter if we had a write error and cleared the port
            if write_error_port:
                self._serial_starter_start(write_error_port)
                return False

            log.warning(f"send_command(0x{cmd:02X}): no valid reply on attempt "
                        f"{attempt}/{CMD_RETRIES}")
            if attempt < CMD_RETRIES:
                time.sleep(CMD_RETRY_DELAY_S)

        # All retries exhausted
        log.error(f"send_command(0x{cmd:02X}): failed after {CMD_RETRIES} attempts — disconnecting")
        self._disconnect()
        return False

    # ─── Frame parser ─────────────────────────────────────────────────────────

    def _parse_frame(self, payload: bytes) -> bool:
        """Decode all 34 payload bytes and update instance attributes."""
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

            # ── Extended fields (v2.0) ─────────────────────────────────────
            self.status_flags      = payload[24]
            self.active_fault_mask = payload[25]

            low_cell_raw,  = struct.unpack_from(">H", payload, 26)
            high_cell_raw, = struct.unpack_from(">H", payload, 28)
            min_t,         = struct.unpack_from("b",  payload, 30)
            max_t,         = struct.unpack_from("b",  payload, 31)

            self.lowest_cell_v  = low_cell_raw  / 1000.0
            self.highest_cell_v = high_cell_raw / 1000.0
            self.min_temp       = float(min_t)
            self.max_temp       = float(max_t)

            # ── Derived topology ───────────────────────────────────────────
            # Each Tesla module is 6S — multiply modules-per-string by CELLS_PER_MODULE
            # to get the number of series cells used for CVL/CCL/DCL calculations.
            modules_per_string = max(self.eep_num_modules // self.eep_num_strings, 1)
            self.cell_count  = modules_per_string * CELLS_PER_MODULE
            self.capacity_ah = round(modules_per_string * AH_PER_MODULE, 1)

            self.last_frame_time = time.time()
            self.frame_count    += 1

            # ── Bookkeeping ───────────────────────────────────────────────────
            log.debug(
                f"#{self.frame_count}: "
                f"{self.voltage:.2f}V {self.current:+.1f}A "
                f"SOC={self.soc}% avg={self.temperature}°C "
                f"min={self.min_temp}°C max={self.max_temp}°C "
                f"loCell={self.lowest_cell_v:.3f}V hiCell={self.highest_cell_v:.3f}V "
                f"state={overlord} flags=0x{self.status_flags:02X} "
                f"faults=0x{self.active_fault_mask:02X} alarms=0x{alarm_flags:04X}"
            )
            return True

        except Exception as exc:
            log.error(f"_parse_frame: {exc}")
            return False

    # ─── Background reader loop ───────────────────────────────────────────────
    #
    # The ESP32 is purely request-reply — it never sends unsolicited frames.
    # This loop sleeps until the next poll interval, then sends an extended
    # CMD_SEND_DATA frame that carries the Venus SmartShunt current so the
    # ESP32 can use it for SOC integration (ping-pong exchange).
    # send_command handles the read, retry, and disconnect logic.
    # No "hunt for start byte" is needed or correct for this protocol.
    #

    def _reader_loop(self) -> None:
        log.info("Reader thread started.")
        while not self._stop_event.is_set():
            if not self._connect():
                time.sleep(1.0)
                continue
            try:
                now  = time.time()
                wait = POLL_INTERVAL_S - (now - self._last_poll_time)
                if wait > 0:
                    time.sleep(wait)
                self._last_poll_time = time.time()

                # Refresh shunt monitor and embed current in the poll frame
                if self._shunt_monitor:
                    self._shunt_monitor.update()
                    shunt_a   = -self._shunt_monitor.current_a
                    staleness = self._shunt_monitor.staleness
                else:
                    shunt_a, staleness = 0.0, 255

                self.send_command(EXT_CMD_SEND_DATA,
                                  current_a=shunt_a, staleness=staleness)

            except serial.SerialException as exc:
                log.error(f"Serial error: {exc} — reconnecting …")
                self._disconnect()
                time.sleep(2.0)
            except Exception as exc:
                log.error(f"Reader error: {exc}")
                self._disconnect()
                time.sleep(2.0)

        log.info("Reader thread stopped.")

    def start(self, shunt_monitor=None) -> None:
        self._shunt_monitor = shunt_monitor
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

def build_dbus_service(bus, cfg: "BmsConfig") -> VeDbusService:
    svc = VeDbusService(DBUS_SERVICE_NAME, bus=bus, register=False)

    # ── Management / identity ─────────────────────────────────────────────────
    svc.add_path("/Mgmt/ProcessName",    __file__)
    svc.add_path("/Mgmt/ProcessVersion", "2.0.0")
    svc.add_path("/Mgmt/Connection",     "USB Serial (auto-detect)")
    svc.add_path("/DeviceInstance",  1)
    svc.add_path("/ProductId",       0xBA77)
    svc.add_path("/ProductName",     "TeslaBMS-ESP32")
    svc.add_path("/FirmwareVersion", "2.0")
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
    svc.add_path("/Dc/0/Voltage",     None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.2f}V")
    svc.add_path("/Dc/0/Current",     None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Dc/0/Power",       None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.0f}W")
    svc.add_path("/Dc/0/Temperature", None, writeable=True)

    # ── Capacity ──────────────────────────────────────────────────────────────
    svc.add_path("/Capacity",          None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.2f}Ah")
    svc.add_path("/InstalledCapacity", None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.0f}Ah")
    svc.add_path("/ConsumedAmphours",  None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.0f}Ah")

    # ── CVL / CCL / DCL ───────────────────────────────────────────────────────
    svc.add_path("/Info/MaxChargeVoltage",    None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.2f}V")
    svc.add_path("/Info/BatteryLowVoltage",   None, writeable=False)
    svc.add_path("/Info/MaxChargeCurrent",    None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/MaxDischargeCurrent", None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.2f}A")
    svc.add_path("/Info/ChargeMode",          None, writeable=False)
    svc.add_path("/Info/TailCurrent",         None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.1f}A")

    # ── System / cell data ────────────────────────────────────────────────────
    svc.add_path("/System/NrOfCellsPerBattery",          None, writeable=True)
    svc.add_path("/System/NrOfModulesOnline",               0, writeable=True)
    svc.add_path("/System/NrOfModulesOffline",              1, writeable=True)
    svc.add_path("/System/NrOfModulesBlockingCharge",    None, writeable=True)
    svc.add_path("/System/NrOfModulesBlockingDischarge", None, writeable=True)
    svc.add_path("/System/MinCellVoltage",  None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/System/MaxCellVoltage",  None, writeable=True,
                 gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/System/MinVoltageCellId", None, writeable=True)
    svc.add_path("/System/MaxVoltageCellId", None, writeable=True)
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
    svc.add_path("/Config/CellOverVoltage",      None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/CellUnderVoltage",     None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.3f}V")
    svc.add_path("/Config/OverTemp",             None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/UnderTemp",            None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.1f}°C")
    svc.add_path("/Config/NumModules",           None, writeable=False)
    svc.add_path("/Config/NumStrings",           None, writeable=False)
    svc.add_path("/Config/CellCount",            None, writeable=False)
    svc.add_path("/Config/CapacityAh",           None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.1f}Ah")
    svc.add_path("/Config/OverCurrentThreshold", None, writeable=False,
                 gettextcallback=lambda p, v: f"{v:.1f}A")

    # ── Contactor status (read-only) ──────────────────────────────────────────
    # Reflects the contactor state reported by the ESP32 each poll cycle.
    # /Contactor/State  — raw integer: 0=Open 1=Precharging 2=Connected 3=Fault
    # /Contactor/StateName — human-readable string for Node-RED display / logic
    # Both paths are updated in every publish cycle (online, stale, and offline)
    # so Node-RED always receives the most-current known value.
    svc.add_path("/Contactor/State",     0,      writeable=False)
    svc.add_path("/Contactor/StateName", "Open", writeable=False)

    # ── Contactor control triggers (write 1 to activate, auto-clears to 0) ──
    # Use from command line:
    #   dbus -y com.victronenergy.battery.teslabms /Control/Startup  SetValue %1
    #   dbus -y com.victronenergy.battery.teslabms /Control/Shutdown SetValue %1
    # Or from Node-RED via a generic dbus write node.
    svc.add_path("/Control/Startup",  0, writeable=True)
    svc.add_path("/Control/Shutdown", 0, writeable=True)

    # ── Legacy switch path (kept for Node-RED compatibility) ──────────────────
    # Write 1 = request STARTUP, write 0 = request SHUTDOWN.
    # Initialized to None (neutral) so the driver takes no action on startup.
    # After the command is sent the path is cleared back to None (one-shot),
    # preventing repeated serial writes that would block the GLib main loop.
    # The actual contactor state is always visible on /Contactor/State above.
    svc.add_path("/Switch", None, writeable=True)

    svc.register()
    return svc


# ─────────────────────────────────────────────────────────────────────────────
#  Dynamic CVL / CCL / DCL helpers
# ─────────────────────────────────────────────────────────────────────────────

def calc_dynamic_cvl(bms: TeslaBMSSerial, cfg: BmsConfig) -> float:
    """
    Dynamic Charge Voltage Limit (CVL).

    Strategy: start from the user's absorption voltage target and pull it
    down so that no individual cell exceeds the EEPROM over-voltage setpoint.

      cell_spread  = highest_cell_v - avg_cell_v
      safe_cell_v  = eep_overvoltage - cell_spread - charge_voltage_margin
      dynamic_cvl  = safe_cell_v × cell_count   (clamped to float … absorption)

    When cells are well-balanced cell_spread ≈ 0 and CVL approaches the full
    absorption target.  As the highest cell diverges, CVL is reduced to protect
    it without tripping the over-voltage alarm.
    """
    cell_count = max(bms.cell_count, 1)

    float_pack      = cfg.pack_float_voltage(cell_count)
    absorption_pack = cfg.pack_absorption_voltage(cell_count)

    # In storage mode the BMS wants to hold a lower SOC.  Target float voltage
    # so chargers do not push to the full absorption ceiling.
    if bms.overlord_state == OVERLORD_STORAGE:
        return float_pack

    # Fall back to avg if per-cell data not yet received
    hi = bms.highest_cell_v if bms.highest_cell_v > 0.0 else bms.avg_cell_volt
    av = bms.avg_cell_volt   if bms.avg_cell_volt  > 0.0 else hi

    spread      = max(hi - av, 0.0)
    safe_cell   = bms.eep_overvoltage - spread - cfg.charge_voltage_margin   # margin keeps CVL below hard OV trip
    dynamic_cvl = round(safe_cell * cell_count, 2)

    return max(float_pack, min(absorption_pack, dynamic_cvl))


# Temperature derating: linearly taper current in a ±DERATE_ZONE window
# around each temperature limit.
_DERATE_ZONE_C = 5.0   # °C below MaxChargeTemp / above MinChargeTemp


def calc_ccl(bms: TeslaBMSSerial, cfg: BmsConfig) -> float:
    """Charge Current Limit with temperature and high cell voltage derating."""
    if not bms.charge_fet:
        return 0.0

    max_ccl = cfg.effective_max_charge_current(bms.eep_overcurrent_thresh)
    t = bms.temperature   # average temp from ESP32

    # ── 1. Temperature gate ───────────────────────────────────────────────────
    if t > cfg.max_charge_temp or t < cfg.min_charge_temp:
        return 0.0

    # ── 2. High temperature taper ─────────────────────────────────────────────
    if t > cfg.max_charge_temp - _DERATE_ZONE_C:
        factor = (cfg.max_charge_temp - t) / _DERATE_ZONE_C
        max_ccl *= max(0.0, min(1.0, factor))

    # ── 3. Low temperature taper ──────────────────────────────────────────────
    if t < cfg.min_charge_temp + _DERATE_ZONE_C:
        factor = (t - cfg.min_charge_temp) / _DERATE_ZONE_C
        max_ccl *= max(0.0, min(1.0, factor))

    # ── 4. High cell voltage taper (highest cell approaching ESP32 OV trip) ───
    # Tapers CCL linearly from full at HighCellVoltage down to zero at eep_overvoltage.
    # Prevents the charger slamming full current into the hard OV trip threshold.
    hi_cell = bms.highest_cell_v
    if hi_cell > 0.0:   # 0.0 means not yet received — skip until first frame
        if hi_cell >= bms.eep_overvoltage:
            return 0.0
        if hi_cell > cfg.high_cell_voltage:
            factor = (bms.eep_overvoltage - hi_cell) / max(bms.eep_overvoltage - cfg.high_cell_voltage, 0.001)
            max_ccl *= max(0.0, min(1.0, factor))

    return round(max_ccl, 1)


def calc_dcl(bms: TeslaBMSSerial, cfg: BmsConfig) -> float:
    """Discharge Current Limit with SOC, cell voltage, and temperature derating."""
    if not bms.discharge_fet:
        return 0.0

    max_dcl = cfg.effective_max_discharge_current(bms.eep_overcurrent_thresh)

    # ── 1. Hard SOC floor ─────────────────────────────────────────────────────
    # Zero DCL entirely below LowSocCutoff to prevent deep discharge even if
    # cell voltages still look OK (resting bounce effect).
    if bms.soc <= cfg.low_soc_cutoff:
        return 0.0

    # ── 2. SOC taper ──────────────────────────────────────────────────────────
    # Linear ramp from full DCL at LowSocDerateStart down to zero at LowSocCutoff.
    # Gives the inverter time to react gracefully rather than a sudden cutoff.
    if bms.soc < cfg.low_soc_derate_start:
        factor = (bms.soc - cfg.low_soc_cutoff) / max(cfg.low_soc_derate_start - cfg.low_soc_cutoff, 1.0)
        max_dcl *= max(0.0, min(1.0, factor))

    # ── 3. Cell voltage taper (lowest cell approaching ESP32 UV hard trip) ────
    # Tapers DCL linearly from full at LowCellVoltage down to zero at eep_undervoltage.
    lo_cell = bms.lowest_cell_v
    if lo_cell > 0.0:   # 0.0 means not yet received — skip until first frame
        if lo_cell <= bms.eep_undervoltage:
            return 0.0
        if lo_cell < cfg.low_cell_voltage:
            factor = (lo_cell - bms.eep_undervoltage) / max(cfg.low_cell_voltage - bms.eep_undervoltage, 0.001)
            max_dcl *= max(0.0, min(1.0, factor))

    # ── 4. Low temperature taper ──────────────────────────────────────────────
    t = bms.min_temp   # use minimum module temp for conservative derating
    if t < bms.eep_undertemp:
        return 0.0
    if t < bms.eep_undertemp + _DERATE_ZONE_C:
        factor = (t - bms.eep_undertemp) / _DERATE_ZONE_C
        max_dcl *= max(0.0, min(1.0, factor))

    return round(max_dcl, 1)


# ─────────────────────────────────────────────────────────────────────────────
#  Publish callback
# ─────────────────────────────────────────────────────────────────────────────

_history = {"min_v": None, "max_v": None, "min_t": None, "max_t": None}


def publish(bms: TeslaBMSSerial, svc: VeDbusService, cfg: BmsConfig, shunt: "ShuntMonitor | None" = None) -> bool:

    # ── 1. One-shot contactor control triggers ────────────────────────────────
    # /Control/Startup and /Control/Shutdown: write 1 to fire the command.
    # Auto-cleared to 0 immediately so they act as one-shot triggers.
    # Works from command line:
    #   dbus -y com.victronenergy.battery.teslabms /Control/Startup  SetValue %1
    #   dbus -y com.victronenergy.battery.teslabms /Control/Shutdown SetValue %1
    if svc["/Control/Startup"] == 1:
        log.info("D-Bus /Control/Startup → EXT_CMD_STARTUP")
        bms.send_command(EXT_CMD_STARTUP)
        svc["/Control/Startup"] = 0

    if svc["/Control/Shutdown"] == 1:
        log.info("D-Bus /Control/Shutdown → EXT_CMD_SHUTDOWN")
        bms.send_command(EXT_CMD_SHUTDOWN)
        svc["/Control/Shutdown"] = 0

    # ── 1b. Legacy /Switch state-based control ────────────────────────────────
    # Write 1 to request STARTUP, write 0 to request SHUTDOWN.
    # One-shot: the driver sends the command and immediately clears /Switch back
    # to None so the command is not re-sent on every subsequent publish cycle.
    # This prevents the GLib main loop from blocking on serial lock contention.
    switch = svc["/Switch"]
    if switch == 1:
        log.info("D-Bus /Switch=1 → EXT_CMD_STARTUP")
        bms.send_command(EXT_CMD_STARTUP)
        svc["/Switch"] = None
    elif switch == 0:
        log.info("D-Bus /Switch=0 → EXT_CMD_SHUTDOWN")
        bms.send_command(EXT_CMD_SHUTDOWN)
        svc["/Switch"] = None

    # ── 2. Connection state ───────────────────────────────────────────────────
    online = not bms.is_stale
    svc["/Connected"]                 = 1 if online else 0
    svc["/System/NrOfModulesOnline"]  = 1 if online else 0
    svc["/System/NrOfModulesOffline"] = 0 if online else 1
    svc["/Alarms/BmsCable"]           = ALARM_OK if online else ALARM_ALARM

    if not online:
        svc["/State"] = 10   # error / comm fault
        # ── Comms-loss safety: cap CVL at the per-cell safety voltage × cell_count ─
        # and zero all current limits so DVCC stops active charging/discharging.
        # CVL is set to a non-zero safety ceiling so that chargers which ignore
        # CCL=0 still cannot charge above a safe pack voltage.
        safety_cvl = round(cfg.comms_loss_cvl_per_cell * max(bms.cell_count, 1), 2)
        log.warning(
            f"BMS offline — setting CVL to safety-net {safety_cvl:.1f}V "
            "and zeroing CCL/DCL to prevent blind charging/discharging"
        )
        svc["/Info/MaxChargeVoltage"]    = safety_cvl
        svc["/Info/MaxChargeCurrent"]    = 0.0
        svc["/Info/MaxDischargeCurrent"] = 0.0
        svc["/Io/AllowToCharge"]                    = 0
        svc["/Io/AllowToDischarge"]                 = 0
        svc["/Io/AllowToBalance"]                   = 0
        svc["/System/NrOfModulesBlockingCharge"]    = 1
        svc["/System/NrOfModulesBlockingDischarge"] = 1
        # Keep contactor paths updated with last-known value so Node-RED
        # always reads a current value; /Connected=0 signals staleness.
        svc["/Contactor/State"]     = bms.contactor_state
        svc["/Contactor/StateName"] = contactor_state_name(bms.contactor_state)
        return True

    # ── 2b. Stale data guard ──────────────────────────────────────────────────
    # Data was received recently enough to keep 'online' status but is older
    # than STALE_TIMEOUT seconds.  Zero CCL/DCL immediately so the charger and
    # inverter ramp down gracefully while comms recover, rather than running on
    # stale limits for up to OFFLINE_TIMEOUT seconds.
    if not bms.data_fresh:
        age = int(time.time() - bms.last_frame_time)
        safety_cvl = round(cfg.comms_loss_cvl_per_cell * max(bms.cell_count, 1), 2)
        log.warning(
            f"BMS data stale ({age}s) — setting CVL to safety-net {safety_cvl:.1f}V "
            "and zeroing CCL/DCL to prevent blind charging/discharging"
        )
        svc["/Info/MaxChargeVoltage"]    = safety_cvl
        svc["/Info/MaxChargeCurrent"]    = 0.0
        svc["/Info/MaxDischargeCurrent"] = 0.0
        svc["/Io/AllowToCharge"]                    = 0
        svc["/Io/AllowToDischarge"]                 = 0
        svc["/Io/AllowToBalance"]                   = 0
        svc["/System/NrOfModulesBlockingCharge"]    = 1
        svc["/System/NrOfModulesBlockingDischarge"] = 1
        # Keep contactor paths updated with last-known value so Node-RED
        # always reads a current value; /Connected=1 but age visible in logs.
        svc["/Contactor/State"]     = bms.contactor_state
        svc["/Contactor/StateName"] = contactor_state_name(bms.contactor_state)
        return True

    # ── 3. Core telemetry ─────────────────────────────────────────────────────
    svc["/Soc"]              = bms.soc
    svc["/Dc/0/Voltage"]     = round(bms.voltage,     2)
   
    if HAS_CURRENT_SENSOR:
        svc["/Dc/0/Power"]   = round(bms.power, 0)
        svc["/Dc/0/Current"] = round(bms.current, 2)
    else:
        current = round(shunt.current_a, 2) if shunt else 0.0
        svc["/Dc/0/Current"] = current
        svc["/Dc/0/Power"]   = round(current * bms.voltage, 0) if shunt else 0.0
    svc["/Dc/0/Temperature"] = round(bms.temperature, 1)

    # Update actual contactor state on the read-only status paths.
    # /Switch is intentionally NOT overwritten here — doing so would stomp
    # any value written by Node-RED or dbus commands.
    svc["/Contactor/State"]     = bms.contactor_state
    svc["/Contactor/StateName"] = contactor_state_name(bms.contactor_state)

    # ── 4. Cell / temperature — real min/max (items 5-8) ─────────────────────
    svc["/System/NrOfCellsPerBattery"] = bms.cell_count

    # Use actual per-pack min/max cell voltages when available
    lo_cell = bms.lowest_cell_v  if bms.lowest_cell_v  > 0.0 else bms.avg_cell_volt
    hi_cell = bms.highest_cell_v if bms.highest_cell_v > 0.0 else bms.avg_cell_volt
    svc["/System/MinCellVoltage"]    = round(lo_cell, 3)
    svc["/System/MaxCellVoltage"]    = round(hi_cell, 3)
    svc["/System/MinVoltageCellId"]  = "min"
    svc["/System/MaxVoltageCellId"]  = "max"

    svc["/System/Temperature1"]         = round(bms.temperature, 1)
    svc["/System/MinCellTemperature"]   = round(bms.min_temp, 1)
    svc["/System/MaxCellTemperature"]   = round(bms.max_temp, 1)

    # ── 5. Capacity ───────────────────────────────────────────────────────────
    cap_remain   = round(bms.capacity_ah * bms.soc / 100.0, 2)
    cap_consumed = round(bms.capacity_ah - cap_remain, 2)
    svc["/InstalledCapacity"]  = bms.capacity_ah
    svc["/Capacity"]           = cap_remain
    svc["/ConsumedAmphours"]   = -cap_consumed

    # ── 6. CVL / CCL / DCL (items 2, 5, 6, 7, 8) ────────────────────────────
    cvl = calc_dynamic_cvl(bms, cfg)
    ccl = calc_ccl(bms, cfg)
    dcl = calc_dcl(bms, cfg)

    svc["/Info/MaxChargeVoltage"]    = cvl
    svc["/Info/BatteryLowVoltage"]   = bms.pack_undervoltage
    svc["/Info/MaxChargeCurrent"]    = ccl
    svc["/Info/MaxDischargeCurrent"] = dcl
    svc["/Info/TailCurrent"]         = cfg.tail_current
    svc["/Info/ChargeMode"]          = (
        "Normal"   if bms.overlord_state == OVERLORD_NORMAL   else
        "Storage"  if bms.overlord_state == OVERLORD_STORAGE  else
        "Shutdown" if bms.overlord_state == OVERLORD_SHUTDOWN else
        "Fault"
    )

    # ── 7. FET / IO flags ─────────────────────────────────────────────────────
    allow_charge    = ccl > 0.0
    allow_discharge = dcl > 0.0
    svc["/Io/AllowToCharge"]                     = 1 if allow_charge    else 0
    svc["/Io/AllowToDischarge"]                  = 1 if allow_discharge else 0
    svc["/Io/AllowToBalance"]                    = 1 if bms.balancing_active else 0   # item 9
    svc["/System/NrOfModulesBlockingCharge"]     = 0 if allow_charge    else 1
    svc["/System/NrOfModulesBlockingDischarge"]  = 0 if allow_discharge else 1

    # ── 8. State machine (item 11) ────────────────────────────────────────────
    if bms.overlord_state == OVERLORD_FAULT:
        svc["/State"] = 10   # error
    elif bms.overlord_state == OVERLORD_SHUTDOWN:
        svc["/State"] = 0    # idle/off — contactors open, clean shutdown
    elif bms.overlord_state == OVERLORD_STORAGE:
        svc["/State"] = 9    # idle / storage
    else:
        svc["/State"] = 9    # normal operation

    # ── 9. Alarms (item 10 — fault mask drives InternalFailure) ──────────────
    svc["/Alarms/LowVoltage"]            = bms.alarm_level(ALARM_UNDER_VOLTAGE)
    svc["/Alarms/HighVoltage"]           = bms.alarm_level(ALARM_OVER_VOLTAGE)
    svc["/Alarms/LowCellVoltage"]        = bms.alarm_level(ALARM_UNDER_VOLTAGE)
    svc["/Alarms/HighCellVoltage"]       = bms.alarm_level(ALARM_OVER_VOLTAGE)
    svc["/Alarms/HighTemperature"]       = bms.alarm_level(ALARM_OVER_TEMP)
    svc["/Alarms/LowTemperature"]        = bms.alarm_level(ALARM_UNDER_TEMP)
    svc["/Alarms/HighChargeCurrent"]     = bms.alarm_level(ALARM_OVER_CURRENT)
    svc["/Alarms/HighDischargeCurrent"]  = bms.alarm_level(ALARM_OVER_CURRENT)

    # InternalFailure only for a genuine fault — NOT for clean Shutdown (item 11)
    svc["/Alarms/InternalFailure"] = (
        ALARM_ALARM if bms.active_fault_mask != 0 and
                       bms.overlord_state == OVERLORD_FAULT
        else ALARM_OK
    )

    svc["/Alarms/LowSoc"] = (
        ALARM_ALARM   if bms.soc < 5  else
        ALARM_WARNING if bms.soc < 10 else
        ALARM_OK
    )

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

    # ── 12. History ───────────────────────────────────────────────────────────
    if _history["min_v"] is None or bms.voltage  < _history["min_v"]: _history["min_v"] = bms.voltage
    if _history["max_v"] is None or bms.voltage  > _history["max_v"]: _history["max_v"] = bms.voltage
    if _history["min_t"] is None or bms.min_temp < _history["min_t"]: _history["min_t"] = bms.min_temp
    if _history["max_t"] is None or bms.max_temp > _history["max_t"]: _history["max_t"] = bms.max_temp

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
    log.info("  TeslaBMS-ESP32 Venus OS driver v2.0 starting …")
    log.info("=" * 62)

    DBusGMainLoop(set_as_default=True)

    bus = (
        dbus.SessionBus()
        if "DBUS_SESSION_BUS_ADDRESS" in os.environ
        else dbus.SystemBus()
    )

    # ── Hard-coded charge/discharge configuration ─────────────────────────────
    cfg = BmsConfig()
    log.info(f"BmsConfig: CCL={cfg.max_charge_current}A DCL={cfg.max_discharge_current}A "
             f"Abs={cfg.absorption_voltage}V/cell Flt={cfg.float_voltage}V/cell")

    # ── SmartShunt monitor ────────────────────────────────────────────────────
    shunt = ShuntMonitor(bus)

    # ── Serial BMS handler ────────────────────────────────────────────────────
    bms = TeslaBMSSerial()
    bms.start(shunt_monitor=shunt)

    # ── D-Bus service ─────────────────────────────────────────────────────────
    svc = build_dbus_service(bus, cfg)
    log.info(f"D-Bus service '{DBUS_SERVICE_NAME}' registered.")

    GLib.timeout_add(PUBLISH_INTERVAL_MS, lambda: publish(bms, svc, cfg, shunt))

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
