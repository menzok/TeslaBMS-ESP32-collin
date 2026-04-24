# TeslaBMS-ESP32 → Venus OS Driver  v2.0

## Table of Contents
1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Installation](#installation)
5. [File Structure](#file-structure)
6. [How It Works](#how-it-works)
7. [Frame Protocol](#frame-protocol)
8. [DVCC Limit Calculations](#dvcc-limit-calculations)
9. [SmartShunt Current Feedback](#smartshunt-current-feedback)
10. [User-Configurable Settings](#user-configurable-settings)
11. [Alarm & Fault Reference](#alarm--fault-reference)
12. [Commands](#commands)
13. [D-Bus Path Reference](#d-bus-path-reference)
14. [Monitoring & Diagnostics](#monitoring--diagnostics)
15. [Known Limitations](#known-limitations)
16. [Troubleshooting](#troubleshooting)
17. [Upgrade Notes from v1](#upgrade-notes-from-v1)

---

## Overview

This driver connects a **TeslaBMS-ESP32** battery management system to a **Victron Venus OS** GX device
(Cerbo GX, Raspberry Pi GX, CCGX, etc.) over a USB serial connection.

Once running, Venus OS treats the Tesla pack as a fully **DVCC Controlling BMS**.  All charge voltage
limits (CVL), charge current limits (CCL), and discharge current limits (DCL) are **calculated on the
Pi** from the raw sensor data sent by the ESP32 — the ESP32 does nothing but report measurements and
thresholds.  The Victron DVCC system automatically distributes the limits to every connected Victron
device (MPPT chargers, Multis, Quattros) without any manual configuration.

```
┌──────────────────────────────────────────────────────────────────┐
│                    Venus OS (Cerbo GX / RPi)                      │
│                                                                    │
│  ┌──────────────┐  DVCC  ┌────────────────────────────────────┐  │
│  │ MPPT / Multi │◄──────►│  dbus-systemcalc-py (Victron)      │  │
│  │  (charger)   │        └───────────────┬────────────────────┘  │
│  └──────────────┘                        │ reads D-Bus            │
│                               ┌──────────▼────────────────────┐  │
│  ┌──────────────┐  D-Bus      │    dbus-teslabms.py  v2.0     │  │
│  │  SmartShunt  │────current─►│    calc_dynamic_cvl()         │  │
│  └──────────────┘             │    calc_ccl() / calc_dcl()    │  │
│                               └──────────┬────────────────────┘  │
│                                          │ USB serial (115200)    │
└──────────────────────────────────────────┼───────────────────────┘
                              7-byte req ──►│◄── 37-byte reply
                         ┌──────────────────▼──────────┐
                         │       ESP32 TeslaBMS         │
                         │  ExternalCommsLayer v2.0     │
                         └─────────────────────────────┘
```

---

## Hardware Requirements

| Item | Notes |
|---|---|
| Victron GX device | Cerbo GX, Raspberry Pi running Venus OS, CCGX, or similar |
| Venus OS version | **v3.00 or newer** (uses `gi.repository.GLib`) |
| USB → UART adapter | CP2102, CH340, or similar; connects ESP32 `EXTERNAL_COMM_SERIAL` to GX USB |
| ESP32 TeslaBMS | Running firmware with `ExternalCommsLayer` v2.0 at 115200 baud |
| SmartShunt *(optional)* | Any Victron battery monitor on D-Bus — used for SOC current feedback when no internal current sensor is fitted on the ESP32 |

The driver auto-detects the serial port by scanning `/dev/ttyUSB*` and `/dev/ttyACM*`. No manual
port configuration is required.

---

## Software Requirements

| Dependency | Where it comes from |
|---|---|
| `python3` | Pre-installed on Venus OS v3.x |
| `gi.repository.GLib` | Pre-installed on Venus OS v3.x |
| `dbus-python` | Pre-installed on Venus OS |
| `velib_python` | Ships with **dbus-serialbattery** (mr-manuel) |
| `pyserial` | Pre-installed on Venus OS |

**You must install `dbus-serialbattery` (mr-manuel) first**, even if you never use it for another BMS.
The driver uses its bundled `velib_python` library located at:

```
/opt/victronenergy/dbus-serialbattery/ext/velib_python/
```

Install dbus-serialbattery from: https://github.com/mr-manuel/venus-os_dbus-serialbattery

---

## Installation

### Step 1 — Copy files to the GX device

```bash
scp dbus-teslabms.py install.sh root@<GX-IP-ADDRESS>:/tmp/
```

Or copy via USB stick to `/tmp/`.

### Step 2 — Run the installer

```bash
ssh root@<GX-IP-ADDRESS>
cd /tmp
bash install.sh
```

The installer will:
1. Verify Venus OS ≥ v3.00
2. Copy `dbus-teslabms.py` to `/data/etc/dbus-teslabms/`
3. Create a runit service at `/service/dbus-teslabms/`
4. Add a hook to `/data/rc.local` so the service survives firmware updates
5. Start the driver immediately

### Step 3 — Verify it is running

```bash
# Check runit service status
svstat /service/dbus-teslabms

# Watch the live log
tail -f /var/log/dbus-teslabms/current

# Confirm the D-Bus service is registered
dbus -y com.victronenergy.battery.teslabms /Connected GetValue
# Expected: 1
```

### Uninstall

```bash
bash /data/etc/dbus-teslabms/uninstall.sh
```

---

## File Structure

After installation:

```
/data/etc/dbus-teslabms/
    dbus-teslabms.py        ← the Python driver
    install.sh              ← installer (kept for reference)
    uninstall.sh            ← generated by installer
    enable.sh               ← called by rc.local on every boot
    startup.log             ← boot-time enable.sh output

/service/dbus-teslabms/    ← runit service directory
    run                     ← service entry point
    log/run                 ← multilog logger

/var/log/dbus-teslabms/    ← rotating log (500 kB × 4 files)
    current                 ← active log file
```

---

## How It Works

### Driver startup sequence

```
1.  rc.local calls enable.sh on every boot
2.  enable.sh creates /service/dbus-teslabms/
3.  runit starts dbus-teslabms.py automatically
4.  DBusGMainLoop initialised
5.  VenusSettings.setup() — registers user-configurable settings in
        com.victronenergy.settings /Settings/TeslaBMS/*
6.  ShuntMonitor created — scans D-Bus for a SmartShunt service
7.  TeslaBMSSerial.start() — background reader thread begins:
        a. Scans /dev/ttyUSB* and /dev/ttyACM* for the ESP32
        b. Sends a 7-byte ping-pong frame; validates 37-byte reply
        c. Opens confirmed port at 115200 baud
        d. Every 2 s: refreshes SmartShunt current, sends 7-byte
           CMD_SEND_DATA frame, reads 37-byte telemetry reply
8.  VeDbusService registered as com.victronenergy.battery.teslabms
9.  GLib timer fires every 1000 ms → publish() writes D-Bus paths
10. dbus-systemcalc-py detects the battery service automatically
11. DVCC distributes CVL/CCL/DCL to all Victron chargers/inverters
```

### Steady-state data flow (ping-pong)

```
Every 2 s:

  1. ShuntMonitor.update()
     └─ reads SmartShunt /Dc/0/Current from D-Bus
        (staleness counter increments if unavailable)

  2. TeslaBMSSerial sends 7-byte CMD_SEND_DATA frame:
     [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]
     └─ embeds current SmartShunt reading + staleness counter

  3. ESP32 receives frame, stores shunt current, sends 37-byte reply

  4. _parse_frame() updates all bms.* attributes

Every 1 s (GLib timer):

  5. publish() calls:
     calc_dynamic_cvl()  → /Info/MaxChargeVoltage
     calc_ccl()          → /Info/MaxChargeCurrent
     calc_dcl()          → /Info/MaxDischargeCurrent
     and writes all other D-Bus paths

  6. dbus-systemcalc-py reads limits → instructs MPPT / Multiplus
```

### Offline detection

If no valid reply is received for **OFFLINE_TIMEOUT** (15 seconds):

- `/Connected` → 0
- `/Alarms/BmsCable` → 1 (Warning)
- `/State` → 0 (Initialising)
- DVCC falls back to its own defaults

### Auto-reconnect

If the serial port is lost (USB disconnected, ESP32 reboot) the reader thread closes the port and
re-runs the auto-detection scan every 5 seconds until the ESP32 is found again. No driver restart is
required.

### EEPROM config — no persistence needed

Every 37-byte reply frame carries the current ESP32 EEPROM thresholds alongside live telemetry.
Venus OS always reflects exactly what is in EEPROM. If a user changes a threshold in the BMS serial
console, it appears in Venus OS within one poll cycle (~2 seconds). There is no caching or stale
settings file.

---

## Frame Protocol

### Telemetry frame (ESP32 → Venus OS, in reply to each poll)

**37 bytes total:** `[0xAA]` + 34-byte payload + 2-byte CRC-16/MODBUS (little-endian, over payload)

| Bytes | Field | Type | Encoding | Unit |
|---|---|---|---|---|
| 0 | Start byte | — | `0xAA` | — |
| 1–2 | Pack voltage | uint16 BE | ÷ 100 | V (10 mV res) |
| 3–4 | Pack current | int16 BE | ÷ 10 | A (100 mA res, + = charge) |
| 5 | SOC | uint8 | raw | % |
| 6 | Average temperature | int8 | raw | °C (signed) |
| 7–8 | Power | int16 BE | raw | W |
| 9–10 | Average cell voltage | uint16 BE | ÷ 100 | V (10 mV res) |
| 11–12 | Alarm flags | uint16 BE | bitmask | see below |
| 13 | Overlord state | uint8 | enum | 0/1/2/3 — see below |
| 14 | Contactor state | uint8 | enum | raw |
| 15–16 | EEPROM over-voltage | uint16 BE | ÷ 1000 | V (1 mV res) |
| 17–18 | EEPROM under-voltage | uint16 BE | ÷ 1000 | V (1 mV res) |
| 19 | EEPROM over-temp | int8 | raw | °C (signed) |
| 20 | EEPROM under-temp | int8 | raw | °C (signed) |
| 21 | Number of modules | uint8 | raw | count |
| 22 | Parallel strings | uint8 | raw | count |
| 23–24 | Over-current threshold | uint16 BE | ÷ 10 | A (0.1 A res) |
| 25 | Status flags | uint8 | bitmask | bit0=currentSensorPresent, bit1=balancingActive |
| 26 | Active fault mask | uint8 | bitmask | 1<<FaultEntry::Type per active fault |
| 27–28 | Lowest cell voltage | uint16 BE | ÷ 1000 | V (1 mV res) |
| 29–30 | Highest cell voltage | uint16 BE | ÷ 1000 | V (1 mV res) |
| 31 | Minimum module temp | int8 | raw | °C (signed) |
| 32 | Maximum module temp | int8 | raw | °C (signed) |
| 33–34 | Reserved | uint8 × 2 | `0x00` | — |
| 35–36 | CRC-16/MODBUS | uint16 LE | over bytes 1–34 | — |

### CMD_SEND_DATA request frame (Venus OS → ESP32, every poll)

**7 bytes total** — this is the ping-pong frame that simultaneously requests telemetry **and** delivers
the Venus SmartShunt current reading to the ESP32 for SOC integration.

| Bytes | Field | Notes |
|---|---|---|
| 0 | `0xAA` | Start byte |
| 1 | `0x03` | `EXT_CMD_SEND_DATA` |
| 2–3 | Shunt current | int16 BE, ÷ 10 → A (+ = charge). 0 when unavailable. |
| 4 | Staleness counter | 0 = shunt data fresh this poll; increments each poll when unavailable |
| 5–6 | CRC-16/MODBUS | LE, computed over bytes 1–4 |

### Control command frames (Venus OS → ESP32)

**4 bytes total** — used for SHUTDOWN and STARTUP only. No telemetry reply is generated.

| Bytes | Field | Notes |
|---|---|---|
| 0 | `0xAA` | Start byte |
| 1 | Command code | `0x01` = SHUTDOWN, `0x02` = STARTUP |
| 2–3 | CRC-16/MODBUS | LE, computed over byte 1 only |

### CRC algorithm

CRC-16/MODBUS: initial value `0xFFFF`, polynomial `0xA001` (reflected).  
Computed over the **payload bytes only** (start byte excluded, for both request and reply).

---

## DVCC Limit Calculations

All CVL / CCL / DCL calculations are performed **entirely on the Pi**. The ESP32 sends only raw
measurements and EEPROM thresholds; it never pre-calculates any limits.

### Dynamic CVL (`calc_dynamic_cvl`)

Protects the highest individual cell from exceeding the EEPROM over-voltage setpoint even when
cells are imbalanced.

```
cell_spread  = highest_cell_v - avg_cell_v
safe_cell_v  = eep_overvoltage - cell_spread
dynamic_cvl  = clamp(safe_cell_v × cell_count,
                     float_pack_voltage,
                     absorption_pack_voltage)
```

- When cells are balanced (`cell_spread ≈ 0`), CVL equals the full absorption target.
- As the highest cell deviates, CVL is reduced proportionally.
- CVL never drops below the float voltage, preventing unnecessary charge inhibition.
- `AbsorptionVoltage` and `FloatVoltage` are set per-cell via Venus OS settings (see below).

### Temperature-derated CCL (`calc_ccl`)

Linear taper in a 5°C window around each charge temperature limit:

```
T < MinChargeTemp                        → CCL = 0
MinChargeTemp ≤ T < MinChargeTemp + 5°C → CCL = max_ccl × (T - MinChargeTemp) / 5
MinChargeTemp + 5°C ≤ T ≤ MaxChargeTemp - 5°C → CCL = max_ccl
MaxChargeTemp - 5°C < T ≤ MaxChargeTemp → CCL = max_ccl × (MaxChargeTemp - T) / 5
T > MaxChargeTemp                        → CCL = 0
```

`max_ccl` = min(`MaxChargeCurrent` setting, EEPROM over-current threshold).

### Temperature-derated DCL (`calc_dcl`)

Linear taper in a 5°C window above the EEPROM under-temp setpoint, using the **minimum** module
temperature for conservative protection:

```
T < eep_undertemp                        → DCL = 0
eep_undertemp ≤ T < eep_undertemp + 5°C → DCL = max_dcl × (T - eep_undertemp) / 5
T ≥ eep_undertemp + 5°C                 → DCL = max_dcl
```

Both CCL and DCL are additionally forced to **0** while the Overlord is in Fault or Shutdown state.

---

## SmartShunt Current Feedback

When the ESP32 does **not** have an internal current sensor (`currentSensorPresent` flag = 0), the
driver reads the SmartShunt current from D-Bus and embeds it in every `CMD_SEND_DATA` poll frame.

### How the driver finds the SmartShunt

`ShuntMonitor` scans D-Bus for any `com.victronenergy.battery.*` service that is **not** the
TeslaBMS service itself, then reads `/Dc/0/Current` from it. The first matching service is used and
cached.  If it disappears the scan is retried on the next poll.

### Staleness tracking

The `staleness` byte in the poll frame increments by 1 each poll when no fresh shunt data is
available (SmartShunt offline, D-Bus read failure, etc.), up to a maximum of 255.

The ESP32 ignores shunt data when:
- `staleness > 0` (Venus couldn't read it this poll), **or**
- the last frame carrying `staleness == 0` arrived more than **6 seconds** ago (≈ 3 missed polls)

When stale, the ESP32 SOC calculator falls back to pure OCV blending until fresh shunt data resumes.

### Current sensor presence flag

When `currentSensorPresent` (statusFlags bit 0) is **false**, the driver publishes
`/Dc/0/Current = None` on D-Bus.  This prevents Venus OS from seeing a false 0 A reading that would
overwrite the SmartShunt's authoritative current measurement in the DVCC system current sensor chain.

---

## User-Configurable Settings

Settings are stored persistently in `com.victronenergy.settings /Settings/TeslaBMS/*` and survive
driver restarts and firmware updates. Defaults are applied on first run.

| Setting | Default | Range | Description |
|---|---|---|---|
| `MaxChargeCurrent` | 250 A | 1–500 A | Maximum CCL (before temperature derating) |
| `MaxDischargeCurrent` | 250 A | 1–500 A | Maximum DCL (before temperature derating) |
| `AbsorptionVoltage` | 4.15 V/cell | 3.50–4.25 V | Per-cell absorption target for CVL |
| `FloatVoltage` | 4.10 V/cell | 3.20–4.20 V | Per-cell float floor for CVL |
| `TailCurrent` | 10 A | 0.5–50 A | Published to `/Info/TailCurrent` for Victron charger absorption end-of-charge |
| `MaxChargeTemp` | 45 °C | 20–60 °C | Charge inhibit above this temperature |
| `MinChargeTemp` | 5 °C | −10–20 °C | Charge inhibit below this temperature |

Settings can be changed via:
- **Venus OS GUI**: Settings → TeslaBMS
- **SSH command line**: `dbus -y com.victronenergy.settings /Settings/TeslaBMS/MaxChargeCurrent SetValue %200`
- **Node-RED**: write to the settings service D-Bus path

---

## Alarm & Fault Reference

### Alarm flags bitmask (payload bytes 11–12)

These flags are set by the ESP32 when live cell or current readings cross EEPROM thresholds.

| Bit | Constant | Meaning | Victron path(s) | Level |
|---|---|---|---|---|
| 0 | `ALARM_OVER_VOLTAGE` | Cell over-voltage | `/Alarms/HighVoltage`, `/Alarms/HighCellVoltage` | Alarm (2) |
| 1 | `ALARM_UNDER_VOLTAGE` | Cell under-voltage | `/Alarms/LowVoltage`, `/Alarms/LowCellVoltage` | Alarm (2) |
| 2 | `ALARM_OVER_TEMP` | Over-temperature | `/Alarms/HighTemperature` | Alarm (2) |
| 3 | `ALARM_UNDER_TEMP` | Under-temperature | `/Alarms/LowTemperature` | Alarm (2) |
| 4 | `ALARM_OVER_CURRENT` | Over-current | `/Alarms/HighChargeCurrent`, `/Alarms/HighDischargeCurrent` | Alarm (2) |
| 5–15 | — | Reserved | — | — |

### Active fault mask (payload byte 26)

The active fault mask reflects which fault types are currently active in the ESP32 EEPROM fault log
(entries with no cleared timestamp).  Each bit is `1 << FaultEntry::Type`:

| Bit | Fault type | Meaning |
|---|---|---|
| 1 | OverVoltage | At least one OV fault is currently active |
| 2 | UnderVoltage | At least one UV fault is currently active |
| 3 | OverTemperature | At least one OT fault is currently active |
| 4 | UnderTemperature | At least one UT fault is currently active |
| 5 | OverCurrent | At least one OC fault is currently active |
| 6 | CommsError | At least one comms error fault is currently active |
| 0, 7 | — | Reserved / unused |

`/Alarms/InternalFailure` is set to **Alarm (2)** only when `active_fault_mask != 0` **and**
`overlord_state == OVERLORD_FAULT (1)`.  A clean shutdown (state 3) does **not** trigger
`InternalFailure`.

### Status flags (payload byte 25)

| Bit | Name | Meaning |
|---|---|---|
| 0 | `currentSensorPresent` | ESP32 has an internal Hall-effect current sensor fitted |
| 1 | `balancingActive` | At least one cell is currently being passively balanced |

### Overlord state (payload byte 13)

| Value | Name | Driver behaviour |
|---|---|---|
| 0 | Normal | Charge and discharge allowed. `/State` = 9 (Running) |
| 1 | Fault | Contactors open due to active fault. CCL = DCL = 0. `/State` = 10 (Error). `/Alarms/InternalFailure` = Alarm |
| 2 | StorageMode | Periodic wake/balance cycle active. `/Info/ChargeMode` = "Storage" |
| 3 | Shutdown | Clean contactor open requested externally. `/State` = 0. **No** `InternalFailure` alarm. |

### SOC-based low SOC alarm

| SOC | `/Alarms/LowSoc` |
|---|---|
| ≥ 10 % | 0 — OK |
| 5–9 % | 1 — Warning |
| < 5 % | 2 — Alarm |

### BMS cable alarm

| Condition | `/Alarms/BmsCable` |
|---|---|
| Frame received within 15 s | 0 — OK |
| No frame for > 15 s | 1 — Warning |

---

## Commands

Commands are sent by writing `1` to a writable D-Bus path. The driver transmits the appropriate
4-byte control frame and resets the path to `0`.  Control commands do **not** generate a telemetry
reply — the next scheduled poll (≤ 2 s) will return fresh data.

### Available commands

| D-Bus path | Command sent | ESP32 action |
|---|---|---|
| `/Control/Shutdown` | `EXT_CMD_SHUTDOWN (0x01)` | `Overlord.requestShutdown()` → opens contactors |
| `/Control/Startup` | `EXT_CMD_STARTUP (0x02)` | `Overlord.requestStartup()` → closes contactors |

### Method 1 — SSH command line

```bash
# Shutdown (open contactors)
dbus -y com.victronenergy.battery.teslabms /Control/Shutdown SetValue %1

# Startup (close contactors)
dbus -y com.victronenergy.battery.teslabms /Control/Startup SetValue %1

# Read current SOC
dbus -y com.victronenergy.battery.teslabms /Soc GetValue

# Read dynamic CVL being sent to chargers
dbus -y com.victronenergy.battery.teslabms /Info/MaxChargeVoltage GetValue
```

### Method 2 — Node-RED

```
[Button inject]  →  [Set msg.payload = 1]  →  [D-Bus out]
                                                 service: com.victronenergy.battery.teslabms
                                                 path:    /Control/Shutdown
```

### Method 3 — VRM / MQTT (remote)

```
Topic:  W/<VRM-portal-ID>/battery/<device-instance>/Control/Shutdown
Value:  {"value": 1}
```

---

## D-Bus Path Reference

Service name: `com.victronenergy.battery.teslabms`

### Identity

| Path | Type | Value |
|---|---|---|
| `/ProductName` | str | `TeslaBMS-ESP32` |
| `/FirmwareVersion` | str | `2.0` |
| `/HardwareVersion` | str | `ESP32 HEX v1.0` |
| `/Connected` | int | 0 = offline, 1 = online |
| `/CustomName` | str | User-editable label |
| `/State` | int | 0 = idle/off, 9 = running, 10 = error |

### Live telemetry

| Path | Unit | Notes |
|---|---|---|
| `/Soc` | % | 0–100 |
| `/Dc/0/Voltage` | V | Pack voltage |
| `/Dc/0/Current` | A | Positive = charging. **`None`** when `currentSensorPresent` is false (prevents overwriting SmartShunt) |
| `/Dc/0/Power` | W | |
| `/Dc/0/Temperature` | °C | Average pack temperature |
| `/System/MinCellVoltage` | V | **Actual lowest cell voltage** from ESP32 |
| `/System/MaxCellVoltage` | V | **Actual highest cell voltage** from ESP32 |
| `/System/MinCellTemperature` | °C | Minimum module temperature |
| `/System/MaxCellTemperature` | °C | Maximum module temperature |
| `/System/NrOfCellsPerBattery` | int | Derived: modules ÷ strings × 6 |
| `/System/Temperature1` | °C | Average pack temperature (alias) |

### DVCC limits (read by dbus-systemcalc-py)

| Path | Unit | Notes |
|---|---|---|
| `/Info/MaxChargeVoltage` | V | **Dynamic CVL** — see calculation section |
| `/Info/BatteryLowVoltage` | V | EEPROM UV × cell count |
| `/Info/MaxChargeCurrent` | A | **Temperature-derated CCL** — see calculation section |
| `/Info/MaxDischargeCurrent` | A | **Temperature-derated DCL** — see calculation section |
| `/Info/TailCurrent` | A | Configurable via Venus OS settings |
| `/Info/ChargeMode` | str | Normal / Storage / Shutdown / Fault |

### Capacity

| Path | Unit | Notes |
|---|---|---|
| `/InstalledCapacity` | Ah | (modules ÷ strings) × 232 Ah |
| `/Capacity` | Ah | Remaining = installed × SOC% |
| `/ConsumedAmphours` | Ah | Negative convention (Victron standard) |

### IO / FET flags

| Path | Value | Notes |
|---|---|---|
| `/Io/AllowToCharge` | 0 or 1 | 0 when CCL = 0 (fault, out-of-temp-range) |
| `/Io/AllowToDischarge` | 0 or 1 | 0 when DCL = 0 (fault, too cold) |
| `/Io/AllowToBalance` | 0 or 1 | Reflects live `balancingActive` flag from ESP32 |

### Alarms

| Path | Levels | Trigger |
|---|---|---|
| `/Alarms/LowVoltage` | 0/2 | `ALARM_UNDER_VOLTAGE` bit |
| `/Alarms/HighVoltage` | 0/2 | `ALARM_OVER_VOLTAGE` bit |
| `/Alarms/LowCellVoltage` | 0/2 | `ALARM_UNDER_VOLTAGE` bit |
| `/Alarms/HighCellVoltage` | 0/2 | `ALARM_OVER_VOLTAGE` bit |
| `/Alarms/LowSoc` | 0/1/2 | SOC < 10% warning, < 5% alarm |
| `/Alarms/HighTemperature` | 0/2 | `ALARM_OVER_TEMP` bit |
| `/Alarms/LowTemperature` | 0/2 | `ALARM_UNDER_TEMP` bit |
| `/Alarms/HighChargeCurrent` | 0/2 | `ALARM_OVER_CURRENT` bit |
| `/Alarms/HighDischargeCurrent` | 0/2 | `ALARM_OVER_CURRENT` bit |
| `/Alarms/InternalFailure` | 0/2 | `active_fault_mask != 0` **and** `overlord_state == Fault` |
| `/Alarms/BmsCable` | 0/1 | No valid frame for > 15 s |

### Live EEPROM config mirror (read-only)

Updated every poll cycle. Always reflects current ESP32 EEPROM values.

| Path | Unit | Notes |
|---|---|---|
| `/Config/CellOverVoltage` | V | EEPROM OV threshold per cell |
| `/Config/CellUnderVoltage` | V | EEPROM UV threshold per cell |
| `/Config/OverTemp` | °C | EEPROM OT threshold |
| `/Config/UnderTemp` | °C | EEPROM UT threshold |
| `/Config/NumModules` | int | Total modules detected by BMS scan |
| `/Config/NumStrings` | int | Parallel string count from EEPROM |
| `/Config/CellCount` | int | Series cells per string |
| `/Config/CapacityAh` | Ah | (modules ÷ strings) × 232 Ah |
| `/Config/OverCurrentThreshold` | A | EEPROM over-current trip point |

### User settings mirror (read-only, reflects /Settings/TeslaBMS/*)

| Path | Unit | Notes |
|---|---|---|
| `/Settings/MaxChargeCurrent` | A | |
| `/Settings/MaxDischargeCurrent` | A | |
| `/Settings/AbsorptionVoltage` | V/cell | |
| `/Settings/FloatVoltage` | V/cell | |
| `/Settings/TailCurrent` | A | |
| `/Settings/MaxChargeTemp` | °C | |
| `/Settings/MinChargeTemp` | °C | |

### Commands (writable)

| Path | Write | Action |
|---|---|---|
| `/Control/Shutdown` | `1` | Sends `EXT_CMD_SHUTDOWN` → Overlord opens contactors |
| `/Control/Startup` | `1` | Sends `EXT_CMD_STARTUP` → Overlord closes contactors |

---

## Monitoring & Diagnostics

### View live log

```bash
tail -f /var/log/dbus-teslabms/current
```

### Inspect all D-Bus paths live

```bash
dbus-spy
# Navigate to com.victronenergy.battery.teslabms
```

### Check service status

```bash
svstat /service/dbus-teslabms
# Example: /service/dbus-teslabms: up (pid 1234) 143 seconds
```

### Stop / start / restart the driver

```bash
svc -d /service/dbus-teslabms          # stop
svc -u /service/dbus-teslabms          # start
svc -d /service/dbus-teslabms && sleep 2 && svc -u /service/dbus-teslabms  # restart
```

---

## Known Limitations

| Limitation | Detail |
|---|---|
| No per-cell granularity | Min/max cell V is reported per-pack (lowest and highest across all modules), not per-cell index. Per-cell D-Bus paths (`/Cell/0/Voltage` etc.) are not published. |
| Single pack only | The driver supports one ESP32 on one serial port. |
| No SOH | State of Health is not calculated. `/Soh` is always `None`. |
| No energy history | Charged/discharged Wh counters are not implemented. |
| No GUI settings page | The Venus OS remote console does not show a custom TeslaBMS page; settings are adjusted via the standard Settings → TeslaBMS path or SSH. |

---

## Troubleshooting

### Driver not starting

```bash
# Check runit sees the service
ls /service/dbus-teslabms

# If missing, re-run enable script
bash /data/etc/dbus-teslabms/enable.sh

# Check the log for Python errors
tail -50 /var/log/dbus-teslabms/current
```

### ESP32 not detected

```bash
# Check USB adapter is visible
ls /dev/ttyUSB* /dev/ttyACM*

# Manually probe (replace ttyUSB0 as needed)
python3 - <<'EOF'
import serial, struct, time

def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else c >> 1
    return c

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
# 7-byte CMD_SEND_DATA with zeroed shunt fields
cmd = bytes([0x03, 0x00, 0x00, 0xFF])   # cmd + curr=0 + staleness=255
frame = bytes([0xAA]) + cmd + struct.pack('<H', crc16(cmd))
s.reset_input_buffer()
s.write(frame)
data = s.read(37)
print(f'Received {len(data)} bytes:', data.hex())
s.close()
EOF
# Expected: 37 bytes starting with AA
```

### Venus OS not showing the battery

```bash
# Confirm service is registered
dbus -y com.victronenergy.battery.teslabms /Connected GetValue
# Expected: 1

# If command fails, driver is not running
tail -f /var/log/dbus-teslabms/current
```

### DVCC not controlling the charger

1. In the Venus OS GUI go to **Settings → System Setup → DVCC** and confirm DVCC is **enabled**.
2. Confirm `/Connected` = 1 and `/Info/MaxChargeVoltage` has a non-zero value.
3. Check that the MPPT/Multi firmware supports DVCC (most do from 2018 onward).
4. Confirm the battery service is recognised as a Controlling BMS — in **Settings → DVCC**, the
   "BMS control" field should show `TeslaBMS-ESP32`.

### Values look wrong (voltage off by 10×, current always 0)

This indicates a frame format mismatch.  Ensure both the ESP32 firmware and Python driver are the
**same major version** (both v2.0).  The v2.0 frame is 37 bytes (34-byte payload); v1 used 27 bytes
and is **not** backward-compatible.

### Driver crashes in a loop

```bash
cat /var/log/dbus-teslabms/current | tail -100
# Common causes:
# - velib_python not found  → install dbus-serialbattery first
# - Wrong Venus OS version  → need v3.x for gi.repository.GLib
# - Permission error on /dev/ttyUSBx  → should not happen running as root
```

---

## Upgrade Notes from v1

| Area | v1 | v2.0 |
|---|---|---|
| Frame size | 27 bytes (24-byte payload) | **37 bytes (34-byte payload)** — not backward-compatible |
| Poll model | ESP32 sent unsolicited frames every ~1 s | **Ping-pong**: Venus sends 7-byte request, ESP32 replies |
| CCL / DCL | Static hardcoded values (`MAX_CHARGE_CURRENT`) | **Temperature-derated**, configurable via Venus OS settings |
| CVL | Static: `(OV − 0.10 V) × cell_count` | **Dynamic**: adjusts with cell spread to protect highest cell |
| Current on D-Bus | Always published (0 A when no sensor) | **Suppressed when no sensor** — prevents overwriting SmartShunt |
| Overlord states | 0 = Normal, 1 = Fault/Shutdown, 2 = Storage | **0 = Normal, 1 = Fault, 2 = Storage, 3 = Shutdown** |
| `InternalFailure` on shutdown | Yes (false alarm) | **No** — clean shutdown never triggers `InternalFailure` |
| Per-cell min/max voltage | Not available (avg only) | **Lowest and highest cell V** reported separately |
| Balancing indicator | Not available | `/Io/AllowToBalance` reflects live balancing state |
| SmartShunt SOC feedback | Not available | Venus reads SmartShunt and injects current into ESP32 SOC |
