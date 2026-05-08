# ANENJI ANJ-3KW-24V-LV-WIFI → Venus OS Driver

## 1. Overview

This staging-area driver publishes the **ANENJI ANJ-3KW-24V-LV-WIFI** as a Venus OS
**vebus** device (`com.victronenergy.vebus.anenji`) so GUI v2 treats it like an
inverter/charger, and also publishes the inverter's internal MPPT data on an
always-running **solarcharger** companion service (`com.victronenergy.solarcharger.anenji`).

What it does:
- Polls the inverter over **Modbus RTU / RS-485** at **9600 8N1**.
- Auto-discovers the USB serial port and Modbus address once per process start.
- Locks the discovered port + address in RAM until the driver restarts.
- Exposes live AC, DC, battery, alarm, PV, and selected writable settings on D-Bus.
- Performs best-effort DVCC write-back for charge current limits.
- Offers simple Node-RED one-shot control paths.

What it cannot fully solve:
- `/Mode` only maps to the inverter's **remote on/off** register, so Venus OS modes
  1/2/3 all become **remote switch on** and mode 4 becomes **off**.
- DVCC can request charge current changes, but the inverter may ignore them unless
  **Battery Type register 322 = 2 (USER mode)**.
- External Victron MPPT power cannot be injected into the ANENJI internal MPPT.
  The driver only monitors external solar power and can best-effort bias charge
  priority toward PV-first operation.

## 2. Hardware requirements

- ANENJI **ANJ-3KW-24V-LV-WIFI** inverter/charger
- Raspberry Pi or Venus OS GX device running **Venus OS 3.71+**
- Industrial USB↔RS-485 adapter using **FT232RL + SP485EEN**
- RS-485 wiring from adapter to inverter communication port
- The adapter usually appears as `/dev/ttyUSB*` or `/dev/ttyACM*`

## 3. Installation

1. Copy this folder to the Venus OS machine.
2. Run as root:
   ```sh
   cd /path/to/Anenji\ Inverter
   bash Install.sh
   ```
3. Watch logs:
   ```sh
   tail -f /var/log/dbus-anenji/current
   ```
4. Reboot, or restart the service with `svc -t /service/dbus-anenji` if needed.

Installer actions:
- verifies Venus OS v3+
- checks for `velib_python`
- copies files to `/data/etc/dbus-anenji/`
- writes `enable.sh` and `uninstall.sh`
- hooks `enable.sh` into `/data/rc.local`
- creates `/service/dbus-anenji/`
- starts the runit service

## 4. Configuration

Persistent localsettings are stored under:
- `/Settings/Anenji/ModbusAddress` (default `1`)
- `/Settings/Anenji/DeviceInstance` (default `276`)
- `/Settings/Anenji/SolarChargerInstance` (default `10`)

Discovery order at startup:
1. probe configured address first
2. then probe addresses `1..10`
3. then full sweep `1..247`
4. remember the discovered **port + address in RAM**
5. on disconnect, retry the **same port only**
6. full port re-scan happens only after process restart / reboot

## 5. Venus OS integration

### Primary service
`com.victronenergy.vebus.anenji`
- shows as inverter/charger on GUI v2 energy flow
- publishes AC input, AC output, battery voltage/current/power, SOC, state, alarms
- exposes writable setting paths under `/Settings/*`
- exposes Node-RED trigger paths under `/Control/*`

### Companion solar service
`com.victronenergy.solarcharger.anenji`
- always runs with the driver
- publishes PV voltage, PV current, PV power, and PV charge output
- allows the inverter's internal MPPT to appear separately in Venus OS
- can coexist with additional Victron MPPT services on the same GX device

## 6. DVCC integration

### What works
- `/BatteryOperationalLimits/MaxChargeCurrent` is writable and queued to
  **register 332**.
- `/Dc/0/MaxChargeCurrent` mirrors the same best-effort write-back.
- `/BatteryOperationalLimits/MaxChargeVoltage` maps to **register 324**.
- `/BatteryOperationalLimits/MaxDischargeCurrent` maps to **register 351**.

### Important limitation
**Register 332 may only take effect when Battery Type (`322`) is set to `2 = USER`.**
That is a device limitation, not a Venus OS limitation.

### External solar / future enhancement
The driver watches other `com.victronenergy.solarcharger.*` services on D-Bus and
logs reported solar power. If DVCC has already written a charge-current limit and
external solar power is present, the driver can best-effort write **register 331**
to **PV first**. This does **not** make the ANENJI internal MPPT consume external
Victron solar power; it only nudges the inverter's own priority logic.

## 7. Solar MPPT coexistence

The companion solar service uses the ANENJI's own PV registers:
- PV voltage = register `219`
- PV current = register `220`
- PV power = register `223`
- PV charge power = register `224`
- PV charge current = register `234`

This service is separate from any Victron MPPT service. Both can appear on the
GX device at the same time because the service names and DeviceInstance numbers
are independent.

## 8. Node-RED control paths

Write `1` to these one-shot paths:
- `/Control/RemoteOn` → register `420 = 1`
- `/Control/RemoteOff` → register `420 = 0`
- `/Control/ClearFaults` → register `426 = 1` (best-effort; undocumented in the public map)

Additional writable paths:
- `/Mode`
- `/Ac/ActiveIn/CurrentLimit`
- `/BatteryOperationalLimits/*`
- `/Settings/*`

## 9. Coexistence with TeslaBMS driver

Running this driver next to `com.victronenergy.battery.teslabms` is supported.
They do not conflict because they use:
- different D-Bus service names
- different DeviceInstance numbers
- different serial ports
- different protocol stacks

DVCC can still use the TeslaBMS battery service for battery limits while using the
ANENJI vebus service for inverter/charger control.

## 10. Known ANENJI-specific limitations

- No documented register exists for true Victron-style charger-only vs inverter-only mode.
- Some writable settings may be rejected depending on inverter operating mode.
- `register 426` is used for clear-faults as a best-effort path because the supplied
  public register summary does not document it.
- Voltage-related tuning is only as accurate as the inverter's own register scaling.
  The driver reads **register 644** (rated battery cell count) and keeps it available
  for battery-voltage-related logic, but the inverter still reports threshold values
  as device-native `0.1V` registers.

## 11. Troubleshooting

- **No device found**: confirm the adapter enumerates as `/dev/ttyUSB*` or `/dev/ttyACM*`.
- **Wrong Modbus address**: adjust `/Settings/Anenji/ModbusAddress`, then restart the driver.
- **Driver only retries one port**: this is intentional; the driver locks to the first
  discovered port until restart to avoid spamming unrelated serial devices.
- **DVCC writes appear ignored**: set Battery Type `322 = USER`, then retry.
- **No PV values**: verify panel wiring and check registers `219/220/223/224/234`.
- **Frequent reconnects**: check RS-485 polarity, USB power stability, and serial logs.

## 12. Register reference table

### Real-time data

| Register | Meaning | Scale |
|---|---|---|
| 201 | Working mode | raw |
| 202 | Grid voltage RMS | 0.1 V |
| 203 | Grid frequency | 0.01 Hz |
| 204 | Grid power | 1 W |
| 210 | Output voltage RMS | 0.1 V |
| 211 | Output current RMS | 0.1 A |
| 213 | Output active power | 1 W |
| 214 | Output apparent power | 1 VA |
| 215 | Battery voltage | 0.1 V |
| 229 | Battery SOC | 1 % |
| 231 | Power flow status | bitfield |
| 232 | Battery current filtered | 0.1 A |
| 219 | PV voltage | 0.1 V |
| 220 | PV current | 0.1 A |
| 223 | PV power | 1 W |
| 224 | PV charge power | 1 W |
| 234 | PV charge current | 0.1 A |

### Device info

| Register | Meaning |
|---|---|
| 171 | Device type |
| 172..183 | Device name (ASCII) |
| 184 | Protocol number |
| 186..197 | Serial number (ASCII) |
| 626..633 | Firmware version (ASCII) |
| 643 | Rated power |
| 644 | Rated battery cell count |

### Writable settings used by this driver

| Register | Path |
|---|---|
| 301 | `/Settings/OutputPriority` |
| 303 | `/Settings/BuzzerMode` |
| 307 | `/Settings/EnergySaving` |
| 308 | `/Settings/OverloadRestart` |
| 309 | `/Settings/OverTempRestart` |
| 320 | `/Settings/OutputVoltage` |
| 321 | `/Settings/OutputFrequency` |
| 322 | `/Settings/BatteryType` |
| 324 | `/BatteryOperationalLimits/MaxChargeVoltage` |
| 331 | `/Settings/ChargePriority` |
| 332 | `/Settings/MaxChargeCurrent`, `/BatteryOperationalLimits/MaxChargeCurrent`, `/Dc/0/MaxChargeCurrent` |
| 333 | `/Settings/MaxGridChargeCurrent`, `/Ac/ActiveIn/CurrentLimit` |
| 351 | `/BatteryOperationalLimits/MaxDischargeCurrent` |
| 420 | `/Mode`, `/Settings/RemoteSwitch`, `/Control/RemoteOn`, `/Control/RemoteOff` |
| 426 | `/Control/ClearFaults` |
