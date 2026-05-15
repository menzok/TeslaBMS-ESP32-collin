# TeslaBMS-ESP32

**Full Featured BMS, based on collins80's work on these modules** (Because i looked at the SIMP BMS Code and cried and Vero Based on that implementation, is $1k for cheap hardware)

This firmware turns an ESP32 into a full-featured Battery Management System (BMS) master controller for **Tesla Model S/X battery modules** (6-cell 18650/2170 modules with the BQ76PL455A-Q1 chip).

It communicates with the modules over the original high-speed daisy-chain TTL UART protocol (612500 baud) and provides complete pack monitoring, balancing, contactor control, advanced SOC estimation, comprehensive safety features, and full **Victron Venus OS DVCC integration** via a Raspberry Pi Venus OS driver.

---

### Based on

This project is a complete ESP32 port of the excellent original work by **collin80**:

→ [collin80/TeslaBMS](https://github.com/collin80/TeslaBMS)

The core communication protocol, module addressing, CRC handling, and register map are directly derived from the original implementation.

**Original wiring diagram** (highly recommended):
[https://cdn.hackaday.io/files/10098432032832/wiring.pdf](https://cdn.hackaday.io/files/10098432032832/wiring.pdf)

---

### Current State (24 April 2026)

The firmware is **stable and fully functional** for solar/off-grid, DIY EV packs, and similar applications. It is intentionally kept as a **flat codebase** so it can be opened and compiled directly in the Arduino IDE (GET VS CODE YALL) with no complex folder structure or external build tools required.

**Victron Venus OS DVCC integration is now fully implemented** — the Venus OS driver (`VenusOS driver/dbus-teslabms.py`) registers the BMS as a proper Controlling BMS and handles all charge limit calculations (CVL, CCL, DCL) on the Pi.

Further detailed documentation is available in the repository:
- [`System.md`](System.md) — Complete system architecture and component documentation
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — High-level design decisions and data flow (VIEW RAW, or looks screwed up)

---

### Features

- Reliable daisy-chain communication with up to 62 modules
- Accurate per-cell voltage and dual temperature monitoring
- Automatic passive cell balancing with configurable threshold + hysteresis
- Non-blocking contactor + pre-charge control (current-based or timed)
- Hybrid SOC estimation (coulomb counting + temperature-compensated OCV lookup)
- Full per-cell OV/UV/OT/UT protection with configurable debounce
- Storage/maintenance mode (periodic wake + balance cycles)
- Persistent configuration and rolling fault log in EEPROM
- Interactive serial console menu
- **Full Victron Venus OS DVCC integration** — dynamic CVL, temperature-derated CCL/DCL, SmartShunt current feedback, proper Controlling BMS recognition

---

### Victron Venus OS Integration

The `VenusOS driver/dbus-teslabms.py` script runs on a Venus OS device (Cerbo GX, Raspberry Pi, etc.) and implements the full DVCC managed-battery feature set:

| Feature | Detail |
|---|---|
| **Dynamic CVL** | Reduces charge voltage limit as the highest cell diverges from average — protects unbalanced cells without triggering OV alarms |
| **CCL derating** | Linearly tapers charge current in the 5°C window below `MaxChargeTemp` and above `MinChargeTemp` |
| **DCL derating** | Linearly tapers discharge current in the 5°C window above the EEPROM under-temp setpoint |
| **SmartShunt feedback** | Reads SmartShunt `/Dc/0/Current` from D-Bus, embeds it in every poll frame so the ESP32 can use it for SOC coulomb counting when no internal sensor is fitted |
| **Current sensor awareness** | When `currentSensorPresent` flag is false, `/Dc/0/Current` is withheld from D-Bus so the SmartShunt reading is not overwritten |
| **Balancing indicator** | `/Io/AllowToBalance` reflects the live per-cell balancing state |
| **Proper fault mapping** | Clean shutdown (`overlordState=3`) no longer triggers a false `InternalFailure` alarm |
| **User settings** | CVL absorption/float voltage, CCL/DCL caps, tail current, and charge temperature limits are all configurable via Venus OS settings |

#### Installation

1. Install `velib_python` on your Venus OS device (comes with Venus OS ≥ v2.80).
2. Copy `VenusOS driver/dbus-teslabms.py` to `/data/apps/dbus-teslabms/`.
3. Make it executable: `chmod +x dbus-teslabms.py`
4. Add a `start.sh` or configure `daemontools` / `rc.local` to launch it at boot.
5. Tune the charge parameters in the Venus OS GUI under **Settings → TeslaBMS**.

The driver auto-detects the ESP32 on `/dev/ttyUSB*` or `/dev/ttyACM*` and manages the `serial-starter` service automatically.

---

### Quick Start (ESP32 firmware)

1. Wire the modules exactly as shown in the [original wiring diagram](https://cdn.hackaday.io/files/10098432032832/wiring.pdf)
2. Connect:
   - `Serial2` (GPIO 16 RX / 17 TX) → BMS daisy-chain
   - `Serial1` (GPIO 9 RX / 10 TX) → Venus OS serial connection (`EXTERNAL_COMM_SERIAL` in `config.h`)
   - GPIO 25 → Optional Pre-charge relay
   - GPIO 26 → Main contactor relay
   - GPIO 34 → Optional current sensor (Hall effect)
3. Open the project in the Arduino IDE (flat folder structure)
4. Select your ESP32 board and flash
5. Open the serial monitor at 115200 baud and type `menu` for the interactive console

*****Ensure you use a IRF520 MOSFET module or similar for the relay control outputs, as the ESP32 GPIOs cannot drive relays directly. *****

---

### Repository Status

This is an active development fork focused on stability, correctness, and full Victron Venus OS integration while staying true to the original TeslaBMS design philosophy.

Contributions, issues, and feature requests are welcome.

---

## Staging Area: ANENJI Inverter Venus OS Driver

The `Anenji Inverter/` directory contains a **standalone Venus OS driver** for the
ANENJI ANJ-3KW-24V-LV-WIFI inverter/charger. This code is **not part of the TeslaBMS-ESP32
project** and is staged here temporarily. It will be moved to its own dedicated repository.

See [Anenji Inverter/README.md](Anenji%20Inverter/README.md) for full documentation.
