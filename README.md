# TeslaBMS-ESP32

**ESP32 port of the original TeslaBMS project**

This firmware turns an ESP32 into a full-featured Battery Management System (BMS) master controller for **Tesla Model S/X battery modules** (6-cell 18650/2170 modules with the BQ76PL455A-Q1 chip).

It communicates with the modules over the original high-speed daisy-chain TTL UART protocol (612500 baud) and provides complete pack monitoring, balancing, contactor control, advanced SOC estimation, and comprehensive safety features.

---

### Based on

This project is a complete ESP32 port of the excellent original work by **collin80**:

→ [collin80/TeslaBMS](https://github.com/collin80/TeslaBMS)

The core communication protocol, module addressing, CRC handling, and register map are directly derived from the original implementation.

**Original wiring diagram** (highly recommended):
[https://cdn.hackaday.io/files/10098432032832/wiring.pdf](https://cdn.hackaday.io/files/10098432032832/wiring.pdf)

---

### Current State (7 April 2026)

The firmware is **stable and fully functional** for solar/off-grid, DIY EV packs, and similar applications. It is intentionally kept as a **flat codebase** so it can be opened and compiled directly in the Arduino IDE (GET VS CODE YALL) with no complex folder structure or external build tools required.

Further detailed documentation is available in the repository:
- [`systems.md`](systems.md) — Complete system architecture and component documentation
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — High-level design decisions and data flow

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
- Clean, modular architecture ready for secondary tasks (MQTT, WiFi web UI, Victron D-Bus, etc.)

### Planned features
- Victron OS integration layer on core 0.  
- some small todo's in todo file....

---

### Quick Start

1. Wire the modules exactly as shown in the [original wiring diagram](https://cdn.hackaday.io/files/10098432032832/wiring.pdf)
2. Connect:
   - `Serial2` (GPIO 16 RX / 17 TX) → BMS daisy-chain
   - GPIO 25 → Pre-charge relay
   - GPIO 26 → Main contactor relay
   - GPIO 34 → Optional current sensor (Hall effect)
3. Open the project in the Arduino IDE (flat folder structure)
4. Select your ESP32 board and flash
5. Open the serial monitor at 115200 baud and type `menu` for the interactive console

---

### Repository Status

This is an active development fork focused on stability, correctness, and preparing for easy addition of modern communication interfaces while staying true to the original TeslaBMS design philosophy.

Contributions, issues, and feature requests are welcome.

---

**Original Author:** collin80  
**ESP32 Port & Maintenance:** menzok
