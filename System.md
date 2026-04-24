# ===========================================================================================================================
# BMSUtil.h / BMSUtil Class
# ===========================================================================================================================

This header provides the low-level serial communication utilities used throughout the TeslaBMS-ESP32 firmware to talk to the individual Tesla battery modules.
It implements the exact packet format, addressing, CRC checksum, and send/receive logic required by the Tesla BMS daisy-chain protocol.
All functionality lives in the BMSUtil class. Every method is static, so no object instance is ever created.
The class is used by BMSModule and BMSModuleManager whenever they need to read or write registers on a specific module.

## Public Static Functions

**`static uint8_t genCRC(uint8_t *input, int lenInput)`**

Calculates the 8-bit CRC for a command or response packet using generator polynomial 0x07.

This is the standard CRC-8 algorithm used by the Tesla BMS modules to verify packet integrity. The function is called automatically by `sendData()` for write packets.

**`static void sendData(uint8_t *data, uint8_t dataLen, bool isWrite)`**

Transmits a complete command packet over the SERIAL port (the hardware UART connected to the BMS chain). The first byte (`data[0]`) is the module address.
If `isWrite == true`, the LSB of the address byte is set to 1 (write flag) and the CRC is appended at the end.
The original value of `data[0]` is restored before the function returns.
When `Logger::isDebug()` is enabled, it prints the exact bytes being sent (helpful for troubleshooting).

**`static int getReply(uint8_t *data, int maxLen)`**

Reads the module's response from the SERIAL port into the supplied buffer. Reads up to `maxLen` bytes.
If more bytes arrive than the buffer can hold, the extras are discarded.
When debug logging is active, it prints every received byte in hex.
Returns the actual number of bytes read.

**`static int sendDataWithReply(uint8_t *data, uint8_t dataLen, bool isWrite, uint8_t *retData, int retLen)`**

Convenience wrapper that combines `sendData()` + `getReply()` with automatic retry logic. Sends the command, waits a short period 
(scaled to the expected response length), then reads the reply.
If the reply length does not exactly match `retLen`, it retries up to 3 times total.
This retry mechanism compensates for occasional glitches caused by the ESP32's serial timing or electrical noise on the BMS bus.
Returns the number of bytes actually received (ideally equal to `retLen`).

## Dependencies & Notes

- Includes `Logger.h` for conditional debug output.
- Relies on the global `SERIAL` (BMS communication UART) and `SERIALCONSOLE` (debug console) objects defined elsewhere in the project.
- The file contains a brief class comment: "Generates comms packets, and sends and receives."

This class forms the entire serial-protocol abstraction layer. Higher-level classes never talk directly to the hardware UART; they call these four functions instead.

# ===========================================================================================================================
# BMSModule.h / BMSModule.cpp / BMSModule Class
# ===========================================================================================================================

This class represents one individual Tesla BMS module in the daisy-chain.
It is the core data container and communication handler for a single physical module (which monitors 6 cells + 2 temperature sensors).

## Critical Architectural Point

All the data lives here. Voltages, temperatures, status/fault registers, balancing state, lifetime min/max statistics, packet counters, and module address are stored
exclusively inside each BMSModule instance.
Data ownership is never passed to any other class. 
BMSModuleManager and all higher-level code (web server, MQTT, display, etc.) can only read or interact with the data through the public accessor/getter functions.
There are no public member variables and no references or pointers to internal data are ever handed out.

## Public Functions

**`BMSModule()`**

Constructor. Resets all cell voltages to 0.0 V, initializes per-cell min/max trackers to safe extremes, clears temperatures and module voltage, sets `exists = false`, 
and zeros packet counters.

**`bool readModuleValues()`** *(central function — called repeatedly by the manager)*

Performs a complete read sequence for this module:
- Calls `readStatus()` to fetch alerts/faults/COV/CUV flags.
- Configures the ADC (auto mode, all inputs enabled).
- Starts conversions.
- Reads the GPAI register block (module voltage + 6 cell voltages + 2 temperatures).
- Validates packet length and CRC.
- Decodes raw 16-bit values using the exact scaling factors used by the Tesla modules:
  - Module voltage: `value * 0.002034609`
  - Cell voltage: `value * 0.000381493`
- Converts both temperature channels to °C using the Steinhart-Hart equation (implemented exactly as written in the code).
- Updates this module's running min/max trackers for voltages and temperatures.
- Increments `goodPackets` or `badPackets`.
- Returns `true` only on a fully valid response.

**`void readStatus()`**

Reads the alert, fault, COV (cell over-voltage), and CUV (cell under-voltage) status registers from the module and stores the raw byte values.

## Voltage Accessors *(all getters, no setters)*

- `float getCellVoltage(int cell)` — current voltage of one specific cell (0–5).
- `float getLowCellV()` / `float getHighCellV()` / `float getAverageV()` — aggregate stats across the 6 cells.
- `float getModuleVoltage()` — total pack voltage measured by this module.
- `float getHighestModuleVolt()` / `float getLowestModuleVolt()` — lifetime extremes for this module.
- `float getHighestCellVolt(int cell)` / `float getLowestCellVolt(int cell)` — lifetime high/low per individual cell.

## Temperature Accessors *(all getters)*

- `float getTemperature(int temp)` — current value of one of the two sensors (0 or 1).
- `float getLowTemp()` / `float getHighTemp()` / `float getAvgTemp()` — aggregate stats.
- `float getHighestTemp()` / `float getLowestTemp()` — lifetime extremes.

## Balancing

**`void balanceCells()`**

Reads the current cell voltages, compares each one against the EEPROM balance threshold + hysteresis, and sends the appropriate balancing command to the
module (turns passive balancing on/off per cell).

**`uint8_t getBalancingState(int cell)`** — returns 0 (off) or 1 (currently balancing) for a given cell.

## Module Identity & Existence

- `void setAddress(int newAddr)` / `int getAddress()`
- `bool isExisting()` / `void setExists(bool ex)` — used by the manager to mark whether a module actually responded during discovery.

## Fault / Status Getters

`uint8_t getFaults()` / `getAlerts()` / `getCOVCells()` / `getCUVCells()` — raw status bytes from the last `readStatus()` call.

## Packet Statistics

`goodPackets` / `badPackets` counters (accessible via getters or direct public members in the current code).

## Private Members *(summary — for documentation only)*

All the data listed above is stored privately and only exposed via the getters. The class is intentionally simple:
it only ever reads from one specific address and caches the results. No global state except the external `eepromdata` struct (used only inside `balanceCells()`).

# ===========================================================================================================================
# BMSModuleManager.h / BMSModuleManager.cpp / BMSModuleManager Class
# ===========================================================================================================================

This is the central orchestrator for the entire battery pack. It owns a fixed array of BMSModule objects (one slot per possible module address) and is responsible 
for discovery, addressing, system-wide commands, and providing aggregated pack-level data to the rest of the firmware.

## Key Architectural Point

The BMSModuleManager is the main (and only) accessor to the individual BMSModule instances.
Outside functions and classes (BMSOverlord, web server, MQTT, display, etc.) cannot access any BMSModule objects directly. They must go exclusively through the 
manager's public methods and the three read-only summary structs (`BatterySummary`, `ModuleSummary`, `CellDetails`). The internal `modules[]` array is 
completely encapsulated — no raw pointers, references, or direct module access is ever handed out.

The header also defines three public structs used by the rest of the system to get clean, read-only snapshots of data:

```cpp
struct BatterySummary { float voltage; float current; uint8_t soc; int8_t avgTemp; int8_t minTemp; int8_t maxTemp; };
struct ModuleSummary  { float voltage; float current; uint8_t soc; int8_t avgTemp; int8_t minTemp; int8_t maxTemp; };
struct CellDetails    { float cellVoltage; float highestCellVolt; float lowestCellVolt; int8_t highTemp; uint8_t faultBits; };
```

## Public Functions

### Setup & Discovery

**`BMSModuleManager()`** — Constructor. Initializes every slot in the `modules[]` array (sets address and `exists = false`), and resets pack-level min/max voltage, temperature, and **cell voltage** trackers to safe defaults.

**`void findBoards()`** — Scans addresses 1 to `MAX_MODULE_ADDR` by sending a short read command to each possible module. Any module that responds correctly is
marked `exists = true` and `numFoundModules` is incremented.

**`void setupBoards()`** — Handles automatic address assignment for any modules that are still at factory address 0. Keeps polling address 0 until
no more uninitialized boards respond, then assigns the next free address.

**`void renumberBoardIDs()`** — Broadcasts a full reset command to every module (forcing them back to address 0), then calls `setupBoards()` so modules are re-numbered 
sequentially in physical daisy-chain order.

### System-wide Control

**`void balanceCells()`** — Loops through every existing module and calls `modules[address].balanceCells()`.

**`void clearFaults()`** — Broadcasts commands (address `0x7F`) to clear both the Alert Status and Fault Status registers on every module.

**`void sleepBoards()`** / **`void wakeBoards()`** — Broadcasts an IO control command to set or clear the sleep bit on all modules (used for low-power states when 
the vehicle is off).

**`void getAllVoltTemp()`** — The main polling / data-collection routine (called repeatedly from the main loop or BMSOverlord).
For every existing module it calls `readModuleValues()`, accumulates the total pack voltage (sum of all module voltages, divided by `eepromdata.parallelStrings`
to support parallel strings), updates the pack's running min/max temperature trackers, and — new in v2.0 — tracks the **pack-wide lowest and highest individual
cell voltages** across all modules for telemetry and dynamic CVL calculation.

### Pack & Module Information (Accessors)

- `int getNumberOfModules() const` — Returns the current count of discovered modules.
- `float getPackVoltage()` — Returns the most recently calculated pack voltage.
- `float getAvgTemperature()` / `float getAvgCellVolt()` — Returns pack-wide averages.
- `float getLowestCellVoltage()` — Returns the lowest individual cell voltage across the entire pack (updated every `getAllVoltTemp()` call).
- `float getHighestCellVoltage()` — Returns the highest individual cell voltage across the entire pack.
- `float getMinTemperature()` — Returns the minimum module temperature across the pack.
- `float getMaxTemperature()` — Returns the maximum module temperature across the pack.
- `bool isAnyBalancing() const` — Returns `true` if any cell in any module is currently being actively balanced.
- `BatterySummary getBatterySummary()` — Returns the full pack-level summary struct (voltage, SOC from EEPROM, temperature stats).
- `ModuleSummary getModuleSummary(int module)` — Returns a summary struct for one specific module.
- `CellDetails getCellDetails(int module, int cell)` — Returns detailed info for one cell (current voltage, lifetime high/low, high temp, fault bits).

### Debug / Serial Output

**`void printPackSummary()`** — Prints a nicely formatted overview of the entire pack plus per-module status (voltages, temps, balancing state, faults, alerts).

**`void printPackDetails()`** — More verbose cell-by-cell output for troubleshooting.

## Private Members *(for documentation only)*

- `BMSModule modules[MAX_MODULE_ADDR + 1]` — Internal array of all module objects (never exposed directly).
- `int numFoundModules`
- Pack-level statistics (`packVolt`, `lowestPackVolt`, `highestPackVolt`, `lowestPackTemp`, `highestPackTemp`, `lowestPackCellV`, `highestPackCellV`).

This class sits at the top of the low-level BMS stack. Everything above it talks exclusively to the manager. No direct calls to individual BMSModule objects or 
BMSUtil occur outside this class.

# ===========================================================================================================================
# SOCCalculator.h / SOCCalculator.cpp / SOCCalculator Class
# ===========================================================================================================================

This class is responsible for calculating and maintaining the State of Charge (SOC) of the entire battery pack. It implements a three-path estimation strategy:

- **When a current sensor is installed** (`eepromdata.currentSensorPresent == true`): primary coulomb counting with trapezoidal integration + slow OCV-based drift
  correction while the pack is at rest.
- **When no internal sensor but Venus shunt data is fresh** (`ExternalComms.isShuntDataFresh()`): coulomb counting using the SmartShunt current value injected by
  the Venus OS driver via the ping-pong frame. Data is considered fresh only when `staleness == 0` **and** the last frame was received within `SHUNT_MAX_AGE_MS`
  (6 000 ms ≈ 3 missed polls at 2 s intervals).
- **When neither source is available**: pure OCV-based estimation with temperature compensation, blending slowly toward the OCV lookup result every update tick.

It uses the global `BMSModuleManager bms` instance to obtain the latest pack voltage and average temperature, and it directly reads from / writes to the global
`eepromdata` struct (specifically `socPercent` and `coulombCountAh`). The actual SOC value is not stored inside the SOCCalculator object.
The header contains a large static OCV-SOC lookup table (14 SOC points × 4 temperature points) with bilinear interpolation for accurate voltage-to-SOC conversion.

## Public Functions

**`SOCCalculator()`**

Constructor. Simply initializes internal state variables to defaults; no calibration or EEPROM access occurs here.

**`void begin()`**

Must be called once during system startup (before contactors close).
- Calculates `_cellsInSeries` from the current number of discovered modules and the parallel-strings setting.
- Sets `_packCapacityAh` based on parallel strings (hard-coded 232 Ah per string for standard Tesla 18650 modules).
- If a current sensor is configured, performs a short blocking zero-offset calibration on the ESP32 ADC (averages 32 samples).
- On cold boot (invalid SOC in EEPROM), seeds `eepromdata.socPercent` with an OCV lookup based on current average cell voltage and temperature.

**`void update()`**

The main SOC calculation routine. Called periodically (every `SOC_UPDATE_INTERVAL_MS`) from the scheduler / main loop.
- Reads the latest pack summary from `bms.getBatterySummary()` to get pack voltage and average temperature.
- **Path 1 — Internal sensor present:**
  - Reads and applies IIR low-pass filtering to the current measurement.
  - Performs coulomb counting (trapezoidal integration of current over elapsed time).
  - When pack current is near zero (< `SOC_ZERO_CURRENT_THRESHOLD`), slowly blends the coulomb-count SOC toward the OCV lookup result to correct long-term drift.
- **Path 2 — Venus shunt data fresh (`ExternalComms.isShuntDataFresh()`):**
  - Same coulomb counting logic as path 1, using the shunt current received from Venus.
  - Falls through to path 3 if `isShuntDataFresh()` returns false (stale data, Venus offline, or staleness counter > 0).
- **Path 3 — No usable current source:**
  - Applies pure OCV blending every update. Filtered current state is reset so it does not drift.
- Applies "hard reset" logic: if average cell voltage stays above `SOC_CELL_FULL_VOLTAGE` (or below `SOC_CELL_EMPTY_VOLTAGE`) for `SOC_RESET_CONFIRM_TICKS` consecutive
  calls, it forces SOC to 100% or 0% and resets the coulomb counter.
- Always clamps `eepromdata.socPercent` between 0 and 100.

**`float getPackCurrentAmps() const`**

Returns the filtered pack current in amps (positive = charge, negative = discharge).
Returns `0.0` if no current sensor is configured. Used by `ContactorController` for pre-charge detection.

**`uint8_t getSOCByte() const`**

Returns the current SOC as an integer byte value (0–100). This is the value placed into `BatterySummary.soc` for the rest of the system.

## Private Helpers *(for documentation only)*

- `_readCurrentAmps()` — oversamples the ADC, applies zero offset and scaling.
- `_adcToVoltage()` — converts raw 12-bit ADC reading to voltage using ESP32 11 dB attenuation constants.
- `_ocvToSOC()` — bilinear interpolation on the static OCV-SOC lookup table using average cell voltage and temperature.
- `_clampSOC()` — simple 0–100 clamp.

This class operates "sideways" to the core BMS communication stack. It does not interact with individual modules or the serial bus — it only consumes 
pack-level data from the manager and updates the persistent EEPROM-based SOC state.

# ===========================================================================================================================
# ContactorController.h / ContactorController.cpp / ContactorController Class
# ===========================================================================================================================

This class implements a non-blocking state-machine controller for the pack's main contactor and pre-charge relay. It handles the safe sequencing required
to connect the battery pack to the vehicle's high-voltage system without inrush current damage.

It drives two GPIO pins (`PRECHARGE_RELAY_PIN` and `CONTACTOR_RELAY_PIN` — defined in `config.h`) and uses the global `eepromdata` settings and the 
`SOCCalculator` (for current sensing) to decide when pre-charge is complete.

## Public Functions

**`void init()`**

Resets the internal state machine to `OPEN` (both relays off). Called once at startup.

**`void open()`**

Immediately de-energizes both the pre-charge relay and the main contactor relay, forcing the state to `OPEN`.
Used for emergency disconnect, fault handling, or normal shutdown.

**`void close()`**

Starts the contactor-closure sequence.
- If the pack is already `CONNECTED`, does nothing.
- Records the start time and turns on the pre-charge relay (if `eepromdata.prechargeEnabled` is true).
- Sets state to `PRECHARGING`.
- If pre-charge is disabled in EEPROM, it skips straight to turning on the main contactor and goes directly to `CONNECTED`.

**`void update()`**

The non-blocking state machine heart — must be called regularly (typically every loop iteration or from the scheduler).
It only acts when in the `PRECHARGING` state:
- Checks whether pre-charge is finished using one of two methods (in order of preference):
  - **Current sensor (if present):** waits until `socCalculator.getPackCurrentAmps()` drops below 0.5 A.
  - **Timed fallback (no current sensor):** waits `eepromdata.prechargeTimeoutMs` milliseconds.
- Enforces a hard safety timeout — if pre-charge takes longer than `eepromdata.prechargeTimeoutMs`, it calls `open()` and transitions to `FAULT`.
- Once pre-charge is complete, it energizes the main contactor, starts a short post-close delay, and moves to `CONNECTED`.
- In the `CONNECTED` state it turns the pre-charge relay back off after 500 ms (to reduce power draw and heat).

**`ContactorState getState() const`**

Returns the current state of the contactor controller: `OPEN`, `PRECHARGING`, `CONNECTED`, or `FAULT`.

The class has no public data members. All state is kept private (`currentState`, `prechargeStartTime`, `postCloseDelayStart`).
It never blocks or uses `delay()` — everything is time-based using `millis()`.

# ===========================================================================================================================
# config.h
# ===========================================================================================================================

This is the primary compile-time configuration header for the entire TeslaBMS-ESP32 project. It contains all the fixed, build-time settings that define how 
the firmware talks to the hardware.

Runtime-adjustable settings (balance threshold, pre-charge timeout, number of parallel strings, SOC calibration, etc.) live in the separate `EEPROMSettings`
class instead.
`config.h` is only changed when the physical hardware changes or when core low-level behavior needs tweaking. There are no classes or objects in this file —
just `#define` constants and one small inline helper function.

## Main Sections

- Serial / Communication Setup
- Hardware Pins & Features
- BQ76PL455A-Q1 Register Map
- Helper Function

**`inline void initPins()`**

Safely sets both relay pins to `LOW` (off) and configures them as outputs, then applies the correct ADC attenuation (`ADC_11db`) to the current-sensor pin so the
ESP32 can read the full voltage range of the Hall sensor.

# ===========================================================================================================================
# EEPROMSettings.h / EEPROMSettings.cpp / EEPROMSettings Class
# ===========================================================================================================================

This file provides the persistent storage layer for all runtime-configurable settings and a few pieces of operational state (SOC, coulomb counter, fault log).
Settings are stored in the ESP32's built-in EEPROM using the Arduino EEPROM library and survive power cycles and firmware uploads.

The design is deliberately simple:
- There is one global struct `EEPROMData eepromdata;` that contains ALL persistent data.
- The `EEPROMSettings` class is purely a static utility (no objects are ever created).
- Every other part of the firmware reads from and writes directly to the global `eepromdata` fields. No data ownership is passed around — the struct is the 
  single source of truth.

## EEPROMData Struct *(the actual data store)*

`FaultEntry` is a small nested struct that records the last 5 faults (type, module, cell, value, timestamps).

## Default Constants

A set of `constexpr` defaults is defined at the top of the header.
These are the factory values used on first boot or when a full reset is performed. They are tuned for safe solar/off-grid use with Tesla modules.

## Public Static Functions

**`static void load()`**

Called once in `setup()`. Reads the entire `eepromdata` struct from EEPROM address 0.
Checks version and checksum. If either is wrong, it treats the EEPROM as invalid, logs a message, clears the struct, calls `loadDefaults()`, and saves.
On success it logs "Settings loaded from EEPROM successfully" and applies the stored `logLevel` to the Logger.

**`static void save()`**

Writes the current `eepromdata` struct to EEPROM and calls `EEPROM.commit()`.
Automatically updates the checksum before writing. Used by the web UI "Save" button and by all the reset functions.

**`static void loadDefaults()`**

Full factory reset. Sets version, `logLevel`, then calls the four per-group reset functions below. Finally calls `save()`.

### Per-menu Reset Functions *(exactly as grouped in the web UI)*

- `static void resetSafetyThresholds()` — Resets OverV/UnderV/OverT/UnderT, `balanceVoltage`, `balanceHyst`, `OVERCURRENT_THRESHOLD_A`, and `CELL_FAULT_DEBOUNCE`.
- `static void resetAdditionalHardware()` — Resets `prechargeEnabled`, `prechargeTimeoutMs`, and all current-sensor fields.
- `static void resetBatteryConfig()` — Resets `parallelStrings`, `STORAGE_WAKE_INTERVAL_MS`, and `STORAGE_BALANCE_DURATION_MS`.
- `static void resetFaultLog()` — Clears the entire 5-entry fault history (not exposed in the web UI fault menu).

This class is the only place in the project that touches the EEPROM. All other files simply read/write the global `eepromdata` fields directly.

# ===========================================================================================================================
# BMSOverlord.h / BMSOverlord.cpp / BMSOverlord Class
# ===========================================================================================================================

This is the top-level coordinator / brain of the entire firmware. It sits above all the lower-level components (`BMSModuleManager`, `SOCCalculator`, 
`ContactorController`, and `EEPROMSettings`) and is responsible for:

- Orchestrating the main system loop (`update()`)
- Enforcing all safety rules (over-voltage, under-voltage, over-temp, under-temp, over-current)
- Managing contactor open/close decisions
- Handling Storage Mode (periodic wake + balance cycles for long-term storage)
- Logging faults to the persistent EEPROM fault log
- Providing the overall system state to the web UI, MQTT, and external control layers (e.g. Victron OS)

It does not own any battery data itself — it only reads from the global `bms`, `socCalculator`, and `eepromdata` instances.

## Public Interface

**`enum class BMSState : uint8_t { Normal, Warning, Fault, StorageMode, Shutdown };`**

The five possible high-level states of the BMS.  `Warning` is defined but currently unused.
**`Shutdown`** (value 3) is now a distinct state from `Fault` (value 1) — a clean external shutdown
no longer maps to the Fault state and does not trigger a false `InternalFailure` alarm in Venus OS.

**`void init()`**

One-time startup routine (called from `setup()`).
- Attaches the ESP32 task watchdog (initially 15 s, later tightened to 1 s).
- Calls `bms.findBoards()` to discover modules.
- Calls `socCalculator.begin()`.
- Initializes the contactor controller.

**`void update()`**

The main system heartbeat — called repeatedly from the Arduino `loop()`.
Performs this exact sequence every cycle:
1. Resets the task watchdog.
2. Calls `bms.getAllVoltTemp()` (the core data refresh).
3. Calls `bms.balanceCells()`.
4. Calls `contactor.update()`.
5. Calls `socCalculator.update()`.
6. Runs `runSafetyChecks()` (fault detection).
7. Runs `handleContactorLogic()`.
8. Runs `handleStorageMode()`.

**`void requestShutdown()`**

External command (from web UI or MQTT) to enter Storage Mode.

**`void requestStartup()`**

External command to exit Storage Mode and return to Normal operation.

**`BMSState getState() const`** / **`bool isFaulted() const`**

Status getters used by the web server, MQTT, and display code.

## How the Safety & State Logic Works *(private methods)*

**`runSafetyChecks()`**

The heart of fault detection.
- Over-current is checked immediately (no debounce) using `socCalculator.getPackCurrentAmps()`.
- For every cell in every module it checks OV, UV, OT, UT against the EEPROM thresholds.
- Each condition has its own per-cell debounce counter (using `eepromdata.CELL_FAULT_DEBOUNCE`). Faults are logged only once when the debounce counter first 
  hits the threshold.
- Transitions `currentState` to `Fault` on any active fault, or back to `Normal` when all faults clear.

**`handleContactorLogic()`**

Simple rules:
- `Fault` or `Shutdown` → force `contactor.open()`
- `Normal` → automatically call `contactor.close()` if not already connected.

**`handleStorageMode()`**

When `storageModeActive == true`:
- Every `STORAGE_WAKE_INTERVAL_MS` it wakes the boards and starts balancing for `STORAGE_BALANCE_DURATION_MS`.
- After the balance window it puts the boards back to sleep.

**`logFault(...)` and `clearLastFaultIfResolved()`**

Maintain the 5-entry rolling fault log in `eepromdata.faultLog[]` (newest entry always at index 4). Each entry records type, module, cell, value, timestamp, and 
cleared timestamp.

## Private Members *(for documentation only)*

- `BMSState currentState`
- `bool storageModeActive`
- Various timers and debounce arrays (`ovDebounce[64][6]`, etc.)
- `FaultEntry* faultLog` (points directly into the EEPROM struct)

This class is intentionally the single point where all high-level policy lives. Everything below it is either data acquisition (`BMSModuleManager`) or a 
specific subsystem (`SOCCalculator`, `ContactorController`).

# ===========================================================================================================================
# TeslaBMS-ESP32.ino *(the main Arduino sketch)*
# ===========================================================================================================================

This is the top-level entry point for the entire firmware. It is intentionally extremely minimal — its only job is to declare the global objects, perform one-time
hardware and settings initialization, and run the main system heartbeat at a controlled rate while keeping the serial console responsive.
No classes or complex logic live here. Everything is delegated to the objects documented in previous files.

## Global Object Instances *(created exactly once at startup)*

```cpp
BMSModuleManager bms;               // owns all module data & low-level comms
ContactorController contactor;      // contactor/pre-charge state machine
SOCCalculator socCalculator;        // SOC, coulomb counting & current sensing
BMSOverlord Overlord;               // high-level safety, state machine & orchestration
Menu menu;                          // interactive serial console (SerialConsoleMenu)
```

These five objects are the only globals in the entire project and are used directly by almost every other component.

# ===========================================================================================================================
# SerialConsoleMenu.h / SerialConsoleMenu.cpp / Menu Class
# ===========================================================================================================================

This is the interactive serial console for the entire TeslaBMS-ESP32 firmware. It provides a text-based menu system over `SERIALCONSOLE` (the USB debug port) 
that lets you view live data, change settings, reset EEPROM sections, view the fault log, and trigger certain actions — all without needing the web UI.

It is intentionally kept simple: a finite state machine that reacts to single-character commands typed by the user. The entire menu system lives in one class 
called `Menu` (declared in the header, implemented in the `.cpp`). There is one global instance `Menu menu;` created in the main `.ino`.

## How It Works *(High-Level Architecture)*

The menu runs non-blockingly inside `menu.loop()` (called on every iteration of `loop()` in the main sketch).
It uses a small command buffer (`cmdBuffer[80]`) to collect characters until the user presses Enter.
Input is handled by `handleInput(char c)` which dispatches based on the current `MenuState`.
Most menus are display-only or have a live pretty-display mode (refreshes every few seconds while you watch).
When you choose to edit a value, the system switches to `WAITING_FOR_INPUT` state and routes the next numeric input to the correct `PendingEdit` handler.
All changes you make are written immediately to the global `eepromdata` struct (via `EEPROMSettings::save()` where appropriate) so they survive power cycles.
The menu never blocks the main BMS loop — `Overlord.update()` continues running in the background.

## Menu Layout *(the state machine)*

The class uses an enum `MenuState` with these top-level screens:

- `ROOT_MENU` — main landing page with quick status and options to jump to sub-menus.
- `CONFIG_MENU` — safety thresholds (OV/UV/OT/UT, balance voltage & hysteresis, over-current trip, cell-fault debounce).
- `MODULE_MENU` — live per-module / per-cell voltage & temperature display (pretty mode available).
- `LOGGING_MENU` — change the Logger verbosity level.
- `DEFAULTS_MENU` — one-click factory resets for different groups of settings.
- `CONTACTOR_CURRENT_MENU` — pre-charge timeout, current-sensor calibration (bias, range, rated amps).
- `FAULT_LOG_MENU` — read-only view of the last 5 faults stored in EEPROM.
- `BATTERY_CONFIG_MENU` — parallel strings, storage-mode wake interval (hours), storage balance duration (minutes), plus live pack summary.
- `WAITING_FOR_INPUT` — temporary state while editing a numeric value.

There is also a `PendingEdit` enum that lists every editable field (one entry per setting). When you select an edit option, `pendingEdit` is set and the next 
numeric input is routed to the correct handler (`handleConfigWaitingInput()`, `handleBatteryConfigWaitingInput()`, etc.).

## Pretty-Display Mode

Several menus (especially Module and Battery Config) support a live updating "pretty" view.
- Type `p` to toggle it on/off.
- While active, the menu prints a refreshed summary every few seconds and shows a warning that you must type `x` to exit.

## What a Developer / User Might Want to Edit or Extend

### Add a new editable setting
1. Add a new entry to the `PendingEdit` enum (header).
2. Add a new case in the appropriate `printXXXMenu()` function (shows the current value).
3. Add the edit prompt in the matching `handleXXXCommand(char c)`.
4. Add handling logic in the corresponding `handleXXXWaitingInput()` function (parse the number, store it in `eepromdata`, call `EEPROMSettings::save()`).
5. *(Optional)* Add it to `loadDefaults()` / `resetXXX()` in `EEPROMSettings` if it belongs to a reset group.

### Add an entirely new sub-menu
1. Add a new value to `MenuState`.
2. Add `printNewMenu()` and `handleNewCommand(char c)`.
3. Add the menu entry in `printRootMenu()` and the jump logic in `handleRootCommand()`.
4. Register any waiting-input handler if the new menu has editable fields.

### Change menu text / help
All the printed help text lives inside the `printXXXMenu()` functions in the `.cpp` — easy to tweak wording or layout.

### Change refresh rate of pretty display
Controlled by `prettyCounter` and the logic inside `loop()` (currently tied to the main 1 Hz update).

### Disable / remove menus
Simply comment out the relevant cases in `handleRootCommand()` and remove the menu from the root printout.

The menu is completely decoupled from the BMS logic — it only reads from `bms`, `socCalculator`, `eepromdata`, and `Logger`. No safety or control code lives here.

# ===========================================================================================================================
# ExternalCommsLayer.h / ExternalCommsLayer.cpp / ExternalCommsLayer Class
# ===========================================================================================================================

This class implements the Venus OS ↔ ESP32 serial communication protocol.  It is the only component
that talks over `EXTERNAL_COMM_SERIAL` (defined in `config.h`, default 115200 baud).

There is one global instance `ExternalCommsLayer ExternalComms;` created in the main `.ino`.

## Role

- Receives command frames from the Venus OS driver on every loop iteration via `update()`.
- On a `CMD_SEND_DATA (0x03)` command, extracts the embedded shunt current + staleness counter,
  then immediately builds and sends the 37-byte telemetry reply.
- On `CMD_SHUTDOWN (0x01)` or `CMD_STARTUP (0x02)`, forwards the request to `BMSOverlord`.
- Exposes the last received shunt current and a freshness predicate to `SOCCalculator`.

## Protocol Summary

### Incoming command frames (Venus OS → ESP32)

| Command | Frame | Carries |
|---|---|---|
| `CMD_SEND_DATA (0x03)` | 7 bytes | Shunt current (int16 BE × 10) + staleness byte + CRC |
| `CMD_SHUTDOWN (0x01)` | 4 bytes | CRC only |
| `CMD_STARTUP (0x02)` | 4 bytes | CRC only |

All frames start with `0xAA`.  CRC-16/MODBUS is verified on every frame; corrupt frames are silently
discarded.  Remaining bytes are read using `Serial.readBytes()` which honours the hardware timeout —
no spin-waits or `available()` polling.

### Outgoing telemetry frame (ESP32 → Venus OS)

37 bytes: `[0xAA]` + 34-byte payload + 2-byte CRC-16/MODBUS.  Sent only in response to
`CMD_SEND_DATA`.  See the Venus OS driver README for the full payload map.

`buildPayload()` assembles all 34 bytes from live BMS data each time it is called:

| Payload region | Source |
|---|---|
| Pack voltage, current, SOC, avg temp, power, avg cell V | `bms.getBatterySummary()` + `socCalculator` |
| Alarm flags | Per-cell/current threshold checks against EEPROM values |
| Overlord state (0–3) | `Overlord.getState()` — 0=Normal, 1=Fault, 2=Storage, **3=Shutdown** |
| Contactor state | `contactor.getState()` |
| EEPROM thresholds | `eepromdata.*` directly |
| statusFlags (bit0=currentSensorPresent, bit1=balancingActive) | `eepromdata.currentSensorPresent`, `bms.isAnyBalancing()` |
| activeFaultMask | Fault log entries with `clearedTimestamp == 0` → `1 << FaultEntry::Type` |
| lowestCellV / highestCellV | `bms.getLowestCellVoltage()` / `bms.getHighestCellVoltage()` |
| minTemp / maxTemp | `bms.getMinTemperature()` / `bms.getMaxTemperature()` |

## Public Interface

**`void init()`** — Sets the serial port baud rate. Call once from `setup()`.

**`void update()`** — Reads and processes one waiting command frame. Call every `loop()` iteration
after `Overlord.update()` so the telemetry reply contains the most recent data.

**`float getShuntCurrentAmps() const`** — The last shunt current received from Venus (A, + = charge).

**`uint8_t getShuntStaleness() const`** — The staleness counter from the last `CMD_SEND_DATA` frame.
`0` = Venus had fresh SmartShunt data; higher values mean Venus could not read the shunt.

**`bool isShuntDataFresh() const`** — Returns `true` when:
- `_shuntStaleness == 0` (Venus had fresh data this poll), **and**
- the last `CMD_SEND_DATA` frame arrived within `SHUNT_MAX_AGE_MS` (6 000 ms).

Used by `SOCCalculator::update()` to decide whether the Venus shunt path is usable.

## Constants (ExternalCommsLayer.h)

| Constant | Value | Meaning |
|---|---|---|
| `EXT_CMD_SHUTDOWN` | `0x01` | Command: open contactors |
| `EXT_CMD_STARTUP` | `0x02` | Command: close contactors |
| `EXT_CMD_SEND_DATA` | `0x03` | Command: request telemetry (carries shunt current) |
| `EXT_SEND_DATA_FRAME_LEN` | `7` | Length of CMD_SEND_DATA request frame |
| `EXT_CTRL_FRAME_LEN` | `4` | Length of SHUTDOWN/STARTUP frame |
| `EXT_FRAME_LEN` | `37` | Length of telemetry reply frame |
| `PAYLOAD_LEN` | `34` | Payload bytes (excludes start byte and CRC) |
| `SHUNT_MAX_AGE_MS` | `6000` | Shunt data expires after 6 s (≈ 3 missed polls) |
| `OVERLORD_STATE_NORMAL` | `0` | |
| `OVERLORD_STATE_FAULT` | `1` | |
| `OVERLORD_STATE_STORAGE` | `2` | |
| `OVERLORD_STATE_SHUTDOWN` | `3` | Clean shutdown — distinct from Fault |
| `STATUS_FLAG_CURRENT_SENSOR` | `bit 0` | `statusFlags`: internal current sensor fitted |
| `STATUS_FLAG_BALANCING` | `bit 1` | `statusFlags`: at least one cell balancing |
| `ALARM_OVER_VOLTAGE` | `bit 0` | `alarmFlags`: cell OV |
| `ALARM_UNDER_VOLTAGE` | `bit 1` | `alarmFlags`: cell UV |
| `ALARM_OVER_TEMP` | `bit 2` | `alarmFlags`: pack OT |
| `ALARM_UNDER_TEMP` | `bit 3` | `alarmFlags`: pack UT |
| `ALARM_OVER_CURRENT` | `bit 4` | `alarmFlags`: pack OC |

# ===========================================================================================================================
# Data Flow Summary
# ===========================================================================================================================

- `Overlord.update()` → `bms.getAllVoltTemp()` → `BMSModule::readModuleValues()` → `BMSUtil` serial I/O.
- Balancing, SOC, contactor, and safety checks run from the same heartbeat.
- All external interfaces (Venus OS driver, serial menu) read via `bms.getBatterySummary()`, `Overlord.getState()`, etc.
- Persistent state lives only in `eepromdata` and is saved explicitly.
- `ExternalComms.update()` is called every loop; it processes the next waiting command from Venus OS and, on `CMD_SEND_DATA`, fills the shunt data fields and immediately transmits the 37-byte telemetry reply.