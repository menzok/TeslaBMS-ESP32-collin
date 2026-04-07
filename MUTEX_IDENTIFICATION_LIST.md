# MUTEX_IDENTIFICATION_LIST.md
# Locations requiring mutex/synchronization protection when a second FreeRTOS task is added
#
# Format: ClassName::FunctionName() → VariableOrDataStructure - Brief reason (Risk: High/Medium/Low)
#
# Context: Currently the firmware runs single-threaded through BMSOverlord::update() called from
# the Arduino loop() task. A planned second task (MQTT, WiFi web server, Victron D-Bus, etc.)
# will read and write eepromdata and internal BMS state concurrently. Every site listed below
# will need a FreeRTOS mutex (or equivalent) acquired before the first access and released after
# the last access in the same logical operation.

---

## eepromdata global struct (PRIMARY SHARED RESOURCE)
# eepromdata is a ~100-byte struct accessed by every major component. On ESP32, a 32-bit float
# write is atomic but a multi-field sequence is not. Any function that reads or writes more than
# one field of eepromdata is a potential torn-read/write site.

##### oh man... what a mess.....

**No, Not exposed to second layer**  EEPROMSettings::load()                  → eepromdata (entire struct)                                            - Full struct write during load; second task reading any field mid-load gets partially old, partially new data (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::save()                  → eepromdata (entire struct)                                            - Full struct read for EEPROM.put(); concurrent write from second task causes torn read to flash (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::loadDefaults()          → eepromdata (multiple fields via three sub-calls)                      - Multi-field write across three functions; second task reading any threshold mid-reset gets inconsistent state (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::resetSafetyThresholds() → eepromdata.OverVSetpoint / UnderVSetpoint / OverTSetpoint / UnderTSetpoint / balanceVoltage / balanceHyst / OVERCURRENT_THRESHOLD_A / CELL_FAULT_DEBOUNCE - Batch field write; safety thresholds must be updated atomically or runSafetyChecks() may use a mix of old and new limits (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::resetAdditionalHardware() → eepromdata.prechargeEnabled / prechargeTimeoutMs / currentSensorPresent / currentSensorVbias / currentSensorVrange / currentSensorRatedAmps - Batch field write; contactor and SOC calculator depend on these being consistent (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::resetBatteryConfig()    → eepromdata.parallelStrings / STORAGE_WAKE_INTERVAL_MS / STORAGE_BALANCE_DURATION_MS / socPercent / coulombCountAh - Batch field write; BMS voltage and SOC calculations use these as divisors (Risk: High)
**No, Not exposed to second layer**  EEPROMSettings::resetFaultLog()         → eepromdata.faultLog[5]                                                - Array memset; second task reading faultLog mid-clear gets partially zeroed entries (Risk: Medium)

BMSOverlord::runSafetyChecks()          → eepromdata.OverVSetpoint / UnderVSetpoint / OverTSetpoint / UnderTSetpoint / OVERCURRENT_THRESHOLD_A / CELL_FAULT_DEBOUNCE - Safety threshold reads spanning a full module×cell nested loop; thresholds must not change mid-check (Risk: High)
BMSOverlord::handleStorageMode()        → eepromdata.STORAGE_WAKE_INTERVAL_MS / STORAGE_BALANCE_DURATION_MS    - Timer parameter reads; if changed by second task while a storage cycle is running the cycle length becomes undefined (Risk: Medium)
BMSOverlord::logFault()                 → eepromdata.faultLog[5]                                                - Array shift + write; second task reading faultLog (e.g. for MQTT publish) during the shift gets a torn entry (Risk: High)
BMSOverlord::clearLastFaultIfResolved() → eepromdata.faultLog[5].clearedTimestamp                               - Single field write into the log array; second task may be iterating the same array (Risk: High)

BMSModuleManager::getAllVoltTemp()      → eepromdata.parallelStrings                                            - Used as divisor for packVolt; changing parallelStrings mid-calculation corrupts the pack voltage result (Risk: High)
BMSModuleManager::getBatterySummary()  → eepromdata.socPercent                                                  - SOC read for summary struct; second task writing socPercent mid-read gives a torn float (Risk: Medium)
BMSModuleManager::getModuleSummary()   → eepromdata.socPercent                                                  - Same as getBatterySummary() (Risk: Medium)

BMSModule::balanceCells()               → eepromdata.balanceVoltage / balanceHyst                               - Threshold reads inside per-cell loop; changed mid-loop by second task (Risk: Medium)

SOCCalculator::begin()                  → eepromdata.parallelStrings / socPercent / currentSensorPresent / currentSensorVbias / currentSensorVrange / currentSensorRatedAmps - Multi-field init read; second task writing any field during begin() leaves the calculator in a bad initial state (Risk: High)
SOCCalculator::update()                 → eepromdata.socPercent / coulombCountAh                                - Float read-modify-write every tick; second task reading socPercent mid-update gets a partially updated value (Risk: High)
SOCCalculator::update()                 → eepromdata.currentSensorPresent / currentSensorVbias / currentSensorVrange / currentSensorRatedAmps - Sensor config reads; changed mid-update by second task corrupts the current sample (Risk: Medium)
SOCCalculator::getPackCurrentAmps()     → eepromdata.currentSensorPresent                                       - Single bool read, but value can change between the check and the use of its result (Risk: Low)
SOCCalculator::_readCurrentAmps()       → eepromdata.currentSensorRatedAmps / currentSensorVbias / currentSensorVrange - Multiplier reads; corrupted if changed mid-calculation (Risk: Medium)

ContactorController::close()            → eepromdata.prechargeEnabled                                           - Bool read that drives a relay GPIO; must be consistent with the matching read in update() (Risk: High)
ContactorController::update()           → eepromdata.prechargeEnabled / prechargeTimeoutMs / currentSensorPresent - Timer and mode reads; contactor state machine depends on these being stable across a single call (Risk: High)

---

## BMSOverlord internal state

BMSOverlord::runSafetyChecks()          → currentState                                                          - State write; second task calling getState() or isFaulted() between the write and the next check sees a transitional value (Risk: High)
BMSOverlord::handleContactorLogic()     → currentState                                                          - State read driving relay GPIO; must see the same value as runSafetyChecks() just set (Risk: High)
BMSOverlord::requestShutdown()          → storageModeActive                                                      - Bool write from second task; handleStorageMode() reads it on the main task every cycle (Risk: High)
BMSOverlord::requestStartup()           → storageModeActive                                                      - Same as requestShutdown() (Risk: High)
BMSOverlord::getState()                 → currentState                                                          - Read by second task for MQTT/Victron publish; non-atomic if state transitions concurrently (Risk: Medium)
BMSOverlord::isFaulted()                → currentState                                                          - Same as getState() (Risk: Medium)

---

## BMSModuleManager internal data

BMSModuleManager::getAllVoltTemp()      → packVolt / lowestPackTemp / highestPackTemp / modules[].cellVolt[] / modules[].temperatures[] - Full write pass over all module data; second task reading any of these during the pass gets a mix of old and new readings (Risk: High)
BMSModuleManager::getPackVoltage()      → packVolt                                                               - Float read; written concurrently by getAllVoltTemp() (Risk: High)
BMSModuleManager::getBatterySummary()  → packVolt / lowestPackTemp / highestPackTemp                             - Multi-field read; any field can be mid-write from getAllVoltTemp() (Risk: High)
BMSModuleManager::getCellDetails()      → modules[].cellVolt[] / modules[].temperatures[]                       - Cell data read; written concurrently by readModuleValues() inside getAllVoltTemp() (Risk: High)
BMSModuleManager::printPackSummary()   → modules[] (all fields)                                                  - Long serial dump iterating all modules; getAllVoltTemp() may update data mid-print (Risk: Medium)
BMSModuleManager::printPackDetails()   → modules[] (all fields)                                                  - Same as printPackSummary() (Risk: Medium)
BMSModuleManager::getNumberOfModules() → numFoundModules                                                         - Read; written by findBoards() / setupBoards() / renumberBoardIDs() if called from second task (Risk: Medium)

---

## SOCCalculator internal state

SOCCalculator::update()                 → _filteredCurrentA / _lastCurrentA / _lastUpdateMs / _fullConfirmTicks / _emptyConfirmTicks - All runtime state writes; second task calling getPackCurrentAmps() mid-update reads a partially updated current value (Risk: High)
SOCCalculator::getPackCurrentAmps()     → _filteredCurrentA                                                      - Float read; written by update() on the main task (Risk: High)

---

## Logger static state

Logger::setLoglevel()                   → Logger::logLevel                                                       - Static write; second task calling setLoglevel() via MQTT while main task checks level in every log call (Risk: Medium)
Logger::log()                           → Logger::lastLogTime                                                    - Static write; harmless race but worth noting (Risk: Low)

---

## Hardware serial (UART) — shared physical resource

BMSUtil::sendData()                     → SERIAL (Serial2 / BMS UART TX)                                        - If second task also calls sendData(), bytes interleave and corrupt BMS protocol frames (Risk: High)
BMSUtil::getReply()                     → SERIAL (Serial2 / BMS UART RX buffer)                                 - Concurrent reads consume each other's bytes and corrupt frame parsing (Risk: High)
Logger::log() / logMessage()            → SERIALCONSOLE (Serial / USB UART TX)                                   - Second task printing debug messages interleaves with BMS log output; garbled but not safety-critical (Risk: Low)
