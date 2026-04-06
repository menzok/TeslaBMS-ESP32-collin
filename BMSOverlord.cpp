#include "BMSOverlord.h"
#include <esp_task_wdt.h>
#include "Logger.h"

void BMSOverlord::init() {
    Serial.println("BMSOverlord: Initializing...");
    // Enable ESP32 task watchdog (15 s for init)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 15000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

    Serial.println("BMSOverlord: Init complete - watchdog enabled (15 s)");

    contactor.init();
    SERIALCONSOLE.println("Scanning for connected BMS boards..."); //Scan for modules and print results
    bms.findBoards();
    if (bms.getNumberOfModules() == 0) {
        SERIALCONSOLE.println("No modules detected. Setup required.");
    }
    else {
        SERIALCONSOLE.printf("Found %d connected BMS modules\n", bms.getNumberOfModules());
    }
    socCalculator.begin();   
}


void BMSOverlord::update() {
    uint32_t now = millis();

    esp_task_wdt_reset();

    if (!watchdogTightened && ++successfulUpdates >= 5) {
        esp_task_wdt_config_t twdt_tight = {
            .timeout_ms = 1000,
            .idle_core_mask = 0,
            .trigger_panic = true
        };
        esp_task_wdt_reconfigure(&twdt_tight);
        watchdogTightened = true;
        Serial.println("BMSOverlord: Watchdog tightened to 1 second....GET OVER HERE");
	}

    bms.getAllVoltTemp();
    bms.balanceCells();
    contactor.update();
    socCalculator.update();
    runSafetyChecks();
    handleContactorLogic();
    handleStorageMode();

    lastUpdateMs = now;
}


void BMSOverlord::runSafetyChecks() {

    float currentA = socCalculator.getPackCurrentAmps();
    bool anyFault = false;

    // Overcurrent - immediate, no debounce
    if (currentA > OVERCURRENT_THRESHOLD_A) {
        if (currentState != BMSState::Fault) {          // rising edge
            logFault(FaultEntry::Type::OverCurrent, 0, 0, currentA);
            Serial.printf("OVERCURRENT FAULT! %.1f A\n", currentA);
        }
        anyFault = true;
    }

    for (uint8_t m = 0; m < bms.getNumberOfModules(); m++) {
        for (uint8_t c = 0; c < 6; c++) {
            CellDetails cell = bms.getCellDetails(m, c);
            float voltage = cell.cellVoltage;
            float tempC = cell.highTemp - 40.0f;

            bool overVoltage = (voltage > eepromdata.OverVSetpoint);
            bool underVoltage = (voltage < eepromdata.UnderVSetpoint);
            bool overTemp = (tempC > eepromdata.OverTSetpoint);
            bool underTemp = (tempC < eepromdata.UnderTSetpoint);

            ///leaving this here, but debugging was not fun...
        //    Serial.printf("M%d C%d  V=%.3fV  T=%.1f°C  ov=%d uv=%d ot=%d ut=%d\n",
        //        m, c, voltage, tempC, overVoltage, underVoltage, overTemp, underTemp);

            // Per-cell debounce + log ONLY ONCE when it first hits the threshold
            if (overVoltage) {
                if (++ovDebounce[m][c] == CELL_FAULT_DEBOUNCE) {   // note: == not >=
                    logFault(FaultEntry::Type::OverVoltage, m, c, voltage);
                    anyFault = true;
                }
            }
            else {
                ovDebounce[m][c] = 0;
            }

            if (underVoltage) {
                if (++uvDebounce[m][c] == CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::UnderVoltage, m, c, voltage);
                    anyFault = true;
                }
            }
            else {
                uvDebounce[m][c] = 0;
            }

            if (overTemp) {
                if (++otDebounce[m][c] == CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::OverTemperature, m, c, tempC);
                    anyFault = true;
                }
            }
            else {
                otDebounce[m][c] = 0;
            }

            if (underTemp) {
                if (++utDebounce[m][c] == CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::UnderTemperature, m, c, tempC);
                    anyFault = true;
                }
            }
            else {
                utDebounce[m][c] = 0;
            }
        }
    }

    // State machine with proper recovery
    if (anyFault && currentState != BMSState::Fault) {
        currentState = BMSState::Fault;
        Logger::warn("*** FAULT DETECTED - Entering Fault state ***");
    }
    else if (!anyFault && currentState == BMSState::Fault) {
        currentState = BMSState::Normal;
        clearLastFaultIfResolved();   // marks the fault as cleared in the log
        Logger::warn("*** FAULT CLEARED - Returning to Normal state ***");
    }
}

void BMSOverlord::clearLastFaultIfResolved() {
    // Find the last (newest) entry that is still active
    for (int8_t i = 4; i >= 0; i--) {
        if (faultLog[i].type != FaultEntry::Type::None && faultLog[i].clearedTimestamp == 0) {
            faultLog[i].clearedTimestamp = millis();
            EEPROMSettings::save();
            return;
        }
    }
}

void BMSOverlord::handleContactorLogic() {
    if (currentState == BMSState::Fault || currentState == BMSState::Shutdown) {
        contactor.open();
    }
    else if (currentState == BMSState::Normal) {           // ← Only Normal should auto-close
        if (contactor.getState() != ContactorState::CONNECTED) {
            contactor.close();
        }
    }
}

void BMSOverlord::handleStorageMode() {
    if (!storageModeActive) return;
    uint32_t now = millis();
    if (now - lastStorageWakeMs >= STORAGE_WAKE_INTERVAL_MS) {
        lastStorageWakeMs = now;
        storageBalanceEndMs = now + STORAGE_BALANCE_DURATION_MS;
        bms.wakeBoards();
        balancingActive = true;
        Serial.println("BMSOverlord: Storage wake - balancing started");
    }
    if (balancingActive && now >= storageBalanceEndMs) {
        bms.sleepBoards();
        balancingActive = false;
        Serial.println("BMSOverlord: Storage cycle complete - boards slept");
    }
}

void BMSOverlord::logFault(FaultEntry::Type type, uint8_t module, uint8_t cell, float value) {
    FaultEntry newEntry;
    newEntry.type = type;
    newEntry.module = module;
    newEntry.cell = cell;
    newEntry.value = value;
    newEntry.timestamp = millis();
    newEntry.clearedTimestamp = 0;

    // Shift everything left (drop oldest), put new one at the end
    for (uint8_t i = 0; i < 4; i++) {
        faultLog[i] = faultLog[i + 1];
    }
    faultLog[4] = newEntry;   // always put newest at slot [4]

    EEPROMSettings::save();
}



void BMSOverlord::requestShutdown() {
    shutdownRequested = true;
    storageModeActive = true;
    Logger::warn("BMSOverlord: Shutdown requested - entering Storage mode");
}

void BMSOverlord::requestStartup() {
    shutdownRequested = false;
    storageModeActive = false;
    Logger::warn("BMSOverlord: Startup requested - returning to Normal mode");
}