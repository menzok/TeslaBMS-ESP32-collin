#include "BMSOverlord.h"
#include <esp_task_wdt.h>
#include "Logger.h"

void BMSOverlord::init() {
    faultLog = eepromdata.faultLog;
    Serial.println("BMSOverlord: Initializing...");
    // Enable ESP32 task watchdog (15 s for init)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

    Serial.println("BMSOverlord: Init complete - watchdog enabled (30 s)");

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
            .timeout_ms = 5000,
            .idle_core_mask = 0,
            .trigger_panic = true
        };
        esp_task_wdt_reconfigure(&twdt_tight);
        watchdogTightened = true;
        Serial.println("BMSOverlord: Watchdog tightened to 5 second....GET OVER HERE");
	}

    lastSuccessfulModules = bms.getAllVoltTemp();
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
    if (fabsf(currentA) > eepromdata.OVERCURRENT_THRESHOLD_A) {
        if (currentState != BMSState::Fault) {          // rising edge
            logFault(FaultEntry::Type::OverCurrent, 0, 0, currentA);
            Serial.printf("OVERCURRENT FAULT! %.1f A\n", currentA);
        }
        anyFault = true;
    }
   // Module Comms Saftey check
    uint8_t expectedModules = bms.getNumberOfModules();

    if (lastSuccessfulModules < expectedModules) {
        if (commsDebounce < 255) commsDebounce++;
        if (commsDebounce == eepromdata.CELL_FAULT_DEBOUNCE) {
            logFault(FaultEntry::Type::CommsError, 0, 0, expectedModules - lastSuccessfulModules);
            Serial.printf("COMMS FAULT: Only %u of %u modules responded!\n",
                lastSuccessfulModules, expectedModules);
        }
        if (commsDebounce >= eepromdata.CELL_FAULT_DEBOUNCE) anyFault = true;
    }
    else {
        commsDebounce = 0;   // good cycle → reset debounce
    }
    for (int m = 1; m <= MAX_MODULE_ADDR; m++) {
        if (!bms.moduleExists(m)) continue;
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

            // Per-cell debounce: cap counter at 255 to prevent wrap-around re-logging,
            // log the fault exactly once on reaching the threshold, assert anyFault while >= threshold.
            if (overVoltage) {
                if (ovDebounce[m][c] < 255) ovDebounce[m][c]++;
                if (ovDebounce[m][c] == eepromdata.CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::OverVoltage, m, c, voltage);
                }
                if (ovDebounce[m][c] >= eepromdata.CELL_FAULT_DEBOUNCE) anyFault = true;
            }
            else {
                ovDebounce[m][c] = 0;
            }

            if (underVoltage) {
                if (uvDebounce[m][c] < 255) uvDebounce[m][c]++;
                if (uvDebounce[m][c] == eepromdata.CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::UnderVoltage, m, c, voltage);
                }
                if (uvDebounce[m][c] >= eepromdata.CELL_FAULT_DEBOUNCE) anyFault = true;
            }
            else {
                uvDebounce[m][c] = 0;
            }

            if (overTemp) {
                if (otDebounce[m][c] < 255) otDebounce[m][c]++;
                if (otDebounce[m][c] == eepromdata.CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::OverTemperature, m, c, tempC);
                }
                if (otDebounce[m][c] >= eepromdata.CELL_FAULT_DEBOUNCE) anyFault = true;
            }
            else {
                otDebounce[m][c] = 0;
            }

            if (underTemp) {
                if (utDebounce[m][c] < 255) utDebounce[m][c]++;
                if (utDebounce[m][c] == eepromdata.CELL_FAULT_DEBOUNCE) {
                    logFault(FaultEntry::Type::UnderTemperature, m, c, tempC);
                }
                if (utDebounce[m][c] >= eepromdata.CELL_FAULT_DEBOUNCE) anyFault = true;
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
    if (now - lastStorageWakeMs >= eepromdata.STORAGE_WAKE_INTERVAL_MS) {
        lastStorageWakeMs = now;
        storageBalanceStartMs = now;
        bms.wakeBoards();
        balancingActive = true;
       // Serial.println("BMSOverlord: Storage wake - balancing started");
    }
    if (balancingActive && (now - storageBalanceStartMs) >= eepromdata.STORAGE_BALANCE_DURATION_MS) {
        bms.sleepBoards();
        balancingActive = false;
      //  Serial.println("BMSOverlord: Storage cycle complete - boards slept");
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

    // Rate-limit EEPROM writes to at most once per 60 seconds to reduce flash wear.
    // The fault is always recorded in RAM above; only the persist is deferred.
    uint32_t now = millis();
    if (now - lastFaultSaveMs >= 60000UL) {
        EEPROMSettings::save();
        lastFaultSaveMs = now;
    }
}



void BMSOverlord::requestShutdown() {
    storageModeActive = true;
    Logger::warn("BMSOverlord: Shutdown requested - entering Storage mode");
}

void BMSOverlord::requestStartup() {
    storageModeActive = false;
    Logger::warn("BMSOverlord: Startup requested - returning to Normal mode");
}