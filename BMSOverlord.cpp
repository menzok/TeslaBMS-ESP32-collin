#include "BMSOverlord.h"
#include <esp_task_wdt.h>

void BMSOverlord::init() {
    Serial.println("BMSOverlord: Initializing...");

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

    // Enable ESP32 task watchdog (15 s for init)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 15000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

    Serial.println("BMSOverlord: Init complete - watchdog enabled (15 s)");
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
    // === Over-current / short-circuit (fast, no debounce) ===
    float currentA = socCalculator.getPackCurrentAmps();
    if (currentA > OVERCURRENT_THRESHOLD_A) {
        logFault(FaultEntry::Type::OverCurrent, 0, 0, currentA);
        currentState = BMSState::Fault;
        return;   // immediate fault
    }

    // === Per-cell voltage & temperature with debounce ===
    uint8_t modules = bms.getNumberOfModules();
    bool anyFault = false;

    for (uint8_t m = 0; m < modules && m < 32; m++) {
        for (uint8_t c = 0; c < 6; c++) {   // Tesla modules have 6 cells
            CellDetails cell = bms.getCellDetails(m, c);

            float voltage = cell.cellVoltage;
            float tempC = cell.highTemp - 40.0f;   // remove +40 offset that getCellDetails() adds

            // Raw threshold checks
            bool overVoltage = (voltage > eepromdata.OverVSetpoint);
            bool underVoltage = (voltage < eepromdata.UnderVSetpoint);
            bool overTemp = (tempC > eepromdata.OverTSetpoint);
            bool underTemp = (tempC < eepromdata.UnderTSetpoint);

            // --- Over Voltage debounce ---
            if (overVoltage) {
                if (++ovDebounce[m] >= CELL_FAULT_DEBOUNCE) {
                    anyFault = true;
                    logFault(FaultEntry::Type::OverVoltage, m, c, voltage);
                    ovDebounce[m] = CELL_FAULT_DEBOUNCE;
                }
            }
            else {
                ovDebounce[m] = 0;
            }

            // --- Under Voltage debounce ---
            if (underVoltage) {
                if (++uvDebounce[m] >= CELL_FAULT_DEBOUNCE) {
                    anyFault = true;
                    logFault(FaultEntry::Type::UnderVoltage, m, c, voltage);
                    uvDebounce[m] = CELL_FAULT_DEBOUNCE;
                }
            }
            else {
                uvDebounce[m] = 0;
            }

            // --- Over Temperature debounce ---
            if (overTemp) {
                if (++otDebounce[m] >= CELL_FAULT_DEBOUNCE) {
                    anyFault = true;
                    logFault(FaultEntry::Type::OverTemperature, m, c, tempC);
                    otDebounce[m] = CELL_FAULT_DEBOUNCE;
                }
            }
            else {
                otDebounce[m] = 0;
            }

            // --- Under Temperature debounce ---
            if (underTemp) {
                if (++utDebounce[m] >= CELL_FAULT_DEBOUNCE) {
                    anyFault = true;
                    logFault(FaultEntry::Type::UnderTemperature, m, c, tempC);
                    utDebounce[m] = CELL_FAULT_DEBOUNCE;
                }
            }
            else {
                utDebounce[m] = 0;
            }
        }
    }

    // Update overall BMS state
    if (anyFault) {
        currentState = (currentState == BMSState::Normal) ? BMSState::Warning : BMSState::Fault;
    }
    else {
        if (currentState == BMSState::Fault || currentState == BMSState::Warning) {
            currentState = BMSState::Normal;
            clearLastFaultIfResolved();
        }
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
    Serial.println("BMSOverlord: Shutdown requested - entering Storage mode");
}

void BMSOverlord::requestStartup() {
    shutdownRequested = false;
    storageModeActive = false;
    Serial.println("BMSOverlord: Startup requested - returning to Normal mode");
}