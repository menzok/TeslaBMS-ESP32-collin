/*
 * Tesla BMS Interactive Menu System for ESP32
 *
 * Based on original work by Collin Kidder / EVTV (2017)
 * Copyright (c) 2017 EVTV / Collin Kidder
 *
 * Heavily rewritten, restructured, and extended in 2026.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "SerialConsoleMenu.h"
#include "BMSModuleManager.h"
#include "EEPROMSettings.h"
#include "Logger.h"

extern BMSModuleManager bms;

// ======================================================
// SHARED FUNCTIONS
// ======================================================

Menu::Menu() {
    isMenuOpen = true;
    printPrettyDisplay = false;
    prettyCounter = 0;
    whichDisplay = 0;
    pendingEdit = NO_EDIT;
    ptrBuffer = 0;
    currentState = ROOT_MENU;
    printRootMenu();
}

void Menu::loop() {
    while (SERIALCONSOLE.available()) {
        handleInput((char)SERIALCONSOLE.read());
    }
    if (!isMenuOpen && printPrettyDisplay && (millis() > (prettyCounter + 3000))) {
        prettyCounter = millis();
        if (whichDisplay == 0) bms.printPackSummary();
        if (whichDisplay == 1) bms.printPackDetails();
    }
}

// Shared helper — prints the pretty-display pause warning on every menu
void Menu::printPrettyWarning() {
    if (printPrettyDisplay) {
        SERIALCONSOLE.println("\nNote: Pack Summary/Details is PAUSED while menu is open.");
        SERIALCONSOLE.println("      It will resume automatically after you exit (X).");
    }
}

// Perform a clean X-exit from anywhere and inform the user
void Menu::exitMenu() {
    SERIALCONSOLE.println("Exiting menu. Type m, menu, help, or ? to open the menu again.");
    if (printPrettyDisplay) {
        SERIALCONSOLE.println("Pack Summary/Details output is now ACTIVE.");
    }
    isMenuOpen = false;
    currentState = ROOT_MENU;
    pendingEdit = NO_EDIT;
    ptrBuffer = 0;
}

void Menu::handleInput(char c) {
    if (c == '\n' || c == '\r') {
        cmdBuffer[ptrBuffer] = 0; // null terminate

        // === X / x — exit the menu completely from anywhere ===
        bool isX = (ptrBuffer == 1 && (cmdBuffer[0] == 'x' || cmdBuffer[0] == 'X'));
        if (isX) {
            exitMenu();
            ptrBuffer = 0;
            return;
        }

        // === COMMANDS THAT RETURN TO ROOT MENU FROM ANYWHERE ===
        if (currentState != WAITING_FOR_INPUT) {
            bool isM = (ptrBuffer == 1 && (cmdBuffer[0] == 'm' || cmdBuffer[0] == 'M'));
            bool isMenu = (ptrBuffer == 4 &&
                (cmdBuffer[0] == 'm' || cmdBuffer[0] == 'M') &&
                (cmdBuffer[1] == 'e' || cmdBuffer[1] == 'E') &&
                (cmdBuffer[2] == 'n' || cmdBuffer[2] == 'N') &&
                (cmdBuffer[3] == 'u' || cmdBuffer[3] == 'U'));
            bool isHelp = (ptrBuffer == 4 &&
                (cmdBuffer[0] == 'h' || cmdBuffer[0] == 'H') &&
                (cmdBuffer[1] == 'e' || cmdBuffer[1] == 'E') &&
                (cmdBuffer[2] == 'l' || cmdBuffer[2] == 'L') &&
                (cmdBuffer[3] == 'p' || cmdBuffer[3] == 'P'));
            bool isQMark = (ptrBuffer == 1 && cmdBuffer[0] == '?');

            if (isM || isMenu || isHelp || isQMark) {
                currentState = ROOT_MENU;
                isMenuOpen = true;
                printRootMenu();
                ptrBuffer = 0;
                return;
            }
        }
        
		if (currentState == WAITING_FOR_INPUT) { 
            handleWaitingForInput();
        }
        else if (ptrBuffer == 1) {
            char cmd = cmdBuffer[0];
            switch (currentState) {
            case ROOT_MENU:              handleRootCommand(cmd);              break;
            case CONFIG_MENU:            handleConfigCommand(cmd);            break;
            case MODULE_MENU:            handleModuleCommand(cmd);            break;
            case LOGGING_MENU:           handleLoggingCommand(cmd);           break;
            case DEFAULTS_MENU:          handleDefaultsCommand(cmd);          break;
            case CONTACTOR_CURRENT_MENU: handleContactorCurrentCommand(cmd);  break;
            case FAULT_LOG_MENU:         handleFaultLogCommand(cmd);          break;
            case BATTERY_CONFIG_MENU:    handleBatteryConfigCommand(cmd);     break;
            default: break;
            }
        }

        ptrBuffer = 0;
        return;
    }

    // Normal character — store in buffer
    cmdBuffer[ptrBuffer++] = (unsigned char)c;
    if (ptrBuffer > 79) ptrBuffer = 79;
}

// ======================================================
// WAITING FOR INPUT — thin dispatcher only, no logic here  Tracks which specific setting we're waiting for, and routes to the appropriate handler when the user hits Enter
// ======================================================

void Menu::handleWaitingForInput() {
    switch (pendingEdit) {
    case EDIT_OVER_VOLTAGE:
    case EDIT_UNDER_VOLTAGE:
    case EDIT_OVER_TEMP:
    case EDIT_UNDER_TEMP:
    case EDIT_BALANCE_VOLTAGE:
    case EDIT_BALANCE_HYST:
    case EDIT_OVERCURRENT_THRESHOLD_A:
    case EDIT_CELL_FAULT_DEBOUNCE:
        handleConfigWaitingInput();
        break;

    case EDIT_PRECHARGE_TIMEOUT_MS:
    case EDIT_CURRENT_SENSOR_VBIAS:
    case EDIT_CURRENT_SENSOR_VRANGE:
    case EDIT_CURRENT_SENSOR_RATED_AMPS:
        handleContactorCurrentWaitingInput();
        break;

    case EDIT_PARALLEL_STRINGS:
    case EDIT_STORAGE_WAKE_INTERVAL_HOURS:
    case EDIT_STORAGE_BALANCE_DURATION_MINS:
        handleBatteryConfigWaitingInput();
        break;

   

    default:
        SERIALCONSOLE.println("Unknown pending edit.");
        break;
    }
}

// ======================================================
// ROOT MENU
// ======================================================

void Menu::printRootMenu() {
    SERIALCONSOLE.println("\n=== Tesla BMS Root Menu ===");
    SERIALCONSOLE.println("1. BMS Saftey Thresholds (Overvolt, Undervolt, Ect....");
    SERIALCONSOLE.println("2. BMS Module Ops (sleep/wake, scan, renumber, balance)");
    SERIALCONSOLE.println("3. BMS Logging (set Debug/Info/Warn/Error/Off)");
    SERIALCONSOLE.println("4. BMS Defaults (factory reset)");
    SERIALCONSOLE.printf("5. Toggle Pack Output to console every 3 seconds. (%s)\n", printPrettyDisplay ? "ON" : "OFF");
    SERIALCONSOLE.printf("6. Toggle Display Mode (summarized or detailed output for option 5) (%s)\n", whichDisplay == 0 ? "Summary" : "Details");
    SERIALCONSOLE.println("7. Contactor & Current Sensor Settings");
    SERIALCONSOLE.println("8. Fault Log (view only)");
    SERIALCONSOLE.println("9. Battery Configuration & Status");
    SERIALCONSOLE.println("X. Exit Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("\n====================");
}

void Menu::handleRootCommand(char c) {
    switch (c) {
    case '1': currentState = CONFIG_MENU;            printConfigMenu();            break;
    case '2': currentState = MODULE_MENU;            printModuleMenu();            break;
    case '3': currentState = LOGGING_MENU;           printLoggingMenu();           break;
    case '4': currentState = DEFAULTS_MENU;          printDefaultsMenu();          break;
    case '5': printPrettyDisplay = !printPrettyDisplay; printPrettyDisplay ? SERIALCONSOLE.printf("Pack Display is now ENABLED (%s)\n", whichDisplay == 0 ? "Pack Summary" : "Pack Details") : SERIALCONSOLE.println("Pack Display is now DISABLED"); break;
    case '6': whichDisplay = (whichDisplay == 0) ? 1 : 0; SERIALCONSOLE.printf("Display mode is now %s\n", whichDisplay == 0 ? "Pack Summary" : "Pack Details"); break;
    case '7': currentState = CONTACTOR_CURRENT_MENU; printContactorCurrentMenu();  break;
    case '8': currentState = FAULT_LOG_MENU;         printFaultLogMenu();          break;
    case '9': currentState = BATTERY_CONFIG_MENU;    printBatteryConfigMenu();     break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// CONFIG MENU  (Safety Thresholds)
// ======================================================

void Menu::printConfigMenu() {
    SERIALCONSOLE.println("\n=== Configuration Settings (Safety Thresholds) ===");
    SERIALCONSOLE.printf("1. High Voltage Limit          [%.3f V]\n", eepromdata.OverVSetpoint);
    SERIALCONSOLE.printf("2. Low Voltage Limit           [%.3f V]\n", eepromdata.UnderVSetpoint);
    SERIALCONSOLE.printf("3. High Temp Limit             [%.1f C]\n", eepromdata.OverTSetpoint);
    SERIALCONSOLE.printf("4. Low Temp Limit              [%.1f C]\n", eepromdata.UnderTSetpoint);
    SERIALCONSOLE.printf("5. Balance Voltage             [%.3f V]\n", eepromdata.balanceVoltage);
    SERIALCONSOLE.printf("6. Balance Hysteresis          [%.3f V]\n", eepromdata.balanceHyst);
    SERIALCONSOLE.printf("7. Over-current Trip Threshold [%.1f A]\n", eepromdata.OVERCURRENT_THRESHOLD_A);   // Short-circuit/over-current protection threshold in Amps
    SERIALCONSOLE.printf("8. Cell Fault Debounce (how many bad readings before faulting)        [%d readings]\n", eepromdata.CELL_FAULT_DEBOUNCE);  // Consecutive fault readings required before a fault is declared (prevents noise/chatter)
    SERIALCONSOLE.println("R. Reset Safety Thresholds to defaults");
    SERIALCONSOLE.println("0. Back to Main Menu");
	SERIALCONSOLE.println("WARNING: Changing these settings can cause damage to your battery and/or create a safety hazard if set incorrectly.");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleConfigCommand(char c) {
    switch (c) {
    case '1': SERIALCONSOLE.printf("Current: %.3f  Enter new High Voltage Limit (0.0-6.0 V, blank to keep):\n", eepromdata.OverVSetpoint);             pendingEdit = EDIT_OVER_VOLTAGE;            currentState = WAITING_FOR_INPUT; break;
    case '2': SERIALCONSOLE.printf("Current: %.3f  Enter new Low Voltage Limit (0.0-6.0 V, blank to keep):\n", eepromdata.UnderVSetpoint);             pendingEdit = EDIT_UNDER_VOLTAGE;           currentState = WAITING_FOR_INPUT; break;
    case '3': SERIALCONSOLE.printf("Current: %.1f  Enter new High Temp Limit (0.0-100.0 C, blank to keep):\n", eepromdata.OverTSetpoint);              pendingEdit = EDIT_OVER_TEMP;               currentState = WAITING_FOR_INPUT; break;
    case '4': SERIALCONSOLE.printf("Current: %.1f  Enter new Low Temp Limit (-20.0-120.0 C, blank to keep):\n", eepromdata.UnderTSetpoint);            pendingEdit = EDIT_UNDER_TEMP;              currentState = WAITING_FOR_INPUT; break;
    case '5': SERIALCONSOLE.printf("Current: %.3f  Enter new Balance Voltage (0.0-6.0 V, blank to keep):\n", eepromdata.balanceVoltage);              pendingEdit = EDIT_BALANCE_VOLTAGE;         currentState = WAITING_FOR_INPUT; break;
    case '6': SERIALCONSOLE.printf("Current: %.3f  Enter new Balance Hysteresis (0.0-1.0 V, blank to keep):\n", eepromdata.balanceHyst);              pendingEdit = EDIT_BALANCE_HYST;            currentState = WAITING_FOR_INPUT; break;
    case '7': SERIALCONSOLE.printf("Current: %.1f  Enter new Over-current Threshold (0.0-1000.0 A, blank to keep):\n", eepromdata.OVERCURRENT_THRESHOLD_A); pendingEdit = EDIT_OVERCURRENT_THRESHOLD_A; currentState = WAITING_FOR_INPUT; break;
    case '8': SERIALCONSOLE.printf("Current: %d    Enter new Cell Fault Debounce (1-20 readings, blank to keep):\n", eepromdata.CELL_FAULT_DEBOUNCE);  pendingEdit = EDIT_CELL_FAULT_DEBOUNCE;     currentState = WAITING_FOR_INPUT; break;
    case 'r':
    case 'R':
        SERIALCONSOLE.println("Resetting Safety Thresholds to defaults...");
        EEPROMSettings::resetSafetyThresholds();
        SERIALCONSOLE.println("Done.");
        printConfigMenu();
        break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

void Menu::handleConfigWaitingInput() {
    if (ptrBuffer == 0) {
        SERIALCONSOLE.println("Value unchanged.");
        returnToConfigMenu();
        return;
    }

    char* endPtr = NULL;
    float newVal = strtof((char*)cmdBuffer, &endPtr);
    if (endPtr == (char*)cmdBuffer) {
        SERIALCONSOLE.println("Invalid input. Please enter a numeric value.");
        returnToConfigMenu();
        return;
    }

    switch (pendingEdit) {
    case EDIT_OVER_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) { eepromdata.OverVSetpoint = newVal;  EEPROMSettings::save(); SERIALCONSOLE.printf("High Voltage Limit set to: %.3f V\n", eepromdata.OverVSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 6.0 V"); }
        break;
    case EDIT_UNDER_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) { eepromdata.UnderVSetpoint = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Low Voltage Limit set to: %.3f V\n", eepromdata.UnderVSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 6.0 V"); }
        break;
    case EDIT_OVER_TEMP:
        if (newVal >= 0.0f && newVal <= 100.0f) { eepromdata.OverTSetpoint = newVal;  EEPROMSettings::save(); SERIALCONSOLE.printf("High Temp Limit set to: %.1f C\n", eepromdata.OverTSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 100.0 C"); }
        break;
    case EDIT_UNDER_TEMP:
        if (newVal >= -20.0f && newVal <= 120.0f) { eepromdata.UnderTSetpoint = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Low Temp Limit set to: %.1f C\n", eepromdata.UnderTSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: -20.0 - 120.0 C"); }
        break;
    case EDIT_BALANCE_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) { eepromdata.balanceVoltage = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Balance Voltage set to: %.3f V\n", eepromdata.balanceVoltage); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 6.0 V"); }
        break;
    case EDIT_BALANCE_HYST:
        if (newVal >= 0.0f && newVal <= 1.0f) { eepromdata.balanceHyst = newVal;    EEPROMSettings::save(); SERIALCONSOLE.printf("Balance Hysteresis set to: %.3f V\n", eepromdata.balanceHyst); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 1.0 V"); }
        break;
    case EDIT_OVERCURRENT_THRESHOLD_A:
        if (newVal >= 0.0f && newVal <= 1000.0f) { eepromdata.OVERCURRENT_THRESHOLD_A = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Over-current Threshold set to: %.1f A\n", eepromdata.OVERCURRENT_THRESHOLD_A); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 1000.0 A"); }
        break;
    case EDIT_CELL_FAULT_DEBOUNCE: {
        uint8_t newInt = (uint8_t)newVal;
        if (newInt >= 1 && newInt <= 20) { eepromdata.CELL_FAULT_DEBOUNCE = newInt; EEPROMSettings::save(); SERIALCONSOLE.printf("Cell Fault Debounce set to: %d readings\n", eepromdata.CELL_FAULT_DEBOUNCE); }
        else { SERIALCONSOLE.println("Invalid value. Range: 1 - 20"); }
        break;
    }
    default: break;
    }

    returnToConfigMenu();
}

void Menu::returnToConfigMenu() {
    pendingEdit = NO_EDIT;
    currentState = CONFIG_MENU;
    printConfigMenu();
}

// ======================================================
// MODULE MENU
// ======================================================

void Menu::printModuleMenu() {
    SERIALCONSOLE.println("\n=== Module Operations ===");
    SERIALCONSOLE.println("1. Sleep Boards");
    SERIALCONSOLE.println("2. Wake Boards");
    SERIALCONSOLE.println("3. Find Boards");
    SERIALCONSOLE.println("4. Renumber Boards");
    SERIALCONSOLE.println("5. Clear Faults");
    SERIALCONSOLE.println("6. Balance Cells");
    SERIALCONSOLE.println("0. Back to Main Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleModuleCommand(char c) {
    switch (c) {
    case '1': SERIALCONSOLE.println("Sleeping all connected boards");  bms.sleepBoards();      break;
    case '2': SERIALCONSOLE.println("Waking up all connected boards"); bms.wakeBoards();       break;
    case '3': SERIALCONSOLE.println("Finding all connected Modules"); bms.findBoards();        break;
    case '4': SERIALCONSOLE.println("Renumbering all boards");         bms.renumberBoardIDs(); break;
    case '5': SERIALCONSOLE.println("Clearing all faults");            bms.clearFaults();      break;
    case '6': SERIALCONSOLE.println("Balancing Cells"); bms.balanceCells();                    break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// LOGGING MENU
// ======================================================

void Menu::printLoggingMenu() {
    SERIALCONSOLE.println("\n=== Logging & Debug ===");
    SERIALCONSOLE.println("1. Set Debug");
    SERIALCONSOLE.println("2. Set Info");
    SERIALCONSOLE.println("3. Set Warning");
    SERIALCONSOLE.println("4. Set Error");
    SERIALCONSOLE.println("5. Set Off");
    SERIALCONSOLE.println("0. Back to Main Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleLoggingCommand(char c) {
    switch (c) {
    case '1': Logger::setLoglevel(Logger::Debug); eepromdata.logLevel = 0; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Debug");   break;
    case '2': Logger::setLoglevel(Logger::Info);  eepromdata.logLevel = 1; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Info");    break;
    case '3': Logger::setLoglevel(Logger::Warn);  eepromdata.logLevel = 2; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Warning"); break;
    case '4': Logger::setLoglevel(Logger::Error); eepromdata.logLevel = 3; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Error");   break;
    case '5': Logger::setLoglevel(Logger::Off);   eepromdata.logLevel = 4; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Off");     break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// DEFAULTS MENU
// ======================================================

void Menu::printDefaultsMenu() {
    SERIALCONSOLE.println("\n=== Defaults & Reset ===");
    SERIALCONSOLE.println("1. Reset ALL settings to factory defaults");
    SERIALCONSOLE.println("0. Back to Main Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleDefaultsCommand(char c) {
    switch (c) {
    case '1':
        SERIALCONSOLE.println("Resetting all settings to factory defaults...");
        EEPROMSettings::loadDefaults();
        SERIALCONSOLE.println("Done. Settings restored to defaults.");
        break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// CONTACTOR & CURRENT SENSOR MENU  
// ======================================================

void Menu::printContactorCurrentMenu() {
    SERIALCONSOLE.println("\n=== Contactor & Current Sensor Settings  ===");
    SERIALCONSOLE.printf("1. Pre-charge Circuit Present          [%s]\n", eepromdata.prechargeEnabled ? "YES" : "NO");
    SERIALCONSOLE.printf("2. Current Sensor Present      [%s]\n", eepromdata.currentSensorPresent ? "YES" : "NO");
    SERIALCONSOLE.printf("3. Pre-charge Timeout          [%d ms]\n", eepromdata.prechargeTimeoutMs);
    SERIALCONSOLE.printf("4. Current Sensor Bias Voltage [%.3f V]\n", eepromdata.currentSensorVbias);        // Mid-point bias voltage for the current sensor (e.g. 2.5V for QN-C15S)
    SERIALCONSOLE.printf("5. Current Sensor Volt Range   [%.3f V]\n", eepromdata.currentSensorVrange);       // Full-scale voltage swing around bias representing rated current (e.g. 0.625V for QN-C15S)
    SERIALCONSOLE.printf("6. Current Sensor Rated Amps   [%d A]\n", eepromdata.currentSensorRatedAmps);    // Maximum measurable current of the sensor (e.g. 500A for QN-C15S)
    SERIALCONSOLE.println("R. Reset Contactor & Current Sensor settings to defaults");
    SERIALCONSOLE.println("0. Back to Main Menu");
	SERIALCONSOLE.println("WARNING: DO NOT ENSURE YOUR CURRENT SENSOR DOES NOT EXCEED ESP32 RATED INPUT VOLTAGE OR YOU RISK DAMAGING YOUR BOARD.");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleContactorCurrentCommand(char c) {
    switch (c) {
    case '1':
        eepromdata.prechargeEnabled = !eepromdata.prechargeEnabled;
        EEPROMSettings::save();
        SERIALCONSOLE.printf("Pre-charge Enabled set to: %s\n", eepromdata.prechargeEnabled ? "YES" : "NO");
        printContactorCurrentMenu();
        break;
    case '2':
        eepromdata.currentSensorPresent = !eepromdata.currentSensorPresent;
        EEPROMSettings::save();
        SERIALCONSOLE.printf("Current Sensor Present set to: %s\n", eepromdata.currentSensorPresent ? "YES" : "NO");
        printContactorCurrentMenu();
        break;
    case '3':
        SERIALCONSOLE.printf("Current: %d  Enter new Pre-charge Timeout (1000-15000 ms, blank to keep):\n", eepromdata.prechargeTimeoutMs);
        pendingEdit = EDIT_PRECHARGE_TIMEOUT_MS;
        currentState = WAITING_FOR_INPUT;
        break;
    case '4':
        SERIALCONSOLE.printf("Current: %.3f  Enter new Current Sensor Bias Voltage (0.0-5.0 V, blank to keep):\n", eepromdata.currentSensorVbias);
        pendingEdit = EDIT_CURRENT_SENSOR_VBIAS;
        currentState = WAITING_FOR_INPUT;
        break;
    case '5':
        SERIALCONSOLE.printf("Current: %.3f  Enter new Current Sensor Volt Range (0.0-5.0 V, blank to keep):\n", eepromdata.currentSensorVrange);
        pendingEdit = EDIT_CURRENT_SENSOR_VRANGE;
        currentState = WAITING_FOR_INPUT;
        break;
    case '6':
        SERIALCONSOLE.printf("Current: %d  Enter new Current Sensor Rated Amps (1-2000 A, blank to keep):\n", eepromdata.currentSensorRatedAmps);
        pendingEdit = EDIT_CURRENT_SENSOR_RATED_AMPS;
        currentState = WAITING_FOR_INPUT;
        break;
    case 'r':
    case 'R':
        SERIALCONSOLE.println("Resetting Contactor & Current Sensor settings to defaults...");
        EEPROMSettings::resetAdditionalHardware();
        SERIALCONSOLE.println("Done.");
        printContactorCurrentMenu();
        break;
    case '0':
        currentState = ROOT_MENU;
        printRootMenu();
        break;
    default:
        SERIALCONSOLE.println("Unknown option");
        break;
    }
}

void Menu::handleContactorCurrentWaitingInput() {
    if (ptrBuffer == 0) {
        SERIALCONSOLE.println("Value unchanged.");
        returnToContactorCurrentMenu();
        return;
    }

    switch (pendingEdit) {
    case EDIT_PRECHARGE_TIMEOUT_MS: {
        uint32_t newVal = strtoul((char*)cmdBuffer, NULL, 10);
        if (newVal >= 1000 && newVal <= 15000) {
            eepromdata.prechargeTimeoutMs = newVal;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Pre-charge Timeout set to: %d ms\n", eepromdata.prechargeTimeoutMs);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Range: 1000 - 15000 ms");
        }
        break;
    }
    case EDIT_CURRENT_SENSOR_VBIAS: {
        char* endPtr = NULL;
        float newVal = strtof((char*)cmdBuffer, &endPtr);
        if (endPtr != (char*)cmdBuffer && newVal >= 0.0f && newVal <= 5.0f) {
            eepromdata.currentSensorVbias = newVal;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Current Sensor Bias Voltage set to: %.3f V\n", eepromdata.currentSensorVbias);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Range: 0.0 - 5.0 V");
        }
        break;
    }
    case EDIT_CURRENT_SENSOR_VRANGE: {
        char* endPtr = NULL;
        float newVal = strtof((char*)cmdBuffer, &endPtr);
        if (endPtr != (char*)cmdBuffer && newVal >= 0.0f && newVal <= 5.0f) {
            eepromdata.currentSensorVrange = newVal;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Current Sensor Volt Range set to: %.3f V\n", eepromdata.currentSensorVrange);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Range: 0.0 - 5.0 V");
        }
        break;
    }
    case EDIT_CURRENT_SENSOR_RATED_AMPS: {
        int newVal = (int)strtol((char*)cmdBuffer, NULL, 10);
        if (newVal >= 1 && newVal <= 2000) {
            eepromdata.currentSensorRatedAmps = newVal;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Current Sensor Rated Amps set to: %d A\n", eepromdata.currentSensorRatedAmps);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Range: 1 - 2000 A");
        }
        break;
    }
    default: break;
    }

    returnToContactorCurrentMenu();
}

void Menu::returnToContactorCurrentMenu() {
    pendingEdit = NO_EDIT;
    currentState = CONTACTOR_CURRENT_MENU;
    printContactorCurrentMenu();
}

// ======================================================
// FAULT LOG MENU  (display only — no reset option)
// ======================================================

void Menu::printFaultLogMenu() {
    SERIALCONSOLE.println("\n=== Fault Log (last 5) (View Only) ===");

    // Map FaultEntry::Type enum to a readable string
    auto faultTypeName = [](FaultEntry::Type t) -> const char* {
        switch (t) {
        case FaultEntry::Type::None:            return "None";
        case FaultEntry::Type::OverVoltage:     return "OverVoltage";
        case FaultEntry::Type::UnderVoltage:    return "UnderVoltage";
        case FaultEntry::Type::OverTemperature: return "OverTemperature";
        case FaultEntry::Type::UnderTemperature:return "UnderTemperature";
        case FaultEntry::Type::OverCurrent:     return "OverCurrent";
        default:                                return "Unknown";
        }
        };

    bool anyFault = false;
    // faultLog[0] is oldest, faultLog[4] is newest — display oldest at top
    for (int i = 0; i < 5; i++) {
        const FaultEntry& f = eepromdata.faultLog[i];
        if (f.type != FaultEntry::Type::None) {
            anyFault = true;
            SERIALCONSOLE.printf("[%d] Type: %-16s  Module: %d  Cell: %d  Value: %.3f\n",
                i + 1, faultTypeName(f.type), f.module, f.cell, f.value);
            SERIALCONSOLE.printf("    Timestamp: %lu ms   Cleared: %lu ms\n",
                (unsigned long)f.timestamp, (unsigned long)f.clearedTimestamp);
        }
    }
    if (!anyFault) {
        SERIALCONSOLE.println("No faults logged.");
    }

    SERIALCONSOLE.println("0. Back to Main Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleFaultLogCommand(char c) {
    switch (c) {
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// BATTERY CONFIG & STATUS MENU
// ======================================================

void Menu::printBatteryConfigMenu() {
    SERIALCONSOLE.println("\n=== Battery Config & Status ===");
    SERIALCONSOLE.printf("1. Parallel Strings            [%d]\n", eepromdata.parallelStrings);                                       // Number of parallel battery strings; must be >= 1 to prevent division-by-zero
    SERIALCONSOLE.printf("2. Storage Wake Interval       [%lu hr]\n", (unsigned long)(eepromdata.STORAGE_WAKE_INTERVAL_MS / 3600000UL));    // How often to wake from storage mode for a balance cycle (stored as ms, shown in hours)
    SERIALCONSOLE.printf("3. Storage Balance Duration    [%lu min]\n", (unsigned long)(eepromdata.STORAGE_BALANCE_DURATION_MS / 60000UL));   // How long to stay awake and balance during each storage wake cycle (stored as ms, shown in minutes)
    SERIALCONSOLE.printf("   State of Charge             [%.1f %%]  (display only)\n", eepromdata.socPercent);                       // Current estimated state of charge — maintained by coulomb counting, read-only here
    SERIALCONSOLE.printf("   Coulomb Count               [%.3f Ah] (display only)\n", eepromdata.coulombCountAh);                    // Net amp-hours accumulated since last reset — read-only here
    SERIALCONSOLE.println("R. Reset Battery Config to defaults");
    SERIALCONSOLE.println("0. Back to Main Menu");
    printPrettyWarning();
    SERIALCONSOLE.println("====================");
}

void Menu::handleBatteryConfigCommand(char c) {
    switch (c) {
    case '1':
        SERIALCONSOLE.printf("Current: %d  Enter new Parallel Strings (1-64, blank to keep):\n", eepromdata.parallelStrings);
        pendingEdit = EDIT_PARALLEL_STRINGS;
        currentState = WAITING_FOR_INPUT;
        break;
    case '2':
        SERIALCONSOLE.printf("Current: %lu hr  Enter new Storage Wake Interval (1 or more hours, blank to keep):\n",
            (unsigned long)(eepromdata.STORAGE_WAKE_INTERVAL_MS / 3600000UL));
        pendingEdit = EDIT_STORAGE_WAKE_INTERVAL_HOURS;
        currentState = WAITING_FOR_INPUT;
        break;
    case '3':
        SERIALCONSOLE.printf("Current: %lu min  Enter new Storage Balance Duration (1 or more minutes, blank to keep):\n",
            (unsigned long)(eepromdata.STORAGE_BALANCE_DURATION_MS / 60000UL));
        pendingEdit = EDIT_STORAGE_BALANCE_DURATION_MINS;
        currentState = WAITING_FOR_INPUT;
        break;
    case 'r':
    case 'R':
        SERIALCONSOLE.println("Resetting Battery Config to defaults...");
        EEPROMSettings::resetBatteryConfig();
        SERIALCONSOLE.println("Done.");
        printBatteryConfigMenu();
        break;
    case '0':
        currentState = ROOT_MENU;
        printRootMenu();
        break;
    default:
        SERIALCONSOLE.println("Unknown option");
        break;
    }
}

void Menu::handleBatteryConfigWaitingInput() {
    if (ptrBuffer == 0) {
        SERIALCONSOLE.println("Value unchanged.");
        returnToBatteryConfigMenu();
        return;
    }

    switch (pendingEdit) {
    case EDIT_PARALLEL_STRINGS: {
        uint8_t newVal = (uint8_t)strtoul((char*)cmdBuffer, NULL, 10);
        if (newVal >= 1 && newVal <= 64) {
            eepromdata.parallelStrings = newVal;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Parallel Strings set to: %d\n", eepromdata.parallelStrings);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Range: 1 - 64 (must be 1 or greater)");
        }
        break;
    }
    case EDIT_STORAGE_WAKE_INTERVAL_HOURS: {
        // User enters hours; stored internally as milliseconds
        uint32_t hours = strtoul((char*)cmdBuffer, NULL, 10);
        if (hours >= 1) {
            eepromdata.STORAGE_WAKE_INTERVAL_MS = hours * 3600000UL;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Storage Wake Interval set to: %lu hr (%lu ms)\n",
                (unsigned long)hours, (unsigned long)eepromdata.STORAGE_WAKE_INTERVAL_MS);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Must be 1 or greater (hours).");
        }
        break;
    }
    case EDIT_STORAGE_BALANCE_DURATION_MINS: {
        // User enters minutes; stored internally as milliseconds
        uint32_t mins = strtoul((char*)cmdBuffer, NULL, 10);
        if (mins >= 1) {
            eepromdata.STORAGE_BALANCE_DURATION_MS = mins * 60000UL;
            EEPROMSettings::save();
            SERIALCONSOLE.printf("Storage Balance Duration set to: %lu min (%lu ms)\n",
                (unsigned long)mins, (unsigned long)eepromdata.STORAGE_BALANCE_DURATION_MS);
        }
        else {
            SERIALCONSOLE.println("Invalid value. Must be 1 or greater (minutes).");
        }
        break;
    }
    default: break;
    }

    returnToBatteryConfigMenu();
}

void Menu::returnToBatteryConfigMenu() {
    pendingEdit = NO_EDIT;
    currentState = BATTERY_CONFIG_MENU;
    printBatteryConfigMenu();
}