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

#include "Menu.h"
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

void Menu::handleInput(char c) {
    if (c == '\n' || c == '\r') {
        cmdBuffer[ptrBuffer] = 0; // null terminate

        // === MULTIPLE COMMANDS THAT RETURN TO ROOT MENU ===
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
            // Single character menu command
            char cmd = cmdBuffer[0];
            switch (currentState) {
            case ROOT_MENU:     handleRootCommand(cmd);     break;
            case CONFIG_MENU:   handleConfigCommand(cmd);   break;
            case MODULE_MENU:   handleModuleCommand(cmd);   break;
            case LOGGING_MENU:  handleLoggingCommand(cmd);  break;
            case DEFAULTS_MENU: handleDefaultsCommand(cmd); break;
            default: break;
            }
        }
        // else: multi-char input outside WAITING_FOR_INPUT is not handled

        ptrBuffer = 0;
        return;
    }

    // Normal character input - store in buffer
    cmdBuffer[ptrBuffer++] = (unsigned char)c;
    if (ptrBuffer > 79) ptrBuffer = 79;
}

// ======================================================
// ROOT MENU
// ======================================================

void Menu::printRootMenu() {
    SERIALCONSOLE.println("\n=== Tesla BMS Root Menu ===");
    SERIALCONSOLE.println("1. BMS Config Settings (OvrV/UndrV/OvrT/UndrT + Bal thresholds)");
    SERIALCONSOLE.println("2. BMS Module Ops (sleep/wake, scan, renumber, balance)");
    SERIALCONSOLE.println("3. BMS Logging (set Debug/Info/Warn/Error/Off)");
    SERIALCONSOLE.println("4. BMS Defaults (factory reset)");
    SERIALCONSOLE.printf("5. Toggle Pack Output to console every 3 seconds. (%s)\n", printPrettyDisplay ? "ON" : "OFF");
    SERIALCONSOLE.printf("6. Toggle Display Mode (summarized or detailed output for option 5) (%s)\n", whichDisplay == 0 ? "Summary" : "Details");
    SERIALCONSOLE.println("0. Exit Menu");
    if (printPrettyDisplay) {
        SERIALCONSOLE.println("\nNote: Pack Summary/Details is PAUSED while menu is open.");
        SERIALCONSOLE.println("      It will resume automatically after you exit the menu.");
    }
    SERIALCONSOLE.println("\n====================");
}

void Menu::handleRootCommand(char c) { // Root Menu commands
    switch (c) {
    case '1': currentState = CONFIG_MENU;    printConfigMenu();    break;
    case '2': currentState = MODULE_MENU;    printModuleMenu();    break;
    case '3': currentState = LOGGING_MENU;   printLoggingMenu();   break;
    case '4': currentState = DEFAULTS_MENU;  printDefaultsMenu();  break;
    case '5': printPrettyDisplay = !printPrettyDisplay; printPrettyDisplay ? SERIALCONSOLE.printf("Pack Display is now ENABLED (%s)\n", whichDisplay == 0 ? "Pack Summary" : "Pack Details") : SERIALCONSOLE.println("Pack Display is now DISABLED"); break;
    case '6': whichDisplay = (whichDisplay == 0) ? 1 : 0; SERIALCONSOLE.printf("Display mode is now %s\n", whichDisplay == 0 ? "Pack Summary" : "Pack Details"); break;
    case '0': SERIALCONSOLE.println("Exiting menu. Type m, menu, help, or ? to open the menu again."); isMenuOpen = false; break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

// ======================================================
// CONFIG MENU
// ======================================================

void Menu::printConfigMenu() {
    SERIALCONSOLE.println("\n=== Configuration Settings ===");
    SERIALCONSOLE.printf("1. High Voltage Limit     [%.3f V]\n", eepromdata.OverVSetpoint);
    SERIALCONSOLE.printf("2. Low Voltage Limit      [%.3f V]\n", eepromdata.UnderVSetpoint);
    SERIALCONSOLE.printf("3. High Temp Limit        [%.1f C]\n", eepromdata.OverTSetpoint);
    SERIALCONSOLE.printf("4. Low Temp Limit         [%.1f C]\n", eepromdata.UnderTSetpoint);
    SERIALCONSOLE.printf("5. Balance Voltage        [%.3f V]\n", eepromdata.balanceVoltage);
    SERIALCONSOLE.printf("6. Balance Hysteresis     [%.3f V]\n", eepromdata.balanceHyst);
    SERIALCONSOLE.println("0. Back to Main Menu");
}

void Menu::handleConfigCommand(char c) {
    switch (c) {
    case '1': SERIALCONSOLE.printf("Current: %.3f  Enter new High Voltage Limit (0.0-6.0 V, blank to keep):\n", eepromdata.OverVSetpoint); pendingEdit = EDIT_OVER_VOLTAGE; currentState = WAITING_FOR_INPUT; break;
    case '2': SERIALCONSOLE.printf("Current: %.3f  Enter new Low Voltage Limit (0.0-6.0 V, blank to keep):\n", eepromdata.UnderVSetpoint); pendingEdit = EDIT_UNDER_VOLTAGE; currentState = WAITING_FOR_INPUT; break;
    case '3': SERIALCONSOLE.printf("Current: %.1f  Enter new High Temp Limit (0.0-100.0 C, blank to keep):\n", eepromdata.OverTSetpoint); pendingEdit = EDIT_OVER_TEMP; currentState = WAITING_FOR_INPUT; break;
    case '4': SERIALCONSOLE.printf("Current: %.1f  Enter new Low Temp Limit (-20.0-120.0 C, blank to keep):\n", eepromdata.UnderTSetpoint); pendingEdit = EDIT_UNDER_TEMP; currentState = WAITING_FOR_INPUT; break;
    case '5': SERIALCONSOLE.printf("Current: %.3f  Enter new Balance Voltage (0.0-6.0 V, blank to keep):\n", eepromdata.balanceVoltage); pendingEdit = EDIT_BALANCE_VOLTAGE; currentState = WAITING_FOR_INPUT; break;
    case '6': SERIALCONSOLE.printf("Current: %.3f  Enter new Balance Hysteresis (0.0-1.0 V, blank to keep):\n", eepromdata.balanceHyst); pendingEdit = EDIT_BALANCE_HYST; currentState = WAITING_FOR_INPUT; break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  SERIALCONSOLE.println("Unknown option"); break;
    }
}

void Menu::handleWaitingForInput() {
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
        if (newVal >= 0.0f && newVal <= 6.0f) { eepromdata.OverVSetpoint = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("High Voltage Limit set to: %.3f V\n", eepromdata.OverVSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 6.0 V"); }
        break;
    case EDIT_UNDER_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) { eepromdata.UnderVSetpoint = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Low Voltage Limit set to: %.3f V\n", eepromdata.UnderVSetpoint); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 6.0 V"); }
        break;
    case EDIT_OVER_TEMP:
        if (newVal >= 0.0f && newVal <= 100.0f) { eepromdata.OverTSetpoint = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("High Temp Limit set to: %.1f C\n", eepromdata.OverTSetpoint); }
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
        if (newVal >= 0.0f && newVal <= 1.0f) { eepromdata.balanceHyst = newVal; EEPROMSettings::save(); SERIALCONSOLE.printf("Balance Hysteresis set to: %.3f V\n", eepromdata.balanceHyst); }
        else { SERIALCONSOLE.println("Invalid value. Range: 0.0 - 1.0 V"); }
        break;
    default:
        SERIALCONSOLE.println("Unknown setting.");
        break;
    }

    returnToConfigMenu();
}

void Menu::returnToConfigMenu() {
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
}

void Menu::handleModuleCommand(char c) {
    switch (c) {
    case '1': SERIALCONSOLE.println("Sleeping all connected boards"); bms.sleepBoards(); break;
    case '2': SERIALCONSOLE.println("Waking up all connected boards"); bms.wakeBoards(); break;
    case '3': bms.findBoards(); break;
    case '4': SERIALCONSOLE.println("Renumbering all boards"); bms.renumberBoardIDs(); break;
    case '5': SERIALCONSOLE.println("Clearing all faults"); bms.clearFaults(); break;
    case '6': bms.balanceCells(); break;
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
}

void Menu::handleLoggingCommand(char c) {
    switch (c) {
    case '1': Logger::setLoglevel(Logger::Debug); eepromdata.logLevel = 0; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Debug"); break;
    case '2': Logger::setLoglevel(Logger::Info);  eepromdata.logLevel = 1; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Info"); break;
    case '3': Logger::setLoglevel(Logger::Warn);  eepromdata.logLevel = 2; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Warning"); break;
    case '4': Logger::setLoglevel(Logger::Error); eepromdata.logLevel = 3; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Error"); break;
    case '5': Logger::setLoglevel(Logger::Off);   eepromdata.logLevel = 4; EEPROMSettings::save(); SERIALCONSOLE.println("Log level set to: Off"); break;
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