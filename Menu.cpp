/*
 * SerialConsole.cpp
 *
 Copyright (c) 2017 EVTV / Collin Kidder

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "Menu.h"
#include "BMSModuleManager.h"
#include "EEPROMSettings.h"
#include "Logger.h"

template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg); return obj; } //Lets us stream SerialUSB


extern BMSModuleManager bms;

Menu::Menu() {
    isMenuOpen = true;
    printPrettyDisplay = false;
    prettyCounter = 0;
    whichDisplay = 0;
    pendingEdit = NO_EDIT;
    ptrBuffer = 0;
}

void Menu::begin() {
    currentState = ROOT_MENU;
    isMenuOpen = true;
    printPrettyDisplay = false;
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

        // 'm', 'M', or "menu" returns to root from any state except WAITING_FOR_INPUT
        if (currentState != WAITING_FOR_INPUT) {
            bool isM    = (ptrBuffer == 1 && (cmdBuffer[0] == 'm' || cmdBuffer[0] == 'M'));
            bool isMenu = (ptrBuffer == 4 &&
                           (cmdBuffer[0] == 'm' || cmdBuffer[0] == 'M') &&
                           (cmdBuffer[1] == 'e' || cmdBuffer[1] == 'E') &&
                           (cmdBuffer[2] == 'n' || cmdBuffer[2] == 'N') &&
                           (cmdBuffer[3] == 'u' || cmdBuffer[3] == 'U'));
            if (isM || isMenu) {
                currentState = ROOT_MENU;
                isMenuOpen = true;
                printRootMenu();
                ptrBuffer = 0;
                return;
            }
        }

        if (currentState == WAITING_FOR_INPUT) {
            handleWaitingForInput();
        } else if (ptrBuffer == 1) {
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

// ====================== MENU PRINTING ======================

void Menu::printRootMenu() {
    Logger::console("\n=== Tesla BMS Root Menu ===");
    Logger::console("1. Configuration Settings");
    Logger::console("2. Module Operations");
    Logger::console("3. Logging & Debug");
    Logger::console("4. Defaults & Reset");
    Logger::console("5. Toggle Pretty Print (%s)", printPrettyDisplay ? "ON" : "OFF");
    Logger::console("6. Toggle Display Mode (%s)", whichDisplay == 0 ? "Summary" : "Details");
    Logger::console("0. Back / Exit");
    Logger::console("Type 'menu' or 'm' from anywhere to return here.");
}

void Menu::printConfigMenu() {
    Logger::console("\n=== Configuration Settings ===");
    Logger::console("1. High Voltage Limit     [%.3f V]", eepromdata.OverVSetpoint);
    Logger::console("2. Low Voltage Limit      [%.3f V]", eepromdata.UnderVSetpoint);
    Logger::console("3. High Temp Limit        [%.1f C]", eepromdata.OverTSetpoint);
    Logger::console("4. Low Temp Limit         [%.1f C]", eepromdata.UnderTSetpoint);
    Logger::console("5. Balance Voltage        [%.3f V]", eepromdata.balanceVoltage);
    Logger::console("6. Balance Hysteresis     [%.3f V]", eepromdata.balanceHyst);
    Logger::console("0. Back to Root");
}

void Menu::printModuleMenu() {
    Logger::console("\n=== Module Operations ===");
    Logger::console("1. Sleep Boards");
    Logger::console("2. Wake Boards");
    Logger::console("3. Find Boards");
    Logger::console("4. Renumber Boards");
    Logger::console("5. Clear Faults");
    Logger::console("6. Balance Cells");
    Logger::console("0. Back to Root");
}

void Menu::printLoggingMenu() {
    Logger::console("\n=== Logging & Debug ===");
    Logger::console("1. Set Debug");
    Logger::console("2. Set Info");
    Logger::console("3. Set Warning");
    Logger::console("4. Set Error");
    Logger::console("5. Set Off");
    Logger::console("0. Back to Root");
}

void Menu::printDefaultsMenu() {
    Logger::console("\n=== Defaults & Reset ===");
    Logger::console("1. Reset ALL settings to factory defaults");
    Logger::console("0. Back to Root");
}

// ====================== COMMAND HANDLERS ======================

void Menu::handleRootCommand(char c) {
    switch (c) {
    case '1': currentState = CONFIG_MENU;    printConfigMenu();    break;
    case '2': currentState = MODULE_MENU;    printModuleMenu();    break;
    case '3': currentState = LOGGING_MENU;   printLoggingMenu();   break;
    case '4': currentState = DEFAULTS_MENU;  printDefaultsMenu();  break;
    case '5':
        printPrettyDisplay = !printPrettyDisplay;
        Logger::console("Pretty Print %s", printPrettyDisplay ? "ENABLED" : "DISABLED");
        break;
    case '6':
        whichDisplay = (whichDisplay == 0) ? 1 : 0;
        Logger::console("Display mode: %s", whichDisplay == 0 ? "Pack Summary" : "Pack Details");
        break;
    case '0':
        Logger::console("Exiting menu.");
        isMenuOpen = false;
        break;
    default:
        Logger::console("Unknown option");
        break;
    }
}

void Menu::handleConfigCommand(char c) {
    switch (c) {
    case '1':
        Logger::console("Current: %.3f  Enter new High Voltage Limit (0.0-6.0 V, blank to keep):", eepromdata.OverVSetpoint);
        pendingEdit = EDIT_OVER_VOLTAGE;
        currentState = WAITING_FOR_INPUT;
        break;
    case '2':
        Logger::console("Current: %.3f  Enter new Low Voltage Limit (0.0-6.0 V, blank to keep):", eepromdata.UnderVSetpoint);
        pendingEdit = EDIT_UNDER_VOLTAGE;
        currentState = WAITING_FOR_INPUT;
        break;
    case '3':
        Logger::console("Current: %.1f  Enter new High Temp Limit (0.0-100.0 C, blank to keep):", eepromdata.OverTSetpoint);
        pendingEdit = EDIT_OVER_TEMP;
        currentState = WAITING_FOR_INPUT;
        break;
    case '4':
        Logger::console("Current: %.1f  Enter new Low Temp Limit (-20.0-120.0 C, blank to keep):", eepromdata.UnderTSetpoint);
        pendingEdit = EDIT_UNDER_TEMP;
        currentState = WAITING_FOR_INPUT;
        break;
    case '5':
        Logger::console("Current: %.3f  Enter new Balance Voltage (0.0-6.0 V, blank to keep):", eepromdata.balanceVoltage);
        pendingEdit = EDIT_BALANCE_VOLTAGE;
        currentState = WAITING_FOR_INPUT;
        break;
    case '6':
        Logger::console("Current: %.3f  Enter new Balance Hysteresis (0.0-1.0 V, blank to keep):", eepromdata.balanceHyst);
        pendingEdit = EDIT_BALANCE_HYST;
        currentState = WAITING_FOR_INPUT;
        break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleWaitingForInput() {
    if (ptrBuffer == 0) {
        Logger::console("Value unchanged.");
        returnToConfigMenu();
        return;
    }

    char* endPtr = NULL;
    float newVal = strtof((char*)cmdBuffer, &endPtr);
    if (endPtr == (char*)cmdBuffer) {
        Logger::console("Invalid input. Please enter a numeric value.");
        returnToConfigMenu();
        return;
    }

    switch (pendingEdit) {
    case EDIT_OVER_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) {
            eepromdata.OverVSetpoint = newVal;
            EEPROMSettings::save();
            Logger::console("High Voltage Limit set to: %.3f V", eepromdata.OverVSetpoint);
        } else {
            Logger::console("Invalid value. Range: 0.0 - 6.0 V");
        }
        break;
    case EDIT_UNDER_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) {
            eepromdata.UnderVSetpoint = newVal;
            EEPROMSettings::save();
            Logger::console("Low Voltage Limit set to: %.3f V", eepromdata.UnderVSetpoint);
        } else {
            Logger::console("Invalid value. Range: 0.0 - 6.0 V");
        }
        break;
    case EDIT_OVER_TEMP:
        if (newVal >= 0.0f && newVal <= 100.0f) {
            eepromdata.OverTSetpoint = newVal;
            EEPROMSettings::save();
            Logger::console("High Temp Limit set to: %.1f C", eepromdata.OverTSetpoint);
        } else {
            Logger::console("Invalid value. Range: 0.0 - 100.0 C");
        }
        break;
    case EDIT_UNDER_TEMP:
        if (newVal >= -20.0f && newVal <= 120.0f) {
            eepromdata.UnderTSetpoint = newVal;
            EEPROMSettings::save();
            Logger::console("Low Temp Limit set to: %.1f C", eepromdata.UnderTSetpoint);
        } else {
            Logger::console("Invalid value. Range: -20.0 - 120.0 C");
        }
        break;
    case EDIT_BALANCE_VOLTAGE:
        if (newVal >= 0.0f && newVal <= 6.0f) {
            eepromdata.balanceVoltage = newVal;
            EEPROMSettings::save();
            Logger::console("Balance Voltage set to: %.3f V", eepromdata.balanceVoltage);
        } else {
            Logger::console("Invalid value. Range: 0.0 - 6.0 V");
        }
        break;
    case EDIT_BALANCE_HYST:
        if (newVal >= 0.0f && newVal <= 1.0f) {
            eepromdata.balanceHyst = newVal;
            EEPROMSettings::save();
            Logger::console("Balance Hysteresis set to: %.3f V", eepromdata.balanceHyst);
        } else {
            Logger::console("Invalid value. Range: 0.0 - 1.0 V");
        }
        break;
    default:
        Logger::console("Unknown setting.");
        break;
    }

    returnToConfigMenu();
}

void Menu::returnToConfigMenu() {
    currentState = CONFIG_MENU;
    printConfigMenu();
}

void Menu::handleModuleCommand(char c) {
    switch (c) {
    case '1': Logger::console("Sleeping all connected boards"); bms.sleepBoards(); break;
    case '2': Logger::console("Waking up all connected boards"); bms.wakeBoards(); break;
    case '3': bms.findBoards(); break;
    case '4': Logger::console("Renumbering all boards"); bms.renumberBoardIDs(); break;
    case '5': Logger::console("Clearing all faults"); bms.clearFaults(); break;
    case '6': bms.balanceCells(); break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleLoggingCommand(char c) {
    switch (c) {
    case '1': Logger::setLoglevel(Logger::Debug); eepromdata.logLevel = 0; EEPROMSettings::save(); Logger::console("Log level set to: Debug"); break;
    case '2': Logger::setLoglevel(Logger::Info);  eepromdata.logLevel = 1; EEPROMSettings::save(); Logger::console("Log level set to: Info"); break;
    case '3': Logger::setLoglevel(Logger::Warn);  eepromdata.logLevel = 2; EEPROMSettings::save(); Logger::console("Log level set to: Warning"); break;
    case '4': Logger::setLoglevel(Logger::Error); eepromdata.logLevel = 3; EEPROMSettings::save(); Logger::console("Log level set to: Error"); break;
    case '5': Logger::setLoglevel(Logger::Off);   eepromdata.logLevel = 4; EEPROMSettings::save(); Logger::console("Log level set to: Off"); break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleDefaultsCommand(char c) {
    switch (c) {
    case '1':
        Logger::console("Resetting all settings to factory defaults...");
        EEPROMSettings::loadDefaults();
        Logger::console("Done. Settings restored to defaults.");
        break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}
