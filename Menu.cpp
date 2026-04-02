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
#include "Logger.h"

template<class T> inline Print& operator <<(Print& obj, T arg) { obj.print(arg); return obj; } //Lets us stream SerialUSB


extern BMSModuleManager bms;

Menu::Menu() {
    isMenuOpen = true;
    printPrettyDisplay = false;
    prettyCounter = 0;
    whichDisplay = 0;
}

void Menu::begin() {
    currentState = ROOT_MENU;
    isMenuOpen = true;
    printPrettyDisplay = false;   // start with pretty print off
    printRootMenu();
}

void Menu::loop() {
    if (!isMenuOpen && printPrettyDisplay && (millis() > (prettyCounter + 3000))) {
        prettyCounter = millis();
        if (whichDisplay == 0) bms.printPackSummary();
        if (whichDisplay == 1) bms.printPackDetails();
    }
}

void Menu::handleInput(char c) {
    if (c == '\n' || c == '\r') {
        // Line complete — process the command
        if (ptrBuffer == 1) {
            // Single character command (menu navigation)
            char cmd = cmdBuffer[0];
            switch (currentState) {
            case ROOT_MENU:     handleRootCommand(cmd);     break;
            case CONFIG_MENU:   handleConfigCommand(cmd);   break;
            case MODULE_MENU:   handleModuleCommand(cmd);   break;
            case LOGGING_MENU:  handleLoggingCommand(cmd);  break;
            case DEFAULTS_MENU: handleDefaultsCommand(cmd); break;
            case STATE_WAITING_FOR_INPUT:
                // Long input finished — handle it here later
                Logger::console("Long input received: ");
                Logger::console(cmdBuffer);
                // TODO: process password, value, etc.
                currentState = ROOT_MENU;   // return to root for now
                printRootMenu();
                break;
            }
        }
        else {
            // Long command received (e.g. VOLTLIMHI=4.25 or WiFi password)
            if (currentState == STATE_WAITING_FOR_INPUT) {
                Logger::console("Long input finished: ");
                Logger::console(cmdBuffer);
                // TODO: parse and save the value here
            }
            else {
                Logger::console("Long command received (not yet supported)");
            }
        }
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
    Logger::console("5. Toggle Pretty Print");
    Logger::console("0. Back / Exit");
    Logger::console("Type 'menu' or 'm' from anywhere to return here.");
}

void Menu::printConfigMenu() {
    Logger::console("\n=== Configuration Settings ===");
    Logger::console("1. Voltage Limits");
    Logger::console("2. Temperature Limits");
    Logger::console("3. Balancing Settings");
    Logger::console("4. Pack Configuration");
    Logger::console("0. Back to Root");
}

void Menu::printModuleMenu() {
    Logger::console("\n=== Module Operations ===");
    Logger::console("1. Sleep Boards");
    Logger::console("2. Wake Boards");
    Logger::console("3. Find Boards");
    Logger::console("4. Renumber Boards");
    Logger::console("5. Clear Faults");
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
        if (printPrettyDisplay) {
            Logger::console("Pretty Print ENABLED");
        }
        else {
            Logger::console("Pretty Print DISABLED");
        }
        break;
    case '0':
        Logger::console("Exiting menu.");
        isMenuOpen = false;           // allow pretty print again
        break;
    default:
        Logger::console("Unknown option");
        break;
    }
}

void Menu::handleConfigCommand(char c) {
    switch (c) {
    case '1': Logger::console("Selected: Voltage Limits");   break;
    case '2': Logger::console("Selected: Temperature Limits"); break;
    case '3': Logger::console("Selected: Balancing Settings"); break;
    case '4': Logger::console("Selected: Pack Configuration"); break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleModuleCommand(char c) {
    switch (c) {
    case '1': Logger::console("Selected: Sleep Boards");   break;
    case '2': Logger::console("Selected: Wake Boards");    break;
    case '3': Logger::console("Selected: Find Boards");    break;
    case '4': Logger::console("Selected: Renumber Boards"); break;
    case '5': Logger::console("Selected: Clear Faults");   break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleLoggingCommand(char c) {
    switch (c) {
    case '1': Logger::console("Selected: Set Debug");  break;
    case '2': Logger::console("Selected: Set Info");   break;
    case '3': Logger::console("Selected: Set Warning"); break;
    case '4': Logger::console("Selected: Set Error");  break;
    case '5': Logger::console("Selected: Set Off");    break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}

void Menu::handleDefaultsCommand(char c) {
    switch (c) {
    case '1': Logger::console("Selected: Reset ALL settings to factory defaults"); break;
    case '0': currentState = ROOT_MENU; printRootMenu(); break;
    default:  Logger::console("Unknown option"); break;
    }
}