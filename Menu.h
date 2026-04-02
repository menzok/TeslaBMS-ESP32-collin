#pragma once

#include <Arduino.h>
#include "config.h"
#include "Logger.h"
#include "EEPROMSettings.h"

class Menu {
public:
    Menu();
    void begin();
    void loop();
    void handleInput(char c);

private:
    enum MenuState {
        ROOT_MENU,
        CONFIG_MENU,
        MODULE_MENU,
        LOGGING_MENU,
        DEFAULTS_MENU,
        WAITING_FOR_INPUT
    };

    MenuState currentState = ROOT_MENU;
    bool isMenuOpen = true;
    bool printPrettyDisplay = false;
    uint32_t prettyCounter = 0;
    int whichDisplay = 0;
    enum PendingEdit {
        NO_EDIT,
        EDIT_OVER_VOLTAGE,
        EDIT_UNDER_VOLTAGE,
        EDIT_OVER_TEMP,
        EDIT_UNDER_TEMP,
        EDIT_BALANCE_VOLTAGE,
        EDIT_BALANCE_HYST
        // Add new editable fields here — no magic numbers
    };

    PendingEdit pendingEdit = NO_EDIT;

    unsigned char cmdBuffer[80];
    uint8_t ptrBuffer = 0;

    void printRootMenu();
    void printConfigMenu();
    void printModuleMenu();
    void printLoggingMenu();
    void printDefaultsMenu();

    void handleRootCommand(char c);
    void handleConfigCommand(char c);
    void handleModuleCommand(char c);
    void handleLoggingCommand(char c);
    void handleDefaultsCommand(char c);
    void handleWaitingForInput();
    void returnToConfigMenu();
};