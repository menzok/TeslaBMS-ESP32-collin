#pragma once

#include <Arduino.h>
#include "config.h"
#include "Logger.h"
#include "EEPROMSettings.h"

class Menu {
public:
    Menu();
    void loop();
    void handleInput(char c);
    
private:
	enum MenuState {  //What menu we are currently in, used to determine what to print and how to handle input
        ROOT_MENU,
        CONFIG_MENU,
        MODULE_MENU,
        LOGGING_MENU,
        DEFAULTS_MENU,
        WAITING_FOR_INPUT
    };
	enum PendingEdit { //What setting we are currently waiting for input on, used to determine how to handle input and return to the correct menu after input is received
        NO_EDIT,
        EDIT_OVER_VOLTAGE,
        EDIT_UNDER_VOLTAGE,
        EDIT_OVER_TEMP,
        EDIT_UNDER_TEMP,
        EDIT_BALANCE_VOLTAGE,
        EDIT_BALANCE_HYST
        // Add new editable fields here
    };


    MenuState currentState = ROOT_MENU;
    bool isMenuOpen = true;
    bool printPrettyDisplay = false;
    uint32_t prettyCounter = 0;
    int whichDisplay = 0;


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