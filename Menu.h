#pragma once

#include <Arduino.h>
#include "Logger.h"

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
    bool isMenuOpen = true;           // new flag
    bool printPrettyDisplay = false;
    uint32_t prettyCounter = 0;
    int whichDisplay = 0;

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
};