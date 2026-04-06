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
    enum MenuState {
        ROOT_MENU,
        CONFIG_MENU,
        MODULE_MENU,
        LOGGING_MENU,
        DEFAULTS_MENU,
        CONTACTOR_CURRENT_MENU,   // renamed from ADDITIONAL_HARDWARE_MENU
        FAULT_LOG_MENU,           // new: display-only fault log
        BATTERY_CONFIG_MENU,      // new: battery config & status
        WAITING_FOR_INPUT
    };
    enum PendingEdit {
        NO_EDIT,
        // Safety Thresholds
        EDIT_OVER_VOLTAGE,
        EDIT_UNDER_VOLTAGE,
        EDIT_OVER_TEMP,
        EDIT_UNDER_TEMP,
        EDIT_BALANCE_VOLTAGE,
        EDIT_BALANCE_HYST,
        EDIT_OVERCURRENT_THRESHOLD_A,   // new: over-current trip threshold
        EDIT_CELL_FAULT_DEBOUNCE,       // new: consecutive fault readings before fault declared
        // Contactor & Current Sensor
        EDIT_PRECHARGE_TIMEOUT_MS,
        EDIT_CURRENT_SENSOR_VBIAS,      // new: current sensor bias voltage
        EDIT_CURRENT_SENSOR_VRANGE,     // new: current sensor voltage range
        EDIT_CURRENT_SENSOR_RATED_AMPS, // new: current sensor rated amperage
        // Battery Config
        EDIT_PARALLEL_STRINGS,              // new: number of parallel battery strings
        EDIT_STORAGE_WAKE_INTERVAL_HOURS,   // new: storage wake interval (display/edit in hours, stored as ms)
        EDIT_STORAGE_BALANCE_DURATION_MINS  // new: storage balance duration (display/edit in minutes, stored as ms)
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

    // Shared helper — prints the pretty-display pause warning on every menu
    void printPrettyWarning();

    void printRootMenu();
    void printConfigMenu();
    void printModuleMenu();
    void printLoggingMenu();
    void printDefaultsMenu();
    void printContactorCurrentMenu();   // renamed from printAdditionalHardwareMenu
    void printFaultLogMenu();           // new: display-only fault log
    void printBatteryConfigMenu();      // new: battery config & status

    void handleRootCommand(char c);
    void handleConfigCommand(char c);
    void handleModuleCommand(char c);
    void handleLoggingCommand(char c);
    void handleDefaultsCommand(char c);
    void handleContactorCurrentCommand(char c);  // renamed from handleAdditionalHardwareCommand
    void handleFaultLogCommand(char c);          // new
    void handleBatteryConfigCommand(char c);     // new

    // WAITING_FOR_INPUT dispatcher — routes to one of the per-menu handlers below
    void handleWaitingForInput();

    // Per-menu input handlers — each menu owns its own input handling and return
    void handleConfigWaitingInput();
    void handleContactorCurrentWaitingInput();   // renamed from handleAdditionalHardwareWaitingInput
    void handleBatteryConfigWaitingInput();       // new
    // Add new per-menu handlers here;

    void returnToConfigMenu();
    void returnToContactorCurrentMenu();  // renamed from returnToAdditionalHardwareMenu
    void returnToBatteryConfigMenu();     // new

    // Helper to perform X-exit and print the exit message
    void exitMenu();
};