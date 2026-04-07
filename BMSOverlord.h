#pragma once

#include "BMSModuleManager.h"
#include "ContactorController.h"
#include "SOCCalculator.h"
#include "EEPROMSettings.h"
#include <Arduino.h>

extern BMSModuleManager bms;
extern ContactorController contactor;
extern SOCCalculator socCalculator;
extern EEPROMData eepromdata;

class BMSOverlord {
public:
    enum class BMSState : uint8_t {
        Normal,
        Warning,
        Fault,
        StorageMode,
        Shutdown
    };

    void init();
    void update();

    // External control layer (Victron OS, etc.)
    void requestShutdown();   // Storage / maintenance mode
    void requestStartup();    // Normal operation

    // Status for menu / comms layer
    BMSState getState() const { return currentState; }
    bool isFaulted() const { return currentState == BMSState::Fault || currentState == BMSState::Shutdown; }
    

private:
 
    FaultEntry* faultLog;  //EEPROM array for storing faults.
    

    // Internal state
    BMSState currentState = BMSState::Normal;
    bool storageModeActive = false;

    // Timers
    uint32_t lastUpdateMs = 0;
    uint32_t lastStorageWakeMs = 0;
    uint32_t storageBalanceStartMs = 0;
    uint32_t lastFaultSaveMs = 0;
    bool balancingActive = false;

    // Watchdog
    bool watchdogTightened = false;
    uint8_t successfulUpdates = 0;

    // Safety debounce (supports up to 64 modules)
    uint8_t ovDebounce[64][6] = {};
    uint8_t uvDebounce[64][6] = {};
    uint8_t otDebounce[64][6] = {};
    uint8_t utDebounce[64][6] = {};
	uint8_t lastSuccessfulModules = 0;
    uint8_t commsDebounce = 0;

    void runSafetyChecks();
    void handleContactorLogic();
    void handleStorageMode();
    void logFault(FaultEntry::Type type, uint8_t module, uint8_t cell, float value);
    void clearLastFaultIfResolved();
    
};
