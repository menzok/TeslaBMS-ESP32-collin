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

    struct FaultEntry {
        enum class Type : uint8_t {
            None,
            OverVoltage,
            UnderVoltage,
            OverTemperature,
            UnderTemperature,
            OverCurrent
        };
        Type type = Type::None;
        uint8_t module = 0;
        uint8_t cell = 0;
        float value = 0.0f;
        uint32_t timestamp = 0;          // when fault occurred
        uint32_t clearedTimestamp = 0;   // 0 = still active
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
    // Configurable constants (easy to promote to eepromdata later)
    static constexpr float OVERCURRENT_THRESHOLD_A = 350.0f;
    static constexpr uint32_t STORAGE_WAKE_INTERVAL_MS = 24 * 60 * 60 * 1000UL;  // 24 hours
    static constexpr uint32_t STORAGE_BALANCE_DURATION_MS = 120000UL;           // 2 minutes

	static constexpr uint8_t CELL_FAULT_DEBOUNCE = 3;  // number of consecutive fault readings before faulting (to prevent noise/chatter)

    // === Variables intended to be moved to EEPROM later ===
    FaultEntry faultLog[5]; 
    

    // Internal state
    BMSState currentState = BMSState::Normal;
    bool shutdownRequested = false;
    bool storageModeActive = false;

    // Timers
    uint32_t lastUpdateMs = 0;
    uint32_t lastStorageWakeMs = 0;
    uint32_t storageBalanceEndMs = 0;
    bool balancingActive = false;

    // Watchdog
    bool watchdogTightened = false;
    uint8_t successfulUpdates = 0;

    // Safety debounce (supports up to 64 modules)
    uint8_t ovDebounce[64][6] = {};
    uint8_t uvDebounce[64][6] = {};
    uint8_t otDebounce[64][6] = {};
    uint8_t utDebounce[64][6] = {};

    void runSafetyChecks();
    void handleContactorLogic();
    void handleStorageMode();
    void logFault(FaultEntry::Type type, uint8_t module, uint8_t cell, float value);
    void clearLastFaultIfResolved();
};
