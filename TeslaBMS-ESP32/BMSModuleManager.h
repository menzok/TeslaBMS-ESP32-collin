#pragma once
#include "config.h"
#include "BMSModule.h"

// Forward declaration – SafetyController.h includes BMSModuleManager.h,
// so we use a pointer + forward declaration to break the cycle.
class SafetyController;

class BMSModuleManager
{
public:
    BMSModuleManager();
    int seriescells();
    void clearmodules();
    void StopBalancing();
    void balanceCells();
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setSensors(int sensor, float Ignore);
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();    // Session-high pack voltage (all-time high since boot)
    float getLowVoltage();     // Session-low pack voltage (all-time low since boot)
    float getHighPackTemp();   // Session-high pack temperature (all-time high since boot)
    float getLowPackTemp();    // Session-low pack temperature (all-time low since boot)
    int getBalancing();
    int getNumModules();
    void printPackSummary();
    void printPackDetails();
    void setModuleTopology(int series, int parallel);
    void setCapacityPerStringAh(float ahPerString);
    float getSOC();
    void updateSOC();
    void loadSOCFromEEPROM();
    void saveSOCToEEPROM();
    void setCurrentAmps(float amps);
    float getCurrentAmps() { return currentAmps; }

    // --- Master updater ---
    // Call once per second from loop(). Executes in order:
    //   getAllVoltTemp (stops balance, reads fresh data) →
    //   getAvgTemperature → conditional balanceCells (only when high cell ≥ balanceVoltage) →
    //   updateSOC → SafetyController::update()
    void update();

    // Provide the SafetyController instance before the first update() call.
    void setSafetyController(SafetyController* sc);

private:
    float packVolt;
    int Pstring;
    float LowCellVolt;
    float HighCellVolt;
    float lowestPackVolt;   // Session-low pack voltage (all-time low since boot)
    float highestPackVolt;  // Session-high pack voltage (all-time high since boot)
    float lowestPackTemp;   // Session-low pack temperature (all-time low since boot)
    float highestPackTemp;  // Session-high pack temperature (all-time high since boot)
    float highTemp;
    float lowTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1];
    int batteryID;
    int numFoundModules;
    bool isFaulted;
    int spack;
    int CellsBalancing;
    int modulesInSeries = 0;
    int parallelStrings = 0;
    float capacityPerStringAh = 0.0f;
    float packCapacityAh = 0.0f;
    float remainingAh = 0.0f;
    float socPercent = 50.0f;
    float currentAmps = 0.0f;
    unsigned long lastSOCUpdate = 0;
    unsigned long lastSaveMillis = 0;
    unsigned long restStartTime = 0;

    SafetyController* safetyController = nullptr;
};
