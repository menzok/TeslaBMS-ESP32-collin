#pragma once

#include "config.h"
#include "BMSModule.h"

// --- Data structures for BMS status readout ---
struct BatterySummary {
    float   voltage;
    float   current;
    uint8_t soc;
    int8_t  avgTemp;
    int8_t  minTemp;
    int8_t  maxTemp;
};

struct ModuleSummary {
    float   voltage;
    float   current;
    uint8_t soc;
    int8_t  avgTemp;
    int8_t  minTemp;
    int8_t  maxTemp;
};

struct CellDetails {
    float   cellVoltage;
    float   highestCellVolt;
    float   lowestCellVolt;
    int8_t  highTemp;
    uint8_t faultBits;
};

class BMSModuleManager
{
public:
    // --- Constructor ---
    BMSModuleManager();

    // --- Setup & discovery ---
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();

    // --- Operations ---
    void balanceCells();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();

    // --- Accessors ---
    BatterySummary getBatterySummary();
    ModuleSummary  getModuleSummary(int module);
    CellDetails    getCellDetails(int module, int cell);
    int            getNumberOfModules() const;
    float          getPackVoltage();
    float          getAvgTemperature();
    float          getAvgCellVolt();

    // --- Serial Output ---
    void printPackSummary();
    void printPackDetails();

private:
    float     packVolt;
    float     lowestPackVolt;
    float     highestPackVolt;
    float     lowestPackTemp;
    float     highestPackTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1];
    int       numFoundModules;
};