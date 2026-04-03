#pragma once

#include "config.h"
#include "BMSModule.h"

class BMSModuleManager
{
public:
    // --- Nested types ---
    struct BatterySummary {
        float   voltage;     // pack voltage in V
        float   current;     // always 0.0f
        uint8_t soc;         // 0-100
        int8_t  avgTemp;     // °C
        int8_t  minTemp;     // °C
        int8_t  maxTemp;     // °C
    };

    struct ModuleSummary {
        float   voltage;     // module voltage in V
        float   current;     // always 0.0f
        uint8_t soc;         // 0-100
        int8_t  avgTemp;     // °C
        int8_t  minTemp;     // °C
        int8_t  maxTemp;     // °C
    };

    struct CellDetails {
        float   cellVoltage;      // this cell voltage in V
        float   highestCellVolt;  // highest cell in this module
        float   lowestCellVolt;   // lowest cell in this module
        int8_t  highTemp;         // module high temp in °C
        uint8_t faultBits;        // always 0
    };

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
    // void readSetpoints();  Unused logic from oringinal implementation ...not sure what for... Reading whats stored on the chip? doesnt seem useful since hackaday says the setvalues are upstream... Maybe useful for fault line?

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
    float     packVolt;                          // All modules added together
    float     lowestPackVolt;
    float     highestPackVolt;
    float     lowestPackTemp;
    float     highestPackTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1];      // store data for as many modules as we've configured for
    int       numFoundModules;                   // The number of modules that seem to exist
    bool      isFaulted;
};