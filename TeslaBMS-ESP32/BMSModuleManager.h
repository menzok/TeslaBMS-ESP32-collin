#pragma once
#include "config.h"
#include "BMSModule.h"


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
    float getHighVoltage();
    float getLowVoltage();
    int getBalancing();
    int getNumModules();
    void printPackSummary();
    void printPackDetails();

private:
    float packVolt;                         // All modules added together
    int Pstring;
    float LowCellVolt;
    float HighCellVolt;
    float lowestPackVolt;
    float highestPackVolt;
    float lowestPackTemp;
    float highestPackTemp;
    float highTemp;
    float lowTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules;                    // The number of modules that seem to exist
    bool isFaulted;
    int spack;
    int CellsBalancing;
};
