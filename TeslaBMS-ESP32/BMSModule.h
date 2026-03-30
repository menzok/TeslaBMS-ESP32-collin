#pragma once

class BMSModule
{
public:
    BMSModule();
    void readStatus();
    void clearmodule();
    void stopBalance();
    bool readModuleValues();
    int getscells();
    float getCellVoltage(int cell);
    float getLowCellV();
    float getHighCellV();
    float getAverageV();
    float getLowTemp();
    float getHighTemp();
    float getHighestModuleVolt();
    float getLowestModuleVolt();
    float getHighestCellVolt(int cell);
    float getLowestCellVolt(int cell);
    float getHighestTemp();
    float getLowestTemp();
    float getAvgTemp();
    float getModuleVoltage();
    float getTemperature(int temp);
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();
    void setAddress(int newAddr);
    int getAddress();
    bool isExisting();
    void setExists(bool ex);
    void settempsensor(int tempsensor);
    void setIgnoreCell(float Ignore);
    void balanceCells();
    uint8_t getBalancingState(int cell);

private:
    float cellVolt[6];          // calculated as 16 bit value * 6.250 / 16383 = volts
    float lowestCellVolt[6];
    float highestCellVolt[6];
    float moduleVolt;           // summed cell voltages
    float retmoduleVolt;        // raw reported module voltage (16-bit * 0.002034629)
    float temperatures[2];
    float lowestTemperature;
    float highestTemperature;
    float lowestModuleVolt;
    float highestModuleVolt;
    float IgnoreCell;           // cells at or below this voltage are skipped (dead-cell filter)
    uint8_t balanceState[6];    // 0 = off, 1 = balancing active for this cell
    bool exists;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;
    int goodPackets;
    int badPackets;
    int sensor;                 // 0=both, 1=TS1 only, 2=TS2 only
    int scells;                 // number of cells above IgnoreCell threshold
    int smiss;                  // consecutive reads where scells differed (debounce counter)
    uint8_t moduleAddress;      //1 to 0x3E
};
