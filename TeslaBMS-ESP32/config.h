#pragma once

#include <Arduino.h>

extern HardwareSerial Serial2;

//Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
#define SERIALCONSOLE   Serial

//Define this to be the serial port the Tesla BMS modules are connected to.
//On the Due you need to use a USART port (Serial1, Serial2, Serial3) and update the call to serialSpecialInit if not Serial1
#define SERIAL  Serial2

#define REG_DEV_STATUS      0
#define REG_GPAI            1
#define REG_VCELL1          3
#define REG_VCELL2          5
#define REG_VCELL3          7
#define REG_VCELL4          9
#define REG_VCELL5          0xB
#define REG_VCELL6          0xD
#define REG_TEMPERATURE1    0xF
#define REG_TEMPERATURE2    0x11
#define REG_ALERT_STATUS    0x20
#define REG_FAULT_STATUS    0x21
#define REG_COV_FAULT       0x22
#define REG_CUV_FAULT       0x23
#define REG_ADC_CTRL        0x30
#define REG_IO_CTRL         0x31
#define REG_BAL_CTRL        0x32
#define REG_BAL_TIME        0x33
#define REG_ADC_CONV        0x34
#define REG_ADDR_CTRL       0x3B

#define MAX_MODULE_ADDR     0x3E

// Voltage above which a cell reading is considered invalid (hardware maximum)
#define CELL_MAX_VALID_VOLT 4.5f
// Voltage above which a cell reading is treated as an open-circuit/invalid reading
#define CELL_OPEN_VOLT      60.0f
// Temperature below which a sensor is considered disconnected
#define TEMP_SENSOR_DISCONNECTED -70.0f

#define EEPROM_VERSION      0x11    //update any time EEPROM struct below is changed.
#define EEPROM_PAGE         0

#define DIN1                55
#define DIN2                54
#define DIN3                57
#define DIN4                56
#define DOUT4_H             2
#define DOUT4_L             3
#define DOUT3_H             4
#define DOUT3_L             5
#define DOUT2_H             6
#define DOUT2_L             7
#define DOUT1_H             8
#define DOUT1_L             9

typedef struct {
    uint8_t version;
    uint8_t checksum;
    uint32_t canSpeed;
    uint8_t batteryID;  //which battery ID should this board associate as on the CAN bus
    uint8_t logLevel;
    float OverVSetpoint;    // per-cell overvoltage trip threshold (V)
    float UnderVSetpoint;   // per-cell undervoltage trip threshold (V)
    float ChargeVsetpoint;  // per-cell target voltage during charging (V)
    float DischVsetpoint;   // per-cell minimum voltage during discharge (V)
    float ChargeHys;        // hysteresis below ChargeVsetpoint before re-enabling charge (V)
    float DischHys;         // hysteresis above DischVsetpoint before re-enabling discharge (V)
    float WarnOff;          // voltage offset below OverV or above UnderV to warn before tripping (V)
    float CellGap;          // maximum allowed voltage gap between highest and lowest cell (V)
    float IgnoreVolt;       // ignore cells below this voltage (dead-cell detection, V)
    uint8_t IgnoreTemp;     // 0=use both sensors, 1=sensor1 only, 2=sensor2 only
    float OverTSetpoint;    // over-temperature trip threshold (°C)
    float UnderTSetpoint;   // under-temperature trip threshold (°C)
    float balanceVoltage;   // cell voltage at which balancing activates (V)
    float balanceHyst;      // how far voltage must drop below balanceVoltage to stop balancing (V)
    int Scells;             // number of cells in series per module string
    int Pstrings;           // number of parallel strings
    uint16_t triptime;      // milliseconds a fault must persist before tripping (ms)
} EEPROMSettings;
