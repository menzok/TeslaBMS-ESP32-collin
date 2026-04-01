#pragma once

#include <Arduino.h>
extern HardwareSerial Serial2;

// Set to the proper port for your USB connection 
// (SerialUSB on Due Native, Serial for Due Programming, or Teensy, etc.)
#define SERIALCONSOLE   Serial

//BMS communication details
#define SERIAL          Serial2
#define BMS_BAUD  612500 
#define BMS_RX_PIN          16      // RX pin for Tesla BMS modules 
#define BMS_TX_PIN          17      // TX pin for Tesla BMS modules

// ====================== HARDWARE PINS & FEATURES ======================
// Configurable pins and enable/disable flags for optional hardware features.

#define FAULT_CHAIN_PIN     13      // GPIO the module fault chain is wired to (active LOW)
#define ENABLE_FAULT_CHAIN  0       // 1 = enabled (recommended), 0 = disabled


// ====================== BQ76PL455A-Q1 CHIP CONSTANTS ======================
// These are fixed hardware register addresses and limits inside every Tesla module.

#define REG_GPAI            1       // General Purpose Analog Input (module voltage)
#define REG_VCELL1          3       // Cell 1 voltage (16-bit ADC result)
#define REG_VCELL2          5       // Cell 2 voltage (16-bit ADC result)
#define REG_VCELL3          7       // Cell 3 voltage (16-bit ADC result)
#define REG_VCELL4          9       // Cell 4 voltage (16-bit ADC result)
#define REG_VCELL5          0xB     // Cell 5 voltage (16-bit ADC result)
#define REG_VCELL6          0xD     // Cell 6 voltage (16-bit ADC result)
#define REG_TEMPERATURE1    0xF     // Temperature sensor 1 (16-bit ADC result)
#define REG_TEMPERATURE2    0x11    // Temperature sensor 2 (16-bit ADC result)
#define REG_ALERT_STATUS    0x20    // Alert status flags
#define REG_FAULT_STATUS    0x21    // Fault status flags
#define REG_COV_FAULT       0x22    // Cell over-voltage fault status
#define REG_CUV_FAULT       0x23    // Cell under-voltage fault status
#define REG_ADC_CTRL        0x30    // ADC control register (start/stop conversions, etc.)
#define REG_IO_CTRL         0x31    // GPIO / I/O control register
#define REG_BAL_CTRL        0x32    // Balancing control (which cells to bleed)
#define REG_BAL_TIME        0x33    // Balancing time / PWM duty cycle
#define REG_ADC_CONV        0x34    // ADC conversion settings
#define REG_ADDR_CTRL       0x3B    // Module address control in daisy-chain
#define MAX_MODULE_ADDR     0x3E    // Limit of the chip hardware address scheme (0x01-0x3E)


// ====================== EEPROM SETTINGS ======================
// Version and struct used for persistent settings stored in EEPROM.

#define EEPROM_VERSION      0x10    // Update any time the EEPROM struct below is changed.

typedef struct {
    uint8_t version;
    uint8_t checksum;
    uint8_t logLevel;
    float OverVSetpoint;
    float UnderVSetpoint;
    float OverTSetpoint;
    float UnderTSetpoint;
    float balanceVoltage;
    float balanceHyst;
} EEPROMSettings;