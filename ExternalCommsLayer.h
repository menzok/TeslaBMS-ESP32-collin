#pragma once
#include <Arduino.h>
#include "config.h"

// ─── Command codes ────────────────────────────────────────────────────────────
#define EXT_CMD_SHUTDOWN      0x01   // Open contactors (via Overlord)
#define EXT_CMD_STARTUP       0x02   // Close contactors (via Overlord)
#define EXT_CMD_SEND_DATA     0x03   // Request immediate data packet

// ─── Alarm flag bitmask ───────────────────────────────────────────────────────
// Bits 0-4 implemented, bits 5-15 reserved for future fault expansion
#define ALARM_NONE            0x0000
#define ALARM_OVER_VOLTAGE    (1 << 0)
#define ALARM_UNDER_VOLTAGE   (1 << 1)
#define ALARM_OVER_TEMP       (1 << 2)
#define ALARM_UNDER_TEMP      (1 << 3)
#define ALARM_OVER_CURRENT    (1 << 4)
// bits 5-15: reserved, not yet implemented

// ─── Frame geometry ───────────────────────────────────────────────────────────
// Total frame: 1 (start 0xAA) + 26 (payload) + 2 (CRC) = 29 bytes
#define EXT_PAYLOAD_LEN       26
#define EXT_FRAME_LEN         29

//
// Payload layout (26 bytes, all multi-byte fields big-endian)
// ─────────────────────────────────────────────────────────────
//  [0-1]   packV               uint16 BE   voltage        × 100   → V   (10 mV res)
//  [2-3]   packI               int16  BE   current        × 10    → A   (100 mA res, + = charge)
//  [4]     soc                 uint8       0-100 %
//  [5]     temp                int8        °C  (signed)
//  [6-7]   power               int16  BE   W
//  [8-9]   avgCell             uint16 BE   avg cell volt  × 100   → V   (10 mV res)
//  [10-11] alarmFlags          uint16 BE   bitmask (ALARM_* above, bits 5-15 reserved)
//  [12]    overlordState       uint8       0=Normal 1=Fault 2=StorageMode
//  [13]    contactorState      uint8       raw contactor enum
//  [14-15] overVoltage         uint16 BE   EEPROM OV      × 1000  → V   (1 mV res)
//  [16-17] underVoltage        uint16 BE   EEPROM UV      × 1000  → V   (1 mV res)
//  [18]    overTemp            int8        EEPROM OT °C (signed, whole degrees)
//  [19]    underTemp           int8        EEPROM UT °C (signed, whole degrees)
//  [20]    numModules          uint8       bms.getNumberOfModules()
//  [21]    numStrings          uint8       eepromdata.PARALLEL_STRINGS
//  [22-23] overCurrentThresh   uint16 BE   EEPROM OVERCURRENT × 10 → A  (0.1A res)
//  [24]    reserved            uint8       0x00
//  [25]    reserved            uint8       0x00
//

class ExternalCommsLayer {
public:
    void init();    // call once in setup()
    void update();  // call every loop after Overlord.update()

private:
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
    void buildPayload(uint8_t* payload);   // fills 26-byte payload
    void sendPacket();                     // transmits full 29-byte frame
    void processIncomingCommand();

    uint8_t txBuffer[EXT_FRAME_LEN];
};

extern ExternalCommsLayer ExternalComms;