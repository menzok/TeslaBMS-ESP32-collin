#pragma once
#include <Arduino.h>
#include "config.h"

// ─── Command codes ────────────────────────────────────────────────────────────
#define EXT_CMD_SHUTDOWN      0x01   // Open contactors (via Overlord)
#define EXT_CMD_STARTUP       0x02   // Close contactors (via Overlord)
#define EXT_CMD_SEND_DATA     0x03   // Request immediate data packet (carries shunt current)

// ─── Alarm flag bitmask ───────────────────────────────────────────────────────
// Bits 0-4 implemented, bits 5-15 reserved for future fault expansion
#define ALARM_NONE            0x0000
#define ALARM_OVER_VOLTAGE    (1 << 0)
#define ALARM_UNDER_VOLTAGE   (1 << 1)
#define ALARM_OVER_TEMP       (1 << 2)
#define ALARM_UNDER_TEMP      (1 << 3)
#define ALARM_OVER_CURRENT    (1 << 4)
// bits 5-15: reserved, not yet implemented

// ─── statusFlags byte (payload[24]) ──────────────────────────────────────────
#define STATUS_FLAG_CURRENT_SENSOR  (1 << 0)  // bit 0: currentSensorPresent
#define STATUS_FLAG_BALANCING       (1 << 1)  // bit 1: any cell currently balancing

// ─── overlordState encoding (payload[12]) ────────────────────────────────────
#define OVERLORD_STATE_NORMAL   0
#define OVERLORD_STATE_FAULT    1
#define OVERLORD_STATE_STORAGE  2
#define OVERLORD_STATE_SHUTDOWN 3   // clean shutdown — distinct from Fault

// ─── Command frame geometry ───────────────────────────────────────────────────
//
//  SHUTDOWN / STARTUP (4 bytes):
//    [0xAA][cmd][CRC_lo][CRC_hi]
//    CRC computed over [cmd] (1 byte)
//
//  SEND_DATA (7 bytes) — ping-pong data exchange:
//    [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]
//    CRC computed over [cmd][curr_hi][curr_lo][staleness] (4 bytes)
//    curr = Venus shunt/SCS current × 10 (int16 BE, + = charge), 100 mA res
//    staleness = 0 if shunt data is fresh, >0 if stale or unavailable
//
#define EXT_SEND_DATA_FRAME_LEN  7   // CMD_SEND_DATA frame length
#define EXT_CTRL_FRAME_LEN       4   // SHUTDOWN / STARTUP frame length

// ─── Shunt data staleness ─────────────────────────────────────────────────────
// Venus polls every ~2 s.  We declare shunt data stale if we have not received
// a fresh SEND_DATA frame within SHUNT_MAX_AGE_MS milliseconds.  Three missed
// polls (6 s) is enough to detect a lost connection while remaining robust
// against a single delayed response.
#define SHUNT_MAX_AGE_MS  6000UL  // 3 × 2 s poll interval

// ─── Reply frame geometry ─────────────────────────────────────────────────────
// Total frame: 1 (start 0xAA) + 34 (payload) + 2 (CRC) = 37 bytes
#define EXT_PAYLOAD_LEN       34
#define EXT_FRAME_LEN         37

//
// Payload layout (34 bytes, all multi-byte fields big-endian)
// ─────────────────────────────────────────────────────────────
//  [0-1]   packV               uint16 BE   voltage        × 100   → V   (10 mV res)
//  [2-3]   packI               int16  BE   current        × 10    → A   (100 mA res, + = charge)
//  [4]     soc                 uint8       0-100 %
//  [5]     temp                int8        °C  (signed, average)
//  [6-7]   power               int16  BE   W
//  [8-9]   avgCell             uint16 BE   avg cell volt  × 100   → V   (10 mV res)
//  [10-11] alarmFlags          uint16 BE   bitmask (ALARM_* above, bits 5-15 reserved)
//  [12]    overlordState       uint8       0=Normal 1=Fault 2=StorageMode 3=Shutdown
//  [13]    contactorState      uint8       raw contactor enum
//  [14-15] overVoltage         uint16 BE   EEPROM OV      × 1000  → V   (1 mV res)
//  [16-17] underVoltage        uint16 BE   EEPROM UV      × 1000  → V   (1 mV res)
//  [18]    overTemp            int8        EEPROM OT °C (signed, whole degrees)
//  [19]    underTemp           int8        EEPROM UT °C (signed, whole degrees)
//  [20]    numModules          uint8       bms.getNumberOfModules()
//  [21]    numStrings          uint8       eepromdata.parallelStrings
//  [22-23] overCurrentThresh   uint16 BE   EEPROM OVERCURRENT × 10 → A  (0.1A res)
//  [24]    statusFlags         uint8       bit0=currentSensorPresent bit1=balancingActive
//  [25]    activeFaultMask     uint8       1<<FaultEntry::Type for each active fault
//  [26-27] lowestCellV         uint16 BE   lowest cell voltage × 1000 → V (1 mV res)
//  [28-29] highestCellV        uint16 BE   highest cell voltage × 1000 → V (1 mV res)
//  [30]    minTemp             int8        minimum module temperature °C (signed)
//  [31]    maxTemp             int8        maximum module temperature °C (signed)
//  [32]    reserved            uint8       0x00
//  [33]    reserved            uint8       0x00
//

class ExternalCommsLayer {
public:
    void init();    // call once in setup()
    void update();  // call every loop after Overlord.update()

    // ── Venus shunt data (embedded in every CMD_SEND_DATA request) ────────────
    float    getShuntCurrentAmps()  const { return _shuntCurrentA; }
    uint8_t  getShuntStaleness()    const { return _shuntStaleness; }
    // Returns true if a SEND_DATA request was received within SHUNT_MAX_AGE_MS
    // AND the staleness counter sent by Venus is zero (shunt data was fresh at source).
    bool     isShuntDataFresh()     const {
        return (_shuntStaleness == 0) &&
               ((millis() - _shuntUpdateMs) < SHUNT_MAX_AGE_MS);
    }

private:
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
    void buildPayload(uint8_t* payload);   // fills 34-byte payload
    void sendPacket();                     // transmits full 37-byte frame
    void processIncomingCommand();

    uint8_t  txBuffer[EXT_FRAME_LEN];

    // ── Received shunt data (from Venus, carried in CMD_SEND_DATA) ────────────
    float    _shuntCurrentA   = 0.0f;
    uint8_t  _shuntStaleness  = 255;  // 255 = never received
    uint32_t _shuntUpdateMs   = 0;
};

extern ExternalCommsLayer ExternalComms;