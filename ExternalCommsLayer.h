#pragma once
#include <Arduino.h>
#include "config.h"

// Command codes (you can change these if you prefer different values)
#define EXT_CMD_SHUTDOWN      0x01
#define EXT_CMD_STARTUP       0x02
#define EXT_CMD_SEND_DATA     0x03

// Alarm flag stubs (expand later when Overlord adds fault bits)
#define ALARM_NONE            0x0000
#define ALARM_OVER_VOLTAGE    (1 << 0)
#define ALARM_UNDER_VOLTAGE   (1 << 1)
#define ALARM_OVER_TEMP       (1 << 2)
#define ALARM_UNDER_TEMP      (1 << 3)
#define ALARM_OVER_CURRENT    (1 << 4)

class ExternalCommsLayer {
public:
    void init();   // call once in setup()
    void update(); // call every ~1 s after Overlord.update()

private:
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
    void buildPayload(uint8_t* payload);
    void sendPacket();
    bool processIncomingCommand();

    uint8_t txBuffer[21];  // 0xAA + 18-byte payload + 2-byte CRC
};

extern ExternalCommsLayer ExternalComms;