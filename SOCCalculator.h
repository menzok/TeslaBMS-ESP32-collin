#pragma once
#include <Arduino.h>
#include "EEPROMSettings.h"
#include "BMSModuleManager.h"
#include "config.h"

extern BMSModuleManager bms;
// ─── Coulomb counter auto-reset thresholds ───────────────────────────────────
#define SOC_CELL_FULL_VOLTAGE       4.15f   // V/cell - declare 100%
#define SOC_CELL_EMPTY_VOLTAGE      3.00f   // V/cell - declare 0%
#define SOC_RESET_CONFIRM_TICKS     5       // consecutive ticks before reset fires

// ─── OCV correction ──────────────────────────────────────────────────────────
#define SOC_OCV_REST_BLEND_RATE     0.02f   // fraction of error corrected per tick at rest
#define SOC_ZERO_CURRENT_THRESHOLD  1.0f    // amps - below this = pack at rest

// ─── ADC ─────────────────────────────────────────────────────────────────────
#define SOC_ADC_OVERSAMPLE          8       // samples averaged per current reading
#define SOC_ZERO_CAL_SAMPLES        32      // blocking samples taken in begin()

// ─── OCV-SOC lookup table ────────────────────────────────────────────────────
#define SOC_LUT_POINTS  14
#define SOC_LUT_TEMPS    4

static const float SOC_LUT_SOC[SOC_LUT_POINTS] = {
     0.0f,  5.0f, 10.0f, 15.0f, 20.0f, 30.0f,
    40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f,
    95.0f, 100.0f
};

static const float SOC_LUT_TEMP_POINTS[SOC_LUT_TEMPS] = {
    -10.0f, 10.0f, 25.0f, 40.0f
};

// [SOC_point][temp_col]
static const float SOC_LUT_OCV[SOC_LUT_POINTS][SOC_LUT_TEMPS] = {
    //   -10°C    10°C     25°C     40°C
        {3.3274f, 3.1611f, 3.0364f, 2.9881f},  //   0%
        {3.3070f, 3.1413f, 3.1795f, 3.1469f},  //   5%
        {3.4013f, 3.2651f, 3.2923f, 3.2576f},  //  10%
        {3.4547f, 3.3807f, 3.4031f, 3.3700f},  //  15%
        {3.4950f, 3.4534f, 3.4728f, 3.4564f},  //  20%
        {3.5304f, 3.4970f, 3.5641f, 3.5409f},  //  30%
        {3.6066f, 3.5989f, 3.6705f, 3.6588f},  //  40%
        {3.6935f, 3.7819f, 3.7642f, 3.7576f},  //  50%
        {3.7819f, 3.8574f, 3.8436f, 3.8416f},  //  60%
        {3.9069f, 3.9394f, 3.9423f, 3.9412f},  //  70%
        {3.9751f, 4.0338f, 4.0491f, 4.0455f},  //  80%
        {4.0571f, 4.0902f, 4.1002f, 4.1023f},  //  90%
        {4.0836f, 4.1083f, 4.1209f, 4.1246f},  //  95%
        {4.1609f, 4.1665f, 4.1776f, 4.1788f},  // 100%
};

// ─────────────────────────────────────────────────────────────────────────────

class SOCCalculator {
public:


    

    SOCCalculator();

    // Call once in setup() BEFORE contactor closes.
    // Blocks ~160ms for ADC zero calibration - safe, contactor is open.
    // cellsInSeries  : number of series cells (to get per-cell voltage)
    // packCapacityAh : total pack Ah

    void begin();


    // Call every SOC_UPDATE_INTERVAL_MS from main scheduler - never blocks
    void update();

    // Pack current in amps (+ charge, - discharge)
    // Returns 0.0 if currentSensorPresent == false
    // Used by ContactorController for precharge detection
    float getPackCurrentAmps() const;
 

    // SOC as 0-100 byte for BatterySummary.soc
    uint8_t getSOCByte() const;

private:
    // ── Internal ESP32 ADC constants ─────────────
    static constexpr float ADC_VREF_11DB = 3.9f;    // full scale at ADC_11db attenuation
    static constexpr float ADC_MAX_COUNT = 4095.0f;  // 12-bit resolution
	float   currentSensorZeroOffsetV;   // Sensor Drift on ADC measured at boot but needs running calibration.
    // ── Runtime state ───────────────────────────────────────────────────
    float         _packCapacityAh;
    int           _cellsInSeries;
    float         _filteredCurrentA;
    float         _lastCurrentA;
    unsigned long _lastUpdateMs;
    bool          _initialised;

    // ── Endpoint reset confirmation ───────────────────────────────────────
    uint8_t       _fullConfirmTicks;
    uint8_t       _emptyConfirmTicks;

    // ── Helpers ───────────────────────────────────────────────────────────
    float   _readCurrentAmps()                          const;
    float   _adcToVoltage(int raw)                      const;
    float   _ocvToSOC(float cellVoltage, float tempC)   const;
    float   _clampSOC(float soc)                        const;
};