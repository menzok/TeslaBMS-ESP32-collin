#include "SOCCalculator.h"
#include "ExternalCommsLayer.h"

// ─────────────────────────────────────────────────────────────────────────────
SOCCalculator::SOCCalculator()
    : currentSensorZeroOffsetV(0.0f)
    , _packCapacityAh(1.0f)
    , _cellsInSeries(1)
    , _filteredCurrentA(0.0f)
    , _lastCurrentA(0.0f)
    , _lastUpdateMs(0)
    , _initialised(false)
    , _fullConfirmTicks(0)
    , _emptyConfirmTicks(0)
    , _lastSaveMs(0)
    , _currentSensorAdaptiveOffsetV(0.0f)
{
}

// ─────────────────────────────────────────────────────────────────────────────
void SOCCalculator::begin()
{
    uint8_t strings = (eepromdata.parallelStrings > 0) ? eepromdata.parallelStrings : 1;
    _cellsInSeries  = (int)(bms.getNumberOfModules() / strings) * 6;
    if (_cellsInSeries == 0) _cellsInSeries = 1;   // prevent /0 in update()
    _packCapacityAh = (float)strings * 22.0f; // 22Ah per string for Tesla 18650 modules - adjust if using different cells
    _fullConfirmTicks  = 0;
    _emptyConfirmTicks = 0;
    _lastUpdateMs = millis();
    _initialised = true;

    // ── Poll live battery data (no more parameters) ───────────────────────
    BatterySummary summary = bms.getBatterySummary();
    float cellVoltage = summary.voltage / (float)_cellsInSeries;
    float cellTempC = (float)summary.avgTemp - 40.0f;   // decode +40 offset

    // Cold start - best guess from OCV lookup (eepromdata already loaded)
    if (eepromdata.socPercent < 0.0f || eepromdata.socPercent > 100.0f) {
        eepromdata.socPercent = _ocvToSOC(cellVoltage, cellTempC);
    }

    if (eepromdata.currentSensorPresent) {
        // ── Blocking zero-cal (unchanged) ─────────────────────────────────
        analogSetPinAttenuation(SOC_CURRENT_SENSOR_PIN, ADC_11db);

        float vSum = 0.0f;
        for (int n = 0; n < SOC_ZERO_CAL_SAMPLES; n++) {
            vSum += _adcToVoltage(analogRead(SOC_CURRENT_SENSOR_PIN));
            delay(5);
        }
        float vMeasured = vSum / (float)SOC_ZERO_CAL_SAMPLES;
        currentSensorZeroOffsetV = vMeasured - eepromdata.currentSensorVbias;

        // Seed filter at true zero
        _filteredCurrentA = 0.0f;
        _lastCurrentA = 0.0f;
    }
}

// ───────────────────────────────────────────────────────────────────────���─────
void SOCCalculator::update()
{
    if (!_initialised) return;
    unsigned long now = millis();


    unsigned long elapsed = now - _lastUpdateMs;

    // Skip if called too early or millis() rolled over
    if (elapsed == 0 || elapsed > 10000UL) {
        _lastUpdateMs = now;
        return;
    }
    _lastUpdateMs = now;
    BatterySummary summary = bms.getBatterySummary();
    float cellVoltage = summary.voltage / (float)_cellsInSeries;
    float cellTempC = (float)summary.avgTemp - 40.0f;   // decode +40 offset

    if (eepromdata.currentSensorPresent) {

        // ── Read and filter current ───────────────────────────────────────
        // IIR low-pass (α=0.3) smooths ADC noise without lagging badly
        float rawA = _readCurrentAmps();
        _filteredCurrentA = 0.3f * rawA + 0.7f * _filteredCurrentA;

        // ── Coulomb counting (trapezoidal) ────────────────────────────────
        // Average current over the elapsed window, integrate to Ah
        float avgCurrentA = (_lastCurrentA + _filteredCurrentA) * 0.5f;
        _lastCurrentA = _filteredCurrentA;

        float deltaAh = avgCurrentA * ((float)elapsed / 3600000.0f);
        eepromdata.coulombCountAh += deltaAh;

        float deltaSOC = (deltaAh / _packCapacityAh) * 100.0f;
        eepromdata.socPercent += deltaSOC;
        eepromdata.socPercent = _clampSOC(eepromdata.socPercent);

        // ── OCV blend correction at rest ──────────────────────────────────
        // When current is near zero the OCV lookup is valid.
        // Nudge CC result toward OCV slowly to prevent long-term drift
        // without jumping during load transients.
        if (fabsf(_filteredCurrentA) < SOC_ZERO_CURRENT_THRESHOLD) {
            float ocvSoc = _ocvToSOC(cellVoltage, cellTempC);
            eepromdata.socPercent = eepromdata.socPercent
                + SOC_OCV_REST_BLEND_RATE * (ocvSoc - eepromdata.socPercent);
            eepromdata.socPercent = _clampSOC(eepromdata.socPercent);
        }

    }
    else if (ExternalComms.isShuntDataFresh()) {
        // ── No internal sensor — use Venus shunt/SCS current ─────────────
        // Venus embeds the SmartShunt reading in every CMD_SEND_DATA request.
        // isShuntDataFresh() returns true only while staleness==0 AND the last
        // update arrived within SHUNT_MAX_AGE_MS (6 s / ~3 missed polls).
        float shuntA = ExternalComms.getShuntCurrentAmps();
        _filteredCurrentA = 0.3f * shuntA + 0.7f * _filteredCurrentA;

        float avgCurrentA = (_lastCurrentA + _filteredCurrentA) * 0.5f;
        _lastCurrentA = _filteredCurrentA;

        float deltaAh = avgCurrentA * ((float)elapsed / 3600000.0f);
        eepromdata.coulombCountAh += deltaAh;

        float deltaSOC = (deltaAh / _packCapacityAh) * 100.0f;
        eepromdata.socPercent += deltaSOC;
        eepromdata.socPercent = _clampSOC(eepromdata.socPercent);

        // OCV blend at rest
        if (fabsf(_filteredCurrentA) < SOC_ZERO_CURRENT_THRESHOLD) {
            float ocvSoc = _ocvToSOC(cellVoltage, cellTempC);
            eepromdata.socPercent = eepromdata.socPercent
                + SOC_OCV_REST_BLEND_RATE * (ocvSoc - eepromdata.socPercent);
            eepromdata.socPercent = _clampSOC(eepromdata.socPercent);
        }

    }
    else {
        // ── No usable current source (no sensor, or shunt data older than
        // SHUNT_MAX_AGE_MS / Venus staleness > 0) — pure OCV blend.
        float ocvSoc = _ocvToSOC(cellVoltage, cellTempC);
        eepromdata.socPercent = eepromdata.socPercent
            + SOC_OCV_REST_BLEND_RATE * (ocvSoc - eepromdata.socPercent);
        eepromdata.socPercent = _clampSOC(eepromdata.socPercent);
        _filteredCurrentA = 0.0f;
        _lastCurrentA = 0.0f;
    }

    // ── Hard resets at known endpoints ────────────────────────────────────
    // Require N consecutive ticks to guard against noise spikes.

    if (cellVoltage >= SOC_CELL_FULL_VOLTAGE) {
        if (++_fullConfirmTicks >= SOC_RESET_CONFIRM_TICKS) {
            eepromdata.socPercent = 100.0f;
            eepromdata.coulombCountAh = 0.0f;
            _fullConfirmTicks = 0;
        }
    }
    else {
        _fullConfirmTicks = 0;
    }

    if (cellVoltage <= SOC_CELL_EMPTY_VOLTAGE) {
        if (++_emptyConfirmTicks >= SOC_RESET_CONFIRM_TICKS) {
            eepromdata.socPercent = 0.0f;
            eepromdata.coulombCountAh = 0.0f;
            _emptyConfirmTicks = 0;
        }
    }
    else {
        _emptyConfirmTicks = 0;
    }
  
    if ((now - _lastSaveMs) >= 1200000UL) {   // 20 minutes save the SOC and Columb count to eeprom.
        EEPROMSettings::save();
        _lastSaveMs = now;
    }
    // ── Adaptive current sensor drift correction (OCV-guided) ─────────────
    // Only adjust when the pack has been at rest long enough for OCV to be trustworthy
    if (fabsf(_filteredCurrentA) < SOC_ZERO_CURRENT_THRESHOLD) {
        float ocvSoc = _ocvToSOC(cellVoltage, cellTempC);
        float socError = ocvSoc - eepromdata.socPercent;

        // Only trim if error is consistent and meaningful
        if (fabsf(socError) > 0.3f) {
            // Very slow adjustment (~0.0003 V per second at 1 Hz), clamped to ±0.1 V
            _currentSensorAdaptiveOffsetV = constrain(
                _currentSensorAdaptiveOffsetV + 0.0003f * socError, -0.1f, 0.1f);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
float SOCCalculator::getPackCurrentAmps() const {
    if (!eepromdata.currentSensorPresent) return 0.0f;
    return _filteredCurrentA;
}

// ─────────────────────────────────────────────────────────────────────────────
uint8_t SOCCalculator::getSOCByte() const {
    return (uint8_t)constrain((int)eepromdata.socPercent, 0, 100);
}

// ─────────────────────────────────────────────────────────────────────────────
// PRIVATE
// ─────────────────────────────────────────────────────────────────────────────

float SOCCalculator::_readCurrentAmps() const {
    int sum = 0;
    for (int n = 0; n < SOC_ADC_OVERSAMPLE; n++) {
        sum += analogRead(SOC_CURRENT_SENSOR_PIN);
    }
    // Apply zero offset captured at boot to cancel sensor + ADC bias
    float vSensor = _adcToVoltage(sum / SOC_ADC_OVERSAMPLE)
        - currentSensorZeroOffsetV
        - _currentSensorAdaptiveOffsetV;

    // Scale by measure range (110% of rated current per QN-C15S datasheet)
    float measureRangeAmps = (float)eepromdata.currentSensorRatedAmps * 1.1f;

    return ((vSensor - eepromdata.currentSensorVbias) / eepromdata.currentSensorVrange)
        * measureRangeAmps;
}

// ─────────────────────────────────────────────────────────────────────────────
float SOCCalculator::_adcToVoltage(int raw) const {
    // ADC_VREF_11DB and ADC_MAX_COUNT are ESP32 silicon constants
    // not user configurable - defined as private constexpr in header
    return ((float)raw / ADC_MAX_COUNT) * ADC_VREF_11DB;
}

// ─────────────────────────────────────────────────────────────────────────────
float SOCCalculator::_ocvToSOC(float cellVoltage, float tempC) const {

    float tClamped = constrain(tempC,
        SOC_LUT_TEMP_POINTS[0],
        SOC_LUT_TEMP_POINTS[SOC_LUT_TEMPS - 1]);

    // Find temperature bracket
    int tLo = 0;
    for (int t = 0; t < SOC_LUT_TEMPS - 1; t++) {
        if (tClamped <= SOC_LUT_TEMP_POINTS[t + 1]) {
            tLo = t;
            break;
        }
    }
    int   tHi = min(tLo + 1, SOC_LUT_TEMPS - 1);
    float tFrac = (SOC_LUT_TEMP_POINTS[tHi] > SOC_LUT_TEMP_POINTS[tLo])
        ? (tClamped - SOC_LUT_TEMP_POINTS[tLo])
        / (SOC_LUT_TEMP_POINTS[tHi] - SOC_LUT_TEMP_POINTS[tLo])
        : 0.0f;

    // Scan SOC rows for bracketing OCV, interpolate both axes
    for (int s = 0; s < SOC_LUT_POINTS - 1; s++) {
        float ocvLo = SOC_LUT_OCV[s][tLo]
            + tFrac * (SOC_LUT_OCV[s][tHi] - SOC_LUT_OCV[s][tLo]);
            float ocvHi = SOC_LUT_OCV[s + 1][tLo]
                + tFrac * (SOC_LUT_OCV[s + 1][tHi] - SOC_LUT_OCV[s + 1][tLo]);

                if (cellVoltage >= ocvLo && cellVoltage <= ocvHi) {
                    float sFrac = (ocvHi > ocvLo)
                        ? (cellVoltage - ocvLo) / (ocvHi - ocvLo)
                        : 0.0f;
                    return _clampSOC(SOC_LUT_SOC[s]
                        + sFrac * (SOC_LUT_SOC[s + 1] - SOC_LUT_SOC[s]));
                }
    }

    // Outside table range - clamp to nearest endpoint
    float ocvBottom = SOC_LUT_OCV[0][tLo]
        + tFrac * (SOC_LUT_OCV[0][tHi] - SOC_LUT_OCV[0][tLo]);
        float ocvTop = SOC_LUT_OCV[SOC_LUT_POINTS - 1][tLo]
            + tFrac * (SOC_LUT_OCV[SOC_LUT_POINTS - 1][tHi]
                - SOC_LUT_OCV[SOC_LUT_POINTS - 1][tLo]);

            if (cellVoltage <= ocvBottom) return  0.0f;
            if (cellVoltage >= ocvTop)    return 100.0f;
            return eepromdata.socPercent;
}

// ─────────────────────────────────────────────────────────────────────────────
float SOCCalculator::_clampSOC(float soc) const {
    return constrain(soc, 0.0f, 100.0f);
}
