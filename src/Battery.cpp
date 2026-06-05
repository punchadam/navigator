#include <Arduino.h>
#include "Battery.h"

using namespace ESP32S3Pinout;

bool Battery::begin(TwoWire& wire, bool initBus) {
    if (initBus) {
        wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    }
    if (!_gauge.begin(&wire)) {
        if (DebugConfig::BATTERY) Serial.println("[batt] MAX17048 not found");
        return false;
    }
    _wire = &wire;
    _gauge.setHibernationThreshold(0.0f); // disable hibernation; CRATE updates every 250 ms
    writeRCOMP(BatteryConfig::RCOMP);
    if (DebugConfig::BATTERY) Serial.println("[batt] MAX17048 found");
    return true;
}

void Battery::writeRCOMP(u8 rcomp) {
    // CONFIG reg (0x0C) is 2 bytes MSB-first: [RCOMP | flags].
    // The Adafruit library has no setter for RCOMP, so read-modify-write directly.
    constexpr u8 ADDR       = 0x36;
    constexpr u8 CONFIG_REG = 0x0C;

    _wire->beginTransmission(ADDR);
    _wire->write(CONFIG_REG);
    _wire->endTransmission(false);
    _wire->requestFrom(ADDR, (u8)2);
    _wire->read();             // discard current RCOMP
    u8 flags = _wire->read(); // preserve sleep/alert bits

    _wire->beginTransmission(ADDR);
    _wire->write(CONFIG_REG);
    _wire->write(rcomp);
    _wire->write(flags);
    _wire->endTransmission();
}

void Battery::trackVoltage() {
    _vLast = _gauge.cellVoltage();
    _vBuf[_vHead] = _vLast;
    _vHead = (_vHead + 1) % V_HIST;
    if (_vCount < V_HIST) _vCount++;
}

bool Battery::voltageRising(f32 half_mean_threshold) const {
    if (_vCount < V_HIST) return false;

    // Gate 1 - half-mean: fast slope-direction detection.
    // Newer half mean vs older half mean. A slope reversal (discharge -> charge)
    // produces about 0.75 mV contrast even when net current is tiny.
    constexpr u8 HALF = V_HIST / 2;
    f32 sumOld = 0.0f, sumNew = 0.0f;
    for (u8 i = 0; i < HALF; i++) {
        sumOld += _vBuf[(_vHead + i)        % V_HIST];
        sumNew += _vBuf[(_vHead + HALF + i) % V_HIST];
    }
    if ((sumNew - sumOld) < (half_mean_threshold * HALF)) return false;

    // Gate 2 - majority vote: stability filter.
    // Most consecutive pairs must be flat or rising (>= tolerates ADC quantisation
    // where a real upward step smaller than 1 LSB appears as zero movement).
    // A single transient load spike causes at most 2 falling pairs; the majority
    // absorbs that without flickering.
    u8 notFalling = 0;
    for (u8 i = 0; i < V_HIST - 1; i++) {
        u8 a = (_vHead + i)     % V_HIST;
        u8 b = (_vHead + i + 1) % V_HIST;
        if (_vBuf[b] >= _vBuf[a]) notFalling++;
    }
    return notFalling >= (V_HIST / 2);
}

bool Battery::didReset() {
    // STATUS register (0x1A): 16-bit, MSB first.
    // Bit 0 of the low byte is RI (Reset Indicator), set on power-on reset.
    // Must be cleared by writing 0 to it; the chip does not self-clear.
    constexpr u8 ADDR       = 0x36;
    constexpr u8 STATUS_REG = 0x1A;

    _wire->beginTransmission(ADDR);
    _wire->write(STATUS_REG);
    _wire->endTransmission(false);
    _wire->requestFrom(ADDR, (u8)2);
    const u8 hi = _wire->read();
    const u8 lo = _wire->read();

    if (!(lo & 0x01)) return false;

    _wire->beginTransmission(ADDR);
    _wire->write(STATUS_REG);
    _wire->write(hi);
    _wire->write(lo & ~0x01u);
    _wire->endTransmission();

    return true;
}

void Battery::writeSoc(f32 pct) {
    // SOC register (0x04): 16-bit, MSB first, in units of 1/256 %.
    // Writing this seeds the ModelGauge algorithm's starting SOC so it
    // continues tracking from a known-good value instead of a cold OCV guess.
    constexpr u8 ADDR    = 0x36;
    constexpr u8 SOC_REG = 0x04;

    pct = pct < 0.0f ? 0.0f : (pct > 100.0f ? 100.0f : pct);
    const u16 raw = (u16)(pct * 256.0f);

    _wire->beginTransmission(ADDR);
    _wire->write(SOC_REG);
    _wire->write((u8)(raw >> 8));
    _wire->write((u8)(raw & 0xFF));
    _wire->endTransmission();
}

f32 Battery::voltage()    { return _vLast;              }
f32 Battery::percent()    { return _gauge.cellPercent(); }
f32 Battery::chargeRate() { return _gauge.chargeRate();  }
