#pragma once

#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include "config.h"
#include "shorthand.h"

class Battery {
public:
    bool begin(TwoWire& wire = Wire, bool initBus = false);

    // Call once per SOC poll. Reads + caches voltage, updates trend buffer.
    void trackVoltage();

    f32  voltage();    // cached from last trackVoltage()
    f32  percent();    // state of charge, 0..100 %
    f32  chargeRate(); // SOC slope %/hr: positive = charging, negative = discharging

    // True if voltage rose >= min_rise_v over the trend window.
    // Returns false until the window is full.
    bool voltageRising(f32 min_rise_v = BatteryConfig::CHARGE_VTREND_V) const;

    // Returns true if the chip's STATUS register has the RI (Reset Indicator)
    // bit set, meaning the chip lost power and re-estimated SOC from OCV.
    // Clears the bit as a side effect - call once, immediately after begin().
    bool didReset();

    // Write a known SOC directly to the chip's SOC register so the ModelGauge
    // algorithm starts from the right place instead of a cold OCV estimate.
    void writeSoc(f32 pct);

private:
    Adafruit_MAX17048 _gauge;
    TwoWire*          _wire  = nullptr;

    void writeRCOMP(u8 rcomp);

    static constexpr u8 V_HIST = BatteryConfig::CHARGE_VTREND_SAMPLES;
    f32 _vBuf[V_HIST] = {};
    u8  _vHead  = 0;
    u8  _vCount = 0;
    f32 _vLast  = 0.0f;
};
