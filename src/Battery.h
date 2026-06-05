#pragma once

#include <Wire.h>
#include "config.h"
#include "shorthand.h"

class Battery {
public:
    bool begin(TwoWire& wire = Wire, bool initBus = false);

    f32  voltage();         // cell voltage, volts
    f32  percent();         // state of charge, 0..100 %
    f32  chargeRate();      // SOC slope, %/hr. positive = charging, negative = discharging
    bool charging(f32 deadband_pct_hr = 0.5f); // chargeRate() above a small deadband

    void reset();           // full power-on-reset via the command register
    void sleep(bool on);    // enter / leave analog sleep

private:
    TwoWire* _bus = nullptr;

    u16  read16(u8 reg);    // returns 0 on bus failure
    bool write16(u8 reg, u16 val);
};