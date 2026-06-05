#include "Battery.h"

using namespace ESP32S3Pinout;  // PIN_I2C_SDA / PIN_I2C_SCL
using namespace MAX10748Config; // MAX17048_ADDR, REG_*, *_LSB_*, CMD_POR, etc.

bool Battery::begin(TwoWire& wire, bool initBus) {
    _bus = &wire;
    if (initBus) {
        _bus->begin(PIN_I2C_SDA, PIN_I2C_SCL);
    }

    // ping
    _bus->beginTransmission(MAX17048_ADDR);
    if (_bus->endTransmission() != 0) {
        _bus = nullptr;
        return false;
    }
    return true;
}

f32 Battery::voltage() {
    return read16(REG_VCELL) * VCELL_LSB_V;
}

f32 Battery::percent() {
    return read16(REG_SOC) * SOC_LSB_PCT;
}

f32 Battery::chargeRate() {
    // CRATE is signed: sign carries charge (+) vs discharge (-).
    return (i16)read16(REG_CRATE) * CRATE_LSB_PCTHR;
}

bool Battery::charging(f32 deadband_pct_hr) {
    return chargeRate() > deadband_pct_hr;
}

void Battery::reset() {
    // After this the gauge needs a moment to compute a first valid SOC.
    write16(REG_CMD, CMD_POR);
}

void Battery::sleep(bool on) {
    // EnSleep in MODE must be set before the SLEEP bit in CONFIG takes effect.
    if (on) {
        write16(REG_MODE, read16(REG_MODE) | MODE_ENSLEEP);
    }
    u16 cfg = read16(REG_CONFIG);
    if (on) cfg |= CFG_SLEEP;
    else    cfg &= ~CFG_SLEEP;
    write16(REG_CONFIG, cfg);
}

// private I2C helpers

u16 Battery::read16(u8 reg) {
    if (!_bus) return 0;
    _bus->beginTransmission(MAX17048_ADDR);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return 0; // repeated start, keep bus
    if (_bus->requestFrom((int)MAX17048_ADDR, (int)2) != 2) return 0;
    u16 hi = _bus->read();
    u16 lo = _bus->read();
    return (u16)((hi << 8) | lo);
}

bool Battery::write16(u8 reg, u16 val) {
    if (!_bus) return false;
    _bus->beginTransmission(MAX17048_ADDR);
    _bus->write(reg);
    _bus->write((u8)(val >> 8));
    _bus->write((u8)(val & 0xFF));
    return _bus->endTransmission() == 0;
}