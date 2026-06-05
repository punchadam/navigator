#include "Orientation.h"

// All the register addresses, bit masks, and timings used below live in
// namespace BNO055Config (config.h). Pull them in for this file so the code
// reads REG_CHIP_ID rather than BNO055Config::REG_CHIP_ID on every line.
using namespace BNO055Config;


BNO055::BNO055(TwoWire& wire, u8 addr, i8 resetPin, i8 intPin)
    : wire_(wire), addr_(addr), resetPin_(resetPin), intPin_(intPin) {}

void BNO055::hardwareReset() {
    if (resetPin_ < 0) return;
    pinMode(resetPin_, OUTPUT);
    digitalWrite(resetPin_, LOW);   // nRESET is active low
    delay(2);                       // datasheet min pulse is tiny; be generous
    digitalWrite(resetPin_, HIGH);
    delay(30);                      // let it begin booting before poll
}

bool BNO055::waitForChip() {
    const u32 deadline = millis() + BOOT_TIMEOUT_MS;
    u8 id = 0;
    while (millis() < deadline) {
        if (readBytes(REG_CHIP_ID, &id, 1) && id == CHIP_ID_VALUE) return true;
        delay(10);
    }
    return false;
}

bool BNO055::begin(OpMode mode) {
    // park the reset line deasserted, then pulse it if a reset pin is wired
    if (resetPin_ >= 0) {
        pinMode(resetPin_, OUTPUT);
        digitalWrite(resetPin_, HIGH);
        hardwareReset();
    }
    if (intPin_ >= 0) pinMode(intPin_, INPUT);

    setPage(0);

    // wait for the chip to answer after power-on / reset
    if (!waitForChip()) return false;

    // with no hardware reset line, fall back to a software reset
    if (resetPin_ < 0) {
        if (!writeReg(REG_OPR_MODE, static_cast<u8>(OpMode::Config))) return false;
        delay(DELAY_TO_CONFIG);
        writeReg(REG_SYS_TRIGGER, SYS_TRIGGER_RESET);
        delay(30);
        if (!waitForChip()) return false;
    }

    // after reset the chip is in CONFIG on page 0 by default.
    setPage(0);

    // normal power, external crystal, explicit units
    if (!writeReg(REG_PWR_MODE, PWR_MODE_NORMAL)) return false;
    delay(10);

    if (USE_EXTERNAL_CRYSTAL) {
        if (!writeReg(REG_SYS_TRIGGER, SYS_TRIGGER_EXTCLK)) return false;
        delay(10);
    }

    if (!writeReg(REG_UNIT_SEL, UNIT_SEL_DEFAULTS)) return false;

    // enter the requested fusion mode
    return setMode(mode);
}

// mode switching
bool BNO055::setMode(OpMode mode) {
    setPage(0);

    // pass through config
    if (!writeReg(REG_OPR_MODE, static_cast<u8>(OpMode::Config))) return false;
    delay(DELAY_TO_CONFIG);

    // set mode
    if (mode != OpMode::Config) {
        if (!writeReg(REG_OPR_MODE, static_cast<u8>(mode))) return false;
        delay(DELAY_FROM_CONFIG);
    }

    mode_ = mode;
    return true;
}

// orientation reads
bool BNO055::readEuler(EulerAngles& out) {
    setPage(0);
    u8 buf[6];
    if (!readBytes(REG_EULER_H_LSB, buf, 6)) return false;

    const i16 h = static_cast<i16>(buf[0] | (buf[1] << 8));
    const i16 r = static_cast<i16>(buf[2] | (buf[3] << 8));
    const i16 p = static_cast<i16>(buf[4] | (buf[5] << 8));

    out.heading = h / 16.0f;    // 16 LSB per degree
    out.roll    = r / 16.0f;
    out.pitch   = p / 16.0f;
    return true;
}

bool BNO055::readQuaternion(Quaternion& out) {
    setPage(0);
    u8 buf[8];
    if (!readBytes(REG_QUAT_W_LSB, buf, 8)) return false;

    const i16 w = static_cast<i16>(buf[0] | (buf[1] << 8));
    const i16 x = static_cast<i16>(buf[2] | (buf[3] << 8));
    const i16 y = static_cast<i16>(buf[4] | (buf[5] << 8));
    const i16 z = static_cast<i16>(buf[6] | (buf[7] << 8));

    constexpr float kScale = 1.0f / (1 << 14);  // 2^14 LSB == 1.0
    out.w = w * kScale;
    out.x = x * kScale;
    out.y = y * kScale;
    out.z = z * kScale;
    return true;
}

// calibraysh
CalibStatus BNO055::readCalibStatus() {
    setPage(0);
    u8 buf = 0;
    // on a failed read, this returns all 0, which keeps the coach asking
    // instead of quietly reporting false calibration status
    readBytes(REG_CALIB_STAT, &buf, 1);

    CalibStatus s;
    s.sys   = (buf >> 6) & 0x03;
    s.gyro  = (buf >> 4) & 0x03;
    s.accel = (buf >> 2) & 0x03;
    s.mag   =  buf       & 0x03;
    return s;
}

// exposes a single accuracy number based on s.sys (for UI)
Accuracy BNO055::accuracy() {
    CalibStatus s = readCalibStatus();
    return static_cast<Accuracy>(s.sys);
}

bool BNO055::readProfile(CalibProfile& out) {
    // remember where to return to
    const OpMode previous = mode_;

    if (!setMode(OpMode::Config)) return false;
    setPage(0);

    const bool ok = readBytes(REG_ACCEL_OFFSET, out.bytes, sizeof(out.bytes));
    out.valid = ok;

    // restore regardless of read result
    setMode(previous);
    return ok;
}

bool BNO055::writeProfile(const CalibProfile& in) {
    if (!in.valid) return false;

    const OpMode previous = mode_;

    if (!setMode(OpMode::Config)) return false;
    setPage(0);

    // single auto-incrementing burst write starting at the first offset register
    wire_.beginTransmission(addr_);
    wire_.write(REG_ACCEL_OFFSET);
    wire_.write(in.bytes, sizeof(in.bytes));
    const bool ok = (wire_.endTransmission() == 0);

    setMode(previous);
    return ok;
}

// interrupt config (page 1 registers, configured from CONFIG mode)
bool BNO055::enableAnyMotionInterrupt(u8 threshold, u8 durationSamples) {
    const OpMode previous = mode_;
    if (!setMode(OpMode::Config)) return false;

    setPage(1);

    bool ok = true;
    ok = writeReg(REG_ACC_AM_THRES, threshold) && ok;

    // duration is 1..4 samples, stored as 0..3 in bits[1:0]; enable X/Y/Z axes
    const u8 dur = static_cast<u8>((durationSamples ? durationSamples - 1 : 0) & 0x03);
    ok = writeReg(REG_ACC_INT_SETTINGS, static_cast<u8>(dur | ACC_INT_AM_AXES)) && ok;

    // enable generation and route to the INT pin (read-modify-write)
    u8 en = 0, msk = 0;
    readBytes(REG_INT_EN, &en, 1);
    readBytes(REG_INT_MASK, &msk, 1);
    ok = writeReg(REG_INT_EN,   static_cast<u8>(en  | INT_ACC_AM)) && ok;
    ok = writeReg(REG_INT_MASK, static_cast<u8>(msk | INT_ACC_AM)) && ok;

    setPage(0);
    if (!setMode(previous)) return false;
    return ok;
}

bool BNO055::disableInterrupts() {
    const OpMode previous = mode_;
    if (!setMode(OpMode::Config)) return false;

    setPage(1);
    const bool a = writeReg(REG_INT_EN, 0x00);
    const bool b = writeReg(REG_INT_MASK, 0x00);
    setPage(0);

    if (!setMode(previous)) return false;
    return a && b;
}

bool BNO055::clearInterrupt() {
    setPage(0);
    // RST_INT must keep CLK_SEL set or we silently fall back to the internal clock
    const u8 v = static_cast<u8>((USE_EXTERNAL_CRYSTAL ? SYS_TRIGGER_EXTCLK : 0x00)
                                 | SYS_TRIGGER_RST_INT);
    return writeReg(REG_SYS_TRIGGER, v);
}

// transport helpers
bool BNO055::writeReg(u8 reg, u8 val) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(val);
    return wire_.endTransmission() == 0;
}

bool BNO055::readBytes(u8 reg, u8* buf, size_t n) {
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;    // repeated start, keep bus

    const size_t got = wire_.requestFrom(static_cast<int>(addr_), static_cast<int>(n));
    if (got != n) return false;

    for (size_t i = 0; i < n; ++i) buf[i] = wire_.read();
    return true;
}

bool BNO055::setPage(u8 page) {
    return writeReg(REG_PAGE_ID, page);
}