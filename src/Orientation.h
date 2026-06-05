#pragma once

#include <Wire.h>
#include "config.h"
#include "geometry.h"   // shared Quaternion (w,x,y,z) used across the pipeline
#include "shorthand.h"

enum class OpMode : u8 {
    Config = 0,
    ImuPlus = 1,
    Compass = 2,
    Ndof = 3
};

enum class Accuracy : u8 {
    Unreliable = 0,
    Low = 1,
    Medium = 2,
    High = 3
};

struct EulerAngles { float heading, roll, pitch; };
struct CalibStatus { u8 sys, gyro, accel, mag; };
struct CalibProfile { u8 bytes[22]; bool valid = false; };

class BNO055 {
public:
    // resetPin / intPin default to -1; pass GPIOs from config.h
    explicit BNO055(TwoWire& wire = Wire, u8 addr = 0x28,
                    i8 resetPin = -1, i8 intPin = -1);

    bool begin(OpMode mode = OpMode::Ndof);
    bool setMode(OpMode mode);

    // pulse the hardware nRESET line (no-op if no reset pin given)
    void hardwareReset();

    bool readEuler(EulerAngles& out);
    bool readQuaternion(Quaternion& out);   // now fills geometry.h's Quaternion

    CalibStatus readCalibStatus();
    Accuracy accuracy();
    bool readProfile(CalibProfile& out);
    bool writeProfile (const CalibProfile& in);

    // Any-motion interrupt on the INT pin. Operates on raw accel data, so it
    // works in fusion modes. The BNO055 has no fusion data-ready interrupt;
    // keep polling readEuler/readQuaternion for orientation.
    bool enableAnyMotionInterrupt(u8 threshold = 20, u8 durationSamples = 1);
    bool disableInterrupts();
    bool clearInterrupt();

    i8 interruptPin() const { return intPin_; }

private:
    bool writeReg(u8 reg, u8 val);
    bool readBytes(u8 reg, u8* buf, size_t n);
    bool setPage(u8 page);
    bool waitForChip();

    TwoWire& wire_;
    u8 addr_;
    i8 resetPin_;
    i8 intPin_;
    OpMode mode_ = OpMode::Config;
};