#pragma once

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "config.h"
#include "geometry.h"
#include "shorthand.h"

class BNO055 {
public:
    explicit BNO055(TwoWire& wire = Wire, u8 addr = 0x28, i8 resetPin = -1);

    bool begin();
    void suspend();
    bool readQuaternion(Quaternion& out);

    // Returns false if not fully calibrated; call setCalibOffsets() after begin().
    bool getCalibOffsets(adafruit_bno055_offsets_t& out);
    void setCalibOffsets(const adafruit_bno055_offsets_t& offsets);
    u8   calibSystemStatus();  // 0..3, 3 = fully calibrated

private:
    TwoWire& wire_;
    u8 addr_;
    i8 resetPin_;
    Adafruit_BNO055 _bno;
};
