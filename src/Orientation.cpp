#include "Orientation.h"

BNO055::BNO055(TwoWire& wire, u8 addr, i8 resetPin)
    : wire_(wire), addr_(addr), resetPin_(resetPin),
      _bno(-1, addr, &wire) {}

bool BNO055::begin() {
    if (resetPin_ >= 0) {
        pinMode(resetPin_, OUTPUT);
        digitalWrite(resetPin_, HIGH);
        // pulse nRESET active-low
        digitalWrite(resetPin_, LOW);
        delay(2);
        digitalWrite(resetPin_, HIGH);
        delay(30);
    }
    if (!_bno.begin(OPERATION_MODE_NDOF)) return false;
    _bno.setExtCrystalUse(BNO055Config::USE_EXTERNAL_CRYSTAL);
    return true;
}

void BNO055::suspend() {
    _bno.enterSuspendMode();
}

bool BNO055::getCalibOffsets(adafruit_bno055_offsets_t& out) {
    return _bno.getSensorOffsets(out);
}

void BNO055::setCalibOffsets(const adafruit_bno055_offsets_t& offsets) {
    _bno.setSensorOffsets(offsets);
}

u8 BNO055::calibSystemStatus() {
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    _bno.getCalibration(&sys, &gyro, &accel, &mag);
    return sys;
}

bool BNO055::readQuaternion(Quaternion& out) {
    imu::Quaternion q = _bno.getQuat();
    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return true;
}
