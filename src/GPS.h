#pragma once

#include <TinyGPS++.h>
#include <WMM_Tinier.h>
#include "config.h"
#include "geometry.h"
#include "shorthand.h"

struct GpsFix {
    bool valid = false; // position is valid AND not stale
    double latitude = 0.0;   // degrees
    double longitude = 0.0;   // degrees
    double altitudeM = 0.0;   // meters above sea level
    u8 satellites = 0;          // satellites in use
};

class GPS {
public:
    GPS();
    WMM_Tinier _wmm;

    void begin();
    void update();

    bool hasFix() const;  // valid, non-stale position fix
    double latitude() const;  // degrees
    double longitude() const;  // degrees
    double altitudeMeters() const;  // meters

    bool dateTimeValid() const;
    u16 year() const;
    u8 month() const;
    u8 day() const;
    u8 hour() const; // UTC
    u8 minute() const; // UTC
    u8 second() const; // UTC

    bool declinationValid() const;  // true if a real WMM value is cached
    double declinationDegrees() const;  // cached value, else config fallback

    // Magnetic -> true correction as a world-up yaw quaternion, i.e. yaw(-decl).
    // Left-multiply the device's body->world (magnetic) orientation by this to
    // re-reference it to TRUE north:  qTrue = declinationQuat() * qDevice.
    Quaternion declinationQuat() const;

    u8 satellites() const;
    u32 charsProcessed() const; // >0 means the module is wired & talking

    // everything
    GpsFix read() const;

private:
    HardwareSerial _serial;
    mutable TinyGPSPlus _gps;

    void _recomputeDeclination();   // gated; called from update()
    double _decimalYear() const;    // GPS date -> fractional year for WMM

    double _declinationDeg = GpsConfig::DEFAULT_DECLINATION_DEG;

    bool _declinationValid = false;
    double _lastCalcLat = 0.0;      // position of last computation
    double _lastCalcLng = 0.0;
    u32 _lastCalcMs = 0;       // millis() of last computation
};