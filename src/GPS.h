#pragma once

#include <TinyGPS++.h>
#include <WMM_Tinier.h>
#include "config.h"
#include "geometry.h"
#include "shorthand.h"

struct GpsFix {
    bool   valid      = false;  // position is valid and not stale
    double latitude   = 0.0;   // degrees
    double longitude  = 0.0;   // degrees
    double altitudeM  = 0.0;   // meters above sea level
    u8     satellites = 0;     // satellites in use
};

class GPS {
public:
    GPS();

    void begin();
    void update();

    // UBX-CFG-RXM: Power Save Mode (about 11 mA cyclic 1 Hz). Module stays
    // UART-responsive; no reinit needed after waking.
    void sleep();

    bool declinationValid() const;    // true once a WMM value has been computed
    double declinationDegrees() const; // computed value, or config fallback
    Quaternion declinationQuat() const; // yaw(-decl): left-multiply to get true-north orientation

    u32  charsProcessed() const; // total chars from module; 0 = not wired
    bool isTalking() const;      // chars arrived within TALKING_TIMEOUT_MS

    GpsFix read() const;

private:
    HardwareSerial _serial;
    mutable TinyGPSPlus _gps;
    WMM_Tinier _wmm;

    bool   hasFix() const;
    double latitude() const;
    double longitude() const;
    double altitudeMeters() const;

    bool dateTimeValid() const;
    u16  year() const;
    u8   month() const;
    u8   day() const;
    u8   hour() const;
    u8   minute() const;
    u8   second() const;

    u8  satellites() const;

    void _recomputeDeclination();

    double _declinationDeg = GpsConfig::DEFAULT_DECLINATION_DEG;
    bool _declinationValid = false;
    double _lastCalcLat = 0.0;
    double _lastCalcLng = 0.0;
    u32 _lastCalcMs = 0;

    u32 _lastCharMs = 0;  // millis() of last received byte; 0 = never
};
