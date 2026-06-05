#include "GPS.h"

GPS::GPS() : _serial(GpsConfig::UART_NUM) {}

void GPS::begin() {
    _serial.begin(GpsConfig::BAUD, SERIAL_8N1, ESP32S3Pinout::PIN_GPS_TX_ESP_RX, ESP32S3Pinout::PIN_GPS_RX_ESP_TX);
    _wmm.begin();
}

void GPS::update() {
    while (_serial.available() > 0) {
        _gps.encode(static_cast<char>(_serial.read()));
    }
    _recomputeDeclination();
}

bool GPS::hasFix() const {
    return _gps.location.isValid() && _gps.location.age() < GpsConfig::FIX_TIMEOUT_MS;
}

double GPS::latitude() const { return _gps.location.lat(); }
double GPS::longitude() const { return _gps.location.lng(); }
double GPS::altitudeMeters() const { return _gps.altitude.meters(); }

bool GPS::dateTimeValid() const {
    return _gps.date.isValid() && _gps.time.isValid();
}

u16 GPS::year() const { return _gps.date.year(); }
u8 GPS::month() const { return _gps.date.month(); }
u8 GPS::day() const { return _gps.date.day(); }
u8 GPS::hour() const { return _gps.time.hour(); }
u8 GPS::minute() const { return _gps.time.minute(); }
u8 GPS::second() const { return _gps.time.second(); }

bool GPS::declinationValid() const {
    return _declinationValid;
}
 
double GPS::declinationDegrees() const {
    return _declinationValid ? _declinationDeg : GpsConfig::DEFAULT_DECLINATION_DEG;
}

Quaternion GPS::declinationQuat() const {
    const float declRad = static_cast<float>(declinationDegrees() * MathConst::DEG2RAD);
    return yawQuat(-declRad);   // +Y of the result frame == true North
}
 
void GPS::_recomputeDeclination() {
    if (!hasFix() || !dateTimeValid()) {
        return;
    }
 
    // WMM_Tinier v1.0.3 epoch is 2025.0 - 2030.0; skip dates outside it.
    const u16 y = year();
    if (y < 2025 || y > 2030) {
        return;
    }
 
    const double lat = latitude();
    const double lng = longitude();
    const u32 now = millis();
 
    const bool first = !_declinationValid;
    const bool moved = TinyGPSPlus::distanceBetween(lat, lng, _lastCalcLat, _lastCalcLng) >= GpsConfig::RECOMPUTE_DECLINATION_DISTANCE;
    const bool stale = (now - _lastCalcMs) >= GpsConfig::RECOMPUTE_DECLINATION_TIME;
 
    if (!(first || moved || stale)) {
        return; // nothing changed enough to bother recomputing
    }
 
    // library wants a 2-digit year (2026 -> 26) and latitude before longitude.
    _declinationDeg = _wmm.magneticDeclination(static_cast<float>(lat), static_cast<float>(lng), static_cast<u8>(y - 2000), month(), day());
 
    _declinationValid = true;
    _lastCalcLat = lat;
    _lastCalcLng = lng;
    _lastCalcMs = now;
}

u8 GPS::satellites() const {
    return _gps.satellites.isValid() ? static_cast<u8>(_gps.satellites.value()) : 0;
}

u32 GPS::charsProcessed() const {
    return _gps.charsProcessed();
}

GpsFix GPS::read() const {
    GpsFix fix{};

    fix.valid = hasFix();
    if (fix.valid) {
        fix.latitude = latitude();
        fix.longitude = longitude();
        fix.altitudeM = altitudeMeters();
    }
    fix.satellites = satellites();

    return fix;
}