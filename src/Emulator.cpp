// Emulator.cpp - software stand-ins for BNO055, MAX17048, and GPS.
//
// Provides alternative definitions of every BNO055:: and Battery:: method so
// that [env:emulator] (which excludes Orientation.cpp and Battery.cpp via
// build_src_filter) can link without real I2C hardware present.
//
// Tunable defaults live in Emulator.h. Runtime tuning is done over Serial:
//
//   s<val>  heading sweep speed, deg/s   e.g.  s45
//   j<val>  heading jitter, deg          e.g.  j0
//   b<val>  battery SOC %, 0-100         e.g.  b50
//   d<val>  discharge rate, %/hr         e.g.  d20
//   g<val>  charge rate, %/hr            e.g.  g40
//   c       toggle charging on/off
//   ?       print current emulator state

#include "Emulator.h"
#include "Orientation.h"
#include "Battery.h"
#include "GPS.h"
#include <Arduino.h>
#include <math.h>

using namespace EmulatorConfig;

// -- Shared mutable state (this file only) --

namespace {

// GPS
uint32_t emu_gps_start = 0;

// BNO055
float emu_sweep_speed = BNO_SWEEP_DEG_PER_SEC;
float emu_jitter = BNO_JITTER_DEG;

// Battery
float emu_soc = BATT_START_PCT;
bool emu_charging = BATT_CHARGING_INIT;
float emu_drain_rate = BATT_DRAIN_RATE_PCT_HR;
float emu_charge_rate = BATT_CHARGE_RATE_PCT_HR;
uint32_t emu_batt_last = 0;

// Serial status cadence
uint32_t emu_last_print = 0;

// Minimal LCG so jitter doesn't depend on Arduino random() seeding.
uint32_t lcg_state = 0xDEADBEEFu;
float lcg_randf() {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (float)(int32_t)(lcg_state) / (float)0x80000000u;  // -1..1
}

void updateSoc() {
    const uint32_t now = millis();
    if (emu_batt_last == 0) { emu_batt_last = now; return; }
    const float dt_hr = (float)(now - emu_batt_last) / 3600000.0f;
    emu_batt_last = now;
    if (emu_charging) {
        emu_soc = fminf(100.0f, emu_soc + emu_charge_rate * dt_hr);
    } else {
        emu_soc = fmaxf(0.0f, emu_soc - emu_drain_rate * dt_hr);
    }
}

void printStatus() {
    Serial.printf(
        "[EMU] heading=%.1f deg/s  jitter=%.1f deg | SOC=%.1f%% %s  "
        "drain=%.1f  charge=%.1f %%/hr\n",
        emu_sweep_speed, emu_jitter,
        emu_soc, emu_charging ? "(CHGR)" : "(DISC)",
        emu_drain_rate, emu_charge_rate
    );
}

void processSerial() {
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) continue;

        const char k = cmd[0];
        const float v = (cmd.length() > 1) ? cmd.substring(1).toFloat() : 0.0f;

        if      (k == 's') { emu_sweep_speed = v;                Serial.printf("[EMU] sweep=%.1f deg/s\n", emu_sweep_speed); }
        else if (k == 'j') { emu_jitter = fmaxf(0.0f, v);       Serial.printf("[EMU] jitter=%.1f deg\n", emu_jitter); }
        else if (k == 'b') { emu_soc = fmaxf(0, fminf(100, v)); Serial.printf("[EMU] SOC=%.1f%%\n", emu_soc); }
        else if (k == 'd') { emu_drain_rate = fmaxf(0.0f, v);   Serial.printf("[EMU] drain=%.1f %%/hr\n", emu_drain_rate); }
        else if (k == 'g') { emu_charge_rate = fmaxf(0.0f, v);  Serial.printf("[EMU] chargeRate=%.1f %%/hr\n", emu_charge_rate); }
        else if (k == 'c') { emu_charging = !emu_charging;       Serial.printf("[EMU] charging=%s\n", emu_charging ? "ON" : "OFF"); }
        else if (k == '?') { printStatus(); }
    }

    if (STATUS_PRINT_MS > 0) {
        const uint32_t now = millis();
        if (now - emu_last_print >= STATUS_PRINT_MS) {
            emu_last_print = now;
            printStatus();
        }
    }
}

} // anonymous namespace

// -- BNO055 - emulated --

BNO055::BNO055(TwoWire& wire, u8 addr, i8 resetPin)
    : wire_(wire), addr_(addr), resetPin_(resetPin),
      _bno(-1, addr, &wire) {}

bool BNO055::begin() {
    Serial.println("[EMU] ---- BNO055 + MAX17048 emulator active ----");
    Serial.println("[EMU] Serial commands (newline-terminated):");
    Serial.println("[EMU]   s<val>  sweep speed deg/s  (e.g. s45)");
    Serial.println("[EMU]   j<val>  jitter deg         (e.g. j2.5)");
    Serial.println("[EMU]   b<val>  battery SOC 0-100  (e.g. b50)");
    Serial.println("[EMU]   d<val>  discharge %/hr     (e.g. d20)");
    Serial.println("[EMU]   g<val>  charge rate %/hr   (e.g. g40)");
    Serial.println("[EMU]   c       toggle charging");
    Serial.println("[EMU]   ?       print state");
    return true;
}

bool BNO055::readQuaternion(Quaternion& out) {
    processSerial();

    const float t = millis() / 1000.0f;

    // Heading sweeps at the configured rate with optional jitter.
    const float yawDeg = fmod(emu_sweep_speed * t, 360.0f);
    const float yawRad = (yawDeg + emu_jitter * lcg_randf()) * (float)(M_PI / 180.0);

    // Slower oscillating pitch and roll so the 3-D tilt rendering gets exercised.
    // about +/-17 deg pitch, about +/-20 deg roll
    const float pitchRad = 0.30f * sinf(t * 0.7f);
    const float rollRad  = 0.35f * sinf(t * 0.5f + 1.0f);

    // Compose: yaw (Z) * pitch (X) * roll (Y)
    const Quaternion qYaw   { cosf(yawRad   * 0.5f), 0.0f,                 0.0f,                 sinf(yawRad   * 0.5f) };
    const Quaternion qPitch { cosf(pitchRad * 0.5f), sinf(pitchRad * 0.5f), 0.0f,                0.0f                  };
    const Quaternion qRoll  { cosf(rollRad  * 0.5f), 0.0f,                 sinf(rollRad  * 0.5f), 0.0f                 };

    out = normalize(multiply(multiply(qYaw, qPitch), qRoll));
    return true;
}

// -- Battery - emulated --

bool Battery::begin(TwoWire& wire, bool) {
    _bus = &wire;
    emu_batt_last = millis();
    emu_soc = BATT_START_PCT;
    return true;
}

f32 Battery::voltage() {
    // 3.0 V at 0% -> 4.2 V at 100% (crude linear model).
    return 3.0f + emu_soc * 0.012f;
}

f32 Battery::percent() {
    updateSoc();
    return emu_soc;
}

f32 Battery::chargeRate() {
    return emu_charging ? emu_charge_rate : -emu_drain_rate;
}

void Battery::reset()          { emu_soc = 100.0f; }
void Battery::sleep(bool)      {}
u16  Battery::read16(u8)       { return 0; }
bool Battery::write16(u8, u16) { return true; }

// -- GPS - emulated --
//
// Fixed position: EmulatorConfig::GPS_LAT / GPS_LON (Phoenix, AZ).
// Declination is computed once via WMM_Tinier using the hardcoded date
// 2026-06-01, which matches the project epoch. hasFix() goes true after
// GPS_FIX_DELAY_MS so the UI's no-GPS state is briefly visible on boot.

GPS::GPS() : _serial(GpsConfig::UART_NUM) {}

void GPS::begin() {
    _wmm.begin();
    // Compute declination for the fixed position on 2026-06-01 (year byte = 26).
    _declinationDeg = static_cast<double>(
        _wmm.magneticDeclination(
            static_cast<float>(EmulatorConfig::GPS_LAT),
            static_cast<float>(EmulatorConfig::GPS_LON),
            26, 6, 1));
    _declinationValid = true;
    emu_gps_start = millis();

    Serial.printf("[EMU] GPS fix: %.6f, %.6f  decl=%.2f deg\n",
                  EmulatorConfig::GPS_LAT, EmulatorConfig::GPS_LON,
                  _declinationDeg);
}

void GPS::update() {}  // no UART to drain

bool GPS::hasFix() const {
    return (millis() - emu_gps_start) >= EmulatorConfig::GPS_FIX_DELAY_MS;
}

double GPS::latitude()       const { return EmulatorConfig::GPS_LAT; }
double GPS::longitude()      const { return EmulatorConfig::GPS_LON; }
double GPS::altitudeMeters() const { return EmulatorConfig::GPS_ALT_M; }

// Fixed date/time: 2026-06-01 12:00:00 UTC.
bool GPS::dateTimeValid() const { return true; }
u16  GPS::year()   const { return 2026; }
u8   GPS::month()  const { return 6; }
u8   GPS::day()    const { return 1; }
u8   GPS::hour()   const { return 12; }
u8   GPS::minute() const { return 0; }
u8   GPS::second() const { return 0; }

bool   GPS::declinationValid()   const { return _declinationValid; }
double GPS::declinationDegrees() const {
    return _declinationValid ? _declinationDeg : GpsConfig::DEFAULT_DECLINATION_DEG;
}

Quaternion GPS::declinationQuat() const {
    const float declRad = static_cast<float>(declinationDegrees() * MathConst::DEG2RAD);
    return yawQuat(-declRad);
}

u8  GPS::satellites()     const { return hasFix() ? EmulatorConfig::GPS_SATELLITES : 0; }
u32 GPS::charsProcessed() const { return 0; }

void GPS::_recomputeDeclination() {}  // computed once in begin()

GpsFix GPS::read() const {
    GpsFix fix{};
    fix.valid = hasFix();
    if (fix.valid) {
        fix.latitude  = EmulatorConfig::GPS_LAT;
        fix.longitude = EmulatorConfig::GPS_LON;
        fix.altitudeM = EmulatorConfig::GPS_ALT_M;
    }
    fix.satellites = satellites();
    return fix;
}
