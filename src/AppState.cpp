// AppState.cpp
// UI controller internals
//
// Animation model: each timed element is a normalized 0..1 envelope computed
// from millis() deltas so the renderer never sees a clock. The bottom label
// has a three-part cycle: rise+fade-in, hold, fade-out that is collapsed into
// two outputs for the renderer, alpha (brightness) and rise (how far below the
// bezel it still is). Idk why I spent so much time on this for what's really
// just a stupid fun project, but it looks spectacular so idc

#include "AppState.h"
#include "config.h"
#include "Color.h"

#include <cstdio>
#include <cstring>
#include <math.h>

namespace {

// accent and all animation timings live in config.h
using namespace UiConfig;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float smooth(float t) { t = clampf(t, 0.f, 1.f); return t * t * (3.f - 2.f * t); }

} // namespace

void AppState::begin(u32 now) {
    // A is locked onto the fuckin bar hell yeah go to rips it's awesome
    // there's a pool table and karaoke and church music IPA on tap
    // B is default to store 90 but u can overwrite it at runtime by
    // long-pressing three times.
    _wp[0] = { "WAYPOINT A", "A", 36.1314, -95.9372, color::pack(  0, 255, 180), false };
    _wp[1] = { "WAYPOINT B", "B", 36.0578, -95.7910, color::pack(255, 170,  40), true  };

    _mode = Mode::Compass;
    _label = "COMPASS";
    _labelAt = now; // greet with a pop on boot
}

// -- mode + butt stuff (lmfao I wrote butt instead of button) --
void AppState::cycleMode(u32 now) {
    _mode = static_cast<Mode>((static_cast<u8>(_mode) + 1) % static_cast<u8>(Mode::COUNT));
    // leaving B abandons any partial or accidental overwrite of the coords
    _setArmed = false;
    _setCount = 0;
    flash(modeName(_mode), now);
}

void AppState::onLongPress(u32 now, bool fixValid, double lat, double lon) {
    switch (_mode) {
        case Mode::Compass:
            _metric = !_metric;
            flash(_metric ? "METRIC" : "IMPERIAL", now);
            break;

        case Mode::WaypointA:
            flash("A LOCKED", now);
            break;

        case Mode::WaypointB: {
            if (!fixValid) {    // can't capture without a position
                _setArmed = false; _setCount = 0;
                flash("NO FIX", now);
                break;
            }
            // restart the count if the previous hold lapsed
            if (!_setArmed || (now - _setLastAt) > SET_WINDOW) {
                _setArmed = true;
                _setCount = 1;
            } else {
                ++_setCount;
            }
            _setLastAt = now;

            if (_setCount >= 3) {   // commit
                _wp[1].lat = lat;
                _wp[1].lon = lon;
                _setArmed = false;
                _setCount = 0;
                flash("TARGET SET", now);
            }
            // 1/3 and 2/3 show as the on-screen counter
            break;
        }

        default: break;
    }
}

const AppState::Waypoint& AppState::waypoint(Mode m) const {
    return (m == Mode::WaypointB) ? _wp[1] : _wp[0];
}

const AppState::Waypoint* AppState::activeWaypoint() const {
    switch (_mode) {
        case Mode::WaypointA: return &_wp[0];
        case Mode::WaypointB: return &_wp[1];
        default: return nullptr;
    }
}

const char* AppState::modeName(Mode m) const {
    switch (m) {
        case Mode::WaypointA: return _wp[0].name;
        case Mode::WaypointB: return _wp[1].name;
        default: return "COMPASS";
    }
}

// -- per-frame update --
void AppState::update(const Inputs& in) {
    _in = in;

    // reset a stalled capture so the counter doesn't linger forever
    if (_setArmed && (in.now - _setLastAt) > SET_WINDOW) {
        _setArmed = false;
        _setCount = 0;
    }

    // charging latch, enter the moment the gauge reports charging, hold through
    // the full-charge plateau, and only leave once the pack is clearly discharging again
    if (in.haveBattery) {
        if (!_chargingScreen) {
            if (in.charging) _chargingScreen = true;
        } else if (in.chargeRatePctHr < -0.5f) {
            _chargingScreen = false;
        }
    } else {
        _chargingScreen = false;
    }
    _full = _chargingScreen && in.battPct >= 99.0f;

    if (in.haveBearing) formatDistance(in.distanceM);
    if (_chargingScreen) formatCharge();
}

// ---- render handoff ---------------------------------------------------------
void AppState::draw(Display& disp, const Quaternion& qShow,
                    const Marker* markers, size_t markerCount) {
    if (_chargingScreen) {
        ChargeState cs;
        cs.pct = (_in.battPct < 0.f) ? 0.f : _in.battPct;
        cs.full = _full;
        cs.line1 = _full ? "CHARGED" : "CHARGING";
        cs.line2 = _chgPct;
        cs.line3 = _chgEta;
        cs.accent = ACCENT;
        disp.renderCharging(cs);
        return;
    }

    // needle in compass mode, colored arrow when waypoint
    if (const Waypoint* wp = activeWaypoint()) disp.showArrow(wp->color);
    else                                       disp.showNeedle();

    HudState h{};
    h.label        = _label;
    h.labelAlpha   = labelAlpha(_in.now);
    h.labelRise    = labelRise(_in.now);
    h.showDistance = _in.haveBearing;
    h.distance     = _distBuf;
    h.noFix        = (activeWaypoint() != nullptr) && !_in.fixValid;
    h.sats         = _in.sats;
    h.fixValid     = _in.fixValid;
    h.calib        = _in.calib;
    h.battPct      = _in.haveBattery ? _in.battPct : -1.0f;
    h.pressSplash  = pressSplash(_in.now);
    h.setArmed     = _setArmed;
    h.setCount     = _setCount;
    h.accent       = ACCENT;

    disp.render(qShow, markers, markerCount, h);
}

// -- animaysh envelopes --
float AppState::labelAlpha(u32 now) const {
    const u32 t = now - _labelAt;
    if (t >= LBL_LIFE) return 0.0f;
    if (t < LBL_IN) return smooth(float(t) / LBL_IN);
    if (t < LBL_IN + LBL_HOLD) return 1.0f;
    return 1.0f - smooth(float(t - LBL_IN - LBL_HOLD) / LBL_OUT);
}

float AppState::labelRise(u32 now) const {
    const u32 t = now - _labelAt;
    if (t >= LBL_IN) return 0.0f;               // settled after the entrance
    return 1.0f - smooth(float(t) / LBL_IN);    // 1 (below bezel) -> 0 (in place)
}

float AppState::pressSplash(u32 now) const {
    const u32 t = now - _pressAt;
    if (_pressAt == 0 || t >= SPLASH_MS) return 0.0f;
    return 1.0f - float(t) / SPLASH_MS;         // linear decay reads as a quick flash
}

// -- string formatting --
void AppState::formatDistance(double m) {
    if (_metric) {
        if (m < 1000.0)        snprintf(_distBuf, sizeof(_distBuf), "%d M",   (int)(m + 0.5));
        else if (m < 100000.0) snprintf(_distBuf, sizeof(_distBuf), "%.2f KM", m / 1000.0);
        else                   snprintf(_distBuf, sizeof(_distBuf), "%.0f KM", m / 1000.0);
    } else {
        const double miles = m / 1609.344;
        if (miles < 0.1)       snprintf(_distBuf, sizeof(_distBuf), "%d FT",  (int)(m * 3.28084 + 0.5));
        else if (miles < 100.0)snprintf(_distBuf, sizeof(_distBuf), "%.2f MI", miles);
        else                   snprintf(_distBuf, sizeof(_distBuf), "%.0f MI", miles);
    }
}

void AppState::formatCharge() {
    const int pct = (int)((_in.battPct < 0.f ? 0.f : _in.battPct) + 0.5f);
    snprintf(_chgPct, sizeof(_chgPct), "%d%%", pct);

    if (_full) {
        snprintf(_chgEta, sizeof(_chgEta), "READY TO GO");
        return;
    }
    const float rate = _in.chargeRatePctHr;
    if (rate <= 0.2f) { // too slow / unknown to estimate
        snprintf(_chgEta, sizeof(_chgEta), "ESTIMATING");
        return;
    }
    const float hrs = (100.0f - _in.battPct) / rate;
    if (hrs < 1.0f) snprintf(_chgEta, sizeof(_chgEta), "FULL IN %d MIN", (int)(hrs * 60.0f + 0.5f));
    else            snprintf(_chgEta, sizeof(_chgEta), "FULL IN %.1f H", hrs);
}