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
    // B is default to tjs but u can overwrite it at runtime by
    // long-pressing three times.
    _wp[0] = { "???", "?", WaypointDefaults::A_LAT, WaypointDefaults::A_LON, color::pack(  0, 255, 180), false };
    _wp[1] = { "YOUR SPOT", "Y", WaypointDefaults::B_LAT, WaypointDefaults::B_LON, color::pack(255, 170,  40), true  };

    _mode = Mode::Compass;
    _label = "COMPASS";
    _labelAt = now; // greet with a pop on boot
}

// -- mode + butt stuff (lmfao I wrote butt instead of button) --
void AppState::cycleMode(u32 now) {
    if (_pendingCycle) return;   // ignore extra presses while transition is in flight
    _pendingCycle    = true;
    _cycleAt         = now + LBL_IN;   // commit when overlay is at full opacity
    _longPressFlash  = false;
    flash(modeName(_mode), now);       // start overlay showing the current mode name
}

void AppState::onLongPress(u32 now, bool fixValid, double lat, double lon) {
    _longPressRecord = { _mode, _setArmed, _setCount, _wp[1].lat, _wp[1].lon };
    _longPressFlash = true;   // battery ring stays hidden for hold-triggered labels
    switch (_mode) {
        case Mode::Compass:
            break;

        case Mode::WaypointA:
            flash("A LOCKED", now);
            break;

        case Mode::WaypointB: {
            if (!fixValid) {    // can't capture without a position
                _setArmed = false; _setCount = 0;
                flash("NO GPS", now);
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

void AppState::undoLongPress(u32 now) {
    switch (_longPressRecord.mode) {
        case Mode::WaypointB:
            _setArmed   = _longPressRecord.setArmed;
            _setCount   = _longPressRecord.setCount;
            _wp[1].lat  = _longPressRecord.wpBLat;
            _wp[1].lon  = _longPressRecord.wpBLon;
            break;
        default: break;
    }
    _labelAt        = 0;     // kill any active label animation
    _longPressFlash = false;
}

AppState::Persist AppState::save() const {
    return { _wp[1].lat, _wp[1].lon };
}

void AppState::restore(const Persist& p) {
    _wp[1].lat = p.wpBLat;
    _wp[1].lon = p.wpBLon;
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

    // commit the deferred mode cycle at peak overlay opacity
    if (_pendingCycle && in.now >= _cycleAt) {
        _pendingCycle = false;
        _mode = static_cast<Mode>((static_cast<u8>(_mode) + 1) % static_cast<u8>(Mode::COUNT));
        _setArmed = false;
        _setCount = 0;
        _label = modeName(_mode);   // swap text without resetting the animation clock
    }

    // reset a stalled capture so the counter doesn't linger forever
    if (_setArmed && (in.now - _setLastAt) > SET_WINDOW) {
        _setArmed = false;
        _setCount = 0;
    }

    if (in.haveBearing) formatDistance(in.distanceM);
}

// -- render handoff --
void AppState::draw(Display& disp, const Quaternion& qRose, const Quaternion& qArrow,
                    const Marker* markers, size_t markerCount) {
    // needle in compass mode, colored arrow when waypoint
    if (const Waypoint* wp = activeWaypoint()) disp.showArrow(wp->color);
    else                                       disp.showNeedle();

    HudState h{};
    h.label = _label;
    h.labelAlpha = labelAlpha(_in.now);
    h.labelRise = labelRise(_in.now);
    h.showDistance = _in.haveBearing;
    h.distance = _distBuf;
    h.noFix = !_in.fixValid;
    h.sats = _in.sats;
    h.fixValid = _in.fixValid;
    h.gpsTalking = _in.gpsTalking;
    h.battPct     = _in.haveBattery ? _in.battPct     : -1.0f;
    h.battVoltage = _in.haveBattery ? _in.battVoltage : -1.0f;
    h.battRingPush = battRingPush(_in.now);
    h.charging = _in.charging;
    h.statusAlpha = battRingPush(_in.now);
    h.centerHint = centerHintFor(_mode);
    h.centerHintAlpha = labelAlpha(_in.now);
    h.setArmed = _setArmed;
    h.setCount = _setCount;
    h.haveImu = _in.haveImu;
    h.calibSys = _in.calibSys;
    h.accent = ACCENT;

    disp.render(qRose, qArrow, markers, markerCount, h);
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

float AppState::battRingPush(u32 now) const {
    if (_longPressFlash) return 1.0f;   // hold actions keep ring off-screen
    const u32 t = now - _labelAt;
    if (t < LBL_IN)               return 1.0f - smooth(float(t) / LBL_IN);
    if (t < LBL_IN + LBL_HOLD)    return 0.0f;
    if (t < LBL_LIFE)              return smooth(float(t - LBL_IN - LBL_HOLD) / LBL_OUT);
    return 1.0f;
}

const char* AppState::centerHintFor(Mode m) const {
    switch (m) {
        case Mode::WaypointA: return "best place in the whole wide world";
        case Mode::WaypointB: return "HOLD TO SET";
        default:              return "";
    }
}

// -- string formatting --
void AppState::formatDistance(double m) {
    const double miles = m / 1609.344;
    if (miles < 0.1)        snprintf(_distBuf, sizeof(_distBuf), "%d FT",  (int)(m * 3.28084 + 0.5));
    else if (miles < 100.0) snprintf(_distBuf, sizeof(_distBuf), "%.2f MI", miles);
    else                    snprintf(_distBuf, sizeof(_distBuf), "%.0f MI", miles);
}

