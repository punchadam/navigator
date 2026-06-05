#pragma once

#include <cstdint>
#include <cstddef>
#include "shorthand.h"
#include "geometry.h"
#include "Display.h"

// AppState
// UI controller
//
// owns everything abt how the UI acts: current mode, animation clocks
// (mode-label pop, button splash), the waypoint table, and all the string formatting
// stuff reqd by hud. Owns no hardware and does no navi math bc main reads the sensors
// and computes bearings/distances, then feeds the results in through update().
//
// Lifecycle per frame, from main:
//      appState.update(inputs);                // advance clocks, format text
//      appState.draw(display, qShow, m, n);    // build HudState, render
//
// The button lives in main but its callbacks just touch cycleMode() / onLongPress()

class AppState {
public:
    enum class Mode : u8 { Compass = 0, WaypointA = 1, WaypointB = 2, COUNT = 3 };

    struct Waypoint {
        const char* name;   // shown in the HUD ("WAYPOINT A")
        const char* tag;    // short rose-marker label ("A")
        double lat;
        double lon;
        u16 color;          // RGB565, used for the rose marker + arrow
        bool settable;      // A is locked and B can be captured from GPS
    };

    // Minimal state that survives deep sleep (stored in RTC memory by main).
    struct Persist {
        double wpBLat, wpBLon;  // waypoint B position
    };
    Persist save()  const;
    void    restore(const Persist& p);

    void begin(u32 now);

    // short-press to change mode
    void cycleMode(u32 now);
    Mode mode() const { return _mode; }

    // long-press has a different effect per mode
    // Compass: no-op (flashes nothing)
    // A: flash "LOCKED" (A's coordinates are fixed)
    // B: capture the target, but needs three holds to commit
    // main passes the live fix so B can be written here.
    void onLongPress(u32 now, bool fixValid, double lat, double lon);

    // Reverses the effect of the last onLongPress call. Called by main when
    // the power-off threshold is crossed so the long press doesn't stick.
    void undoLongPress(u32 now);

    // waypoint access for main's navi math
    const Waypoint& waypoint(Mode m) const; // A or B
    const Waypoint* activeWaypoint() const; // nullptr in compass mode

    // -- per-frame inputs from main --
    struct Inputs {
        u32 now;
        // orientation
        bool haveImu;
        u8 calibSys; // 0..3 BNO055 system calibration status
        // gps
        bool fixValid;
        bool gpsTalking;
        u8 sats;
        // navigation (computed in main for the *active* mode)
        bool haveBearing; // waypoint mode + fix
        double distanceM;
        // battery
        bool haveBattery;
        f32 battPct; // 0..100, <0 = unknown
        f32 battVoltage; // volts, <0 = unknown
        bool charging;
    };

    // advance clocks, expire the waypoint-set gesture, and format the distance string
    void update(const Inputs& in);

    // build the HudState and hand off to the renderer.
    // qRose  = true-north frame (rose/cardinals stay N-aligned).
    // qArrow = bearing frame in waypoint mode (same as qRose in compass mode).
    void draw(Display& disp, const Quaternion& qRose, const Quaternion& qArrow,
              const Marker* markers, size_t markerCount);

private:
    Mode _mode = Mode::Compass;
    Waypoint _wp[2];    // [0] = A (locked), [1] = B (settable)

    // animation clocks
    u32 _labelAt = 0; // last time the bottom label was (re)triggered
    const char* _label = "COMPASS"; // what the bottom label currently shows
    bool _longPressFlash = false; // suppress battery ring for hold-triggered labels

    // deferred mode cycle: short press starts the overlay; mode flips at peak
    bool _pendingCycle = false;
    u32 _cycleAt = 0;

    // waypoint-set gesture
    bool _setArmed = false;
    u8 _setCount = 0;
    u32 _setLastAt = 0;

    // cached, formatted string (HudState points at this)
    char _distBuf[16] = {0};

    Inputs _in{};   // last inputs, used by draw()

    struct LongPressRecord {
        Mode   mode;
        bool   setArmed;
        u8     setCount;
        double wpBLat;
        double wpBLon;
    };
    LongPressRecord _longPressRecord{};

    // bring a string up at the bottom bezel (mode names + transient flashes)
    void flash(const char* msg, u32 now) { _label = msg; _labelAt = now; }

    void formatDistance(double meters);

    float labelAlpha(u32 now)   const;
    float labelRise(u32 now)    const;
    float battRingPush(u32 now) const;
    const char* centerHintFor(Mode m) const;

    const char* modeName(Mode m) const;
};