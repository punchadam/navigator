#pragma once

#include <cstdint>
#include <cstddef>
#include "shorthand.h"
#include "geometry.h"
#include "Display.h"

// AppState
// UI controller
//
// owns everything abt how the UI acts: current mode, unit preference, animation clocks
// (mode-label pop, button splash), the waypoint table, the charging-screen latch, and
// all the string formatting stuff reqd by hud. Owns no hardware and does no navi math
// bc main reads the sensors and computes bearings/distances, then feeds the results in
// through update(). The sensor stuff used to be all over the place I finally fixed it last-min
//
// Lifecycle per frame, from main:
//      appState.update(inputs);                // advance clocks, format text
//      appState.draw(display, qShow, m, n);    // build HudState, render
//
// The button lives in main but its callbacks just touch cycleMode() / notePress(), onLongPress()

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

    void begin(u32 now);

    // short-press to change mode
    void cycleMode(u32 now);
    void notePress(u32 now) { _pressAt = now; } // fires the left-edge splash
    Mode mode() const { return _mode; }

    // long-press has a different effect per mode
    // Compass: toggle distance units
    // A: flash "LOCKED" (A's coordinates are fixed)
    // B: capture the target, but needs three holds to commit
    // main passes the live fix so B can be written here.
    void onLongPress(u32 now, bool fixValid, double lat, double lon);

    bool unitsMetric() const { return _metric; }

    // waypoint access for main's navi math
    const Waypoint& waypoint(Mode m) const; // A or B
    const Waypoint* activeWaypoint() const; // nullptr in compass mode

    // -- per-frame inputs from main --
    struct Inputs {
        u32 now;
        // orientation
        bool haveImu;
        u8 calib;               // 0..3 orientation calibration
        // gps
        bool fixValid;
        u8 sats;
        // navigation (computed in main for the *active* mode)
        bool haveBearing;       // waypoint mode + fix
        double distanceM;
        // battery
        bool haveBattery;
        f32 battPct;            // 0..100, <0 = unknown
        f32 chargeRatePctHr;    // signed: + charging, - discharging
        bool charging;
    };

    // advance clocks, expire the waypoint-set gesture, run the charging latch,
    // and format the distance and charging strings
    void update(const Inputs& in);

    // build the HudState / ChargeState and hand off to the renderer
    void draw(Display& disp, const Quaternion& qShow,
              const Marker* markers, size_t markerCount);

    bool chargingScreenActive() const { return _chargingScreen; }

private:
    Mode _mode = Mode::Compass;
    bool _metric  = true;
    Waypoint _wp[2];    // [0] = A (locked), [1] = B (settable)

    // animation clocks
    u32 _labelAt = 0;   // last time the bottom label was (re)triggered
    const char* _label = "COMPASS"; // what the bottom label currently shows
    u32 _pressAt = 0;   // last short press, for the splash

    // waypoint-set gesture
    bool _setArmed  = false;
    u8 _setCount  = 0;
    u32 _setLastAt = 0;

    // charging latch
    bool _chargingScreen = false;
    bool _full           = false;

    // cached, formatted strings (HudState/ChargeState point at these)
    char _distBuf[16] = {0};
    char _chgPct[8] = {0};
    char _chgEta[20] = {0};

    Inputs _in{};   // last inputs, used by draw()

    // bring a string up at the bottom bezel (mode names + transient flashes)
    void flash(const char* msg, u32 now) { _label = msg; _labelAt = now; }

    void formatDistance(double meters);
    void formatCharge();

    float labelAlpha(u32 now) const;
    float labelRise(u32 now)  const;
    float pressSplash(u32 now) const;

    const char* modeName(Mode m) const;
};