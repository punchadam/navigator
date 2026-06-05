#pragma once

#include <cstdint>
#include <cstddef>
#include "LGFX_Config.hpp"
#include "shorthand.h"
#include "geometry.h"
#include "Canvas.h"

struct Marker {
    float angleRad; // world frame compass angle, 0 = north, clockwise
    u16 color;
    const char* label;
};

// Everything the 2-D HUD needs for one frame. The app (AppState) owns the
// clocks and hands the renderer *normalized* animation phases, so the Display
// stays a pure function of state — no millis(), no easing math, no string
// formatting in here. All colors are RGB565; all phases are 0..1.
struct HudState {
    // --- bottom-bezel label (curved, rises + fades on a mode switch) --------
    const char* label;        // mode name, or a transient flash ("METRIC", "TARGET SET")
    float       labelAlpha;   // 0 hidden .. 1 solid
    float       labelRise;    // 0 settled .. 1 fully below the bezel (entrance)

    // --- distance, shown quietly in the same spot once the label is gone ----
    bool        showDistance; // waypoint mode + fix
    const char* distance;     // preformatted, units already applied
    bool        noFix;        // waypoint mode, no fix

    // --- corner status -----------------------------------------------------
    u8     sats;
    bool        fixValid;
    u8     calib;        // orientation calibration, 0..3 (0 = uncalibrated)

    // --- battery ring (Apple-Watch style, hugs the bezel) ------------------
    float       battPct;      // 0..100, <0 = unknown -> ring hidden

    // --- left-edge press splash (button is physically on the left) ---------
    float       pressSplash;  // 0 none .. 1 fresh press

    // --- waypoint-set gesture feedback (B mode, long-press x3) --------------
    bool        setArmed;     // currently capturing a target
    u8     setCount;     // 0..3 progress

    // --- theme -------------------------------------------------------------
    u16    accent;
};

// Full-screen charging view. No compass, no rose — a single bolt plus a smart
// readout. The app formats every string; the renderer just lays them out.
struct ChargeState {
    float       pct;          // 0..100, drives the bolt-fill ring
    bool        full;         // true -> green bolt, "charged" copy
    const char* line1;        // e.g. "CHARGING"
    const char* line2;        // e.g. "78%"
    const char* line3;        // e.g. "FULL IN 1.5 H"
    u16    accent;
};

class Display {
public:
    // Which 3-D pointer floats above the rose.
    //   Needle : the classic two-tone compass needle (compass mode)
    //   Arrow  : a single-color arrow that points at the bearing (waypoint modes)
    enum class Indicator : u8 { Needle, Arrow };

    void begin();

    // qShow: device orientation (compass) or the pre-rotated bearing frame
    //        (waypoint) — the renderer treats both identically. The 2-D HUD is
    //        composited on top of the finished 3-D frame, then pushed.
    void render(const Quaternion& qShow, const Marker* m, size_t count,
                const HudState& hud);

    // Full-screen charging view. Turns every compass feature off.
    void renderCharging(const ChargeState& cs);

    // --- indicator control -----------------------------------------------------
    void      setIndicator(Indicator i)   { _indicator = i; }
    Indicator indicator() const           { return _indicator; }

    // Arrow color is RGB565 (same convention as Marker::color). The needle
    // ignores this — it's permanently two-tone.
    void     setArrowColor(u16 rgb565) { _arrowColor = rgb565; }
    u16 arrowColor() const             { return _arrowColor; }

    // Ergonomic one-liners for mode switches.
    void showNeedle()                { _indicator = Indicator::Needle; }
    void showArrow(u16 rgb565)  { _arrowColor = rgb565; _indicator = Indicator::Arrow; }

private:
    LGFX        lcd;
    LGFX_Sprite fb{ &lcd };

    Indicator _indicator  = Indicator::Needle;
    u16  _arrowColor = 0xFD20;   // amber by default; override anytime
};