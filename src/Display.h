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

// Everything the 2-D HUD needs for one frame. AppState owns the clocks and
// hands the renderer normalized animation phases, so Display stays a pure
// function of state - no millis(), no easing math, no string formatting.
// All colors are RGB565; all phases are 0..1.
struct HudState {
    // -- bottom-bezel label (curved, rises + fades on a mode switch) --
    const char* label; // mode name, or a transient flash ("METRIC", "TARGET SET")
    float labelAlpha; // 0 hidden .. 1 solid
    float labelRise; // 0 settled .. 1 fully below the bezel (entrance)

    // -- distance, shown quietly in the same spot once the label is gone --
    bool showDistance; // waypoint mode + fix
    const char* distance; // preformatted, units already applied
    bool noFix; // waypoint mode, no fix

    // -- corner status --
    u8 sats;
    bool fixValid;
    bool gpsTalking; // false = module silent; show "--" for sats
    float statusAlpha; // 0 hidden .. 1 solid (fades in as battery ring fades out)

    // -- battery ring (Apple-Watch style, hugs the bezel) --
    float battPct; // 0..100, <0 = unknown -> ring hidden
    float battVoltage; // volts, <0 = unknown
    float battRingPush; // 0 settled at bezel, 1 slid off the glass edge
    bool charging; // true -> show charge-state dot at 9:00

    // -- center hint (context-sensitive hold action, fades with mode label) --
    const char* centerHint;
    float centerHintAlpha; // 0 hidden .. 1 solid, same envelope as label

    // -- waypoint-set gesture feedback (B mode, long-press x3) --
    bool setArmed; // currently capturing a target
    u8 setCount; // 0..3 progress

    // -- IMU calibration pips (3 dots, filled = calibrated steps) --
    // Hidden when haveImu is false or calibSys reaches 3 (fully calibrated).
    bool haveImu;
    u8 calibSys; // 0..3

    // -- theme --
    u16 accent;
};

class Display {
public:
    // Which 3-D pointer floats above the rose.
    //   Needle : the classic two-tone compass needle (compass mode)
    //   Arrow  : a single-color arrow that points at the bearing (waypoint modes)
    enum class Indicator : u8 { Needle, Arrow };

    void begin();

    // Blank the screen, send SLPIN to the panel, and cut the backlight (if wired).
    // Call before esp_deep_sleep_start(). The panel re-wakes via lcd.init() in begin().
    void sleep();

    // qRose : drives the rose/ring/cardinals - always the true-north frame.
    // qArrow: drives the needle or bearing arrow - same as qRose in compass
    //         mode, or yaw(bearing)*qRose in waypoint mode so the arrow points
    //         at the target while N/E/S/W letters stay geographically fixed.
    void render(const Quaternion& qRose, const Quaternion& qArrow,
                const Marker* m, size_t count, const HudState& hud);

    // Take over the whole screen with a centered, faded image from LittleFS.
    // Each image is decoded once into a PSRAM sprite; subsequent frames for the
    // same image just blit - no per-frame re-decode.
    //   fadeAlpha: 0.0 = black, 1.0 = full image (fade in / fade out)
    // A null or unreadable path draws a quiet placeholder.
    void showSlide(const char* path, float fadeAlpha);

    // -- indicator control --
    void setIndicator(Indicator i) { _indicator = i; }
    Indicator indicator() const { return _indicator; }

    // Arrow color is RGB565 (same convention as Marker::color). The needle
    // ignores this - it's permanently two-tone.
    void setArrowColor(u16 rgb565) { _arrowColor = rgb565; }
    u16 arrowColor() const { return _arrowColor; }

    // Ergonomic one-liners for mode switches.
    void showNeedle() { _indicator = Indicator::Needle; }
    void showArrow(u16 rgb565) { _arrowColor = rgb565; _indicator = Indicator::Arrow; }

private:
    LGFX lcd;
    LGFX_Sprite fb{ &lcd };
    LGFX_Sprite _imgBuf; // SCREEN*SCREEN decode buffer (zoom-to-fill)

    char _imgPath[64] = {}; // path of the currently decoded image
    int _imgW = 0; // decoded image width (px); -1 = decode failed
    int _imgH = 0; // decoded image height (px); -1 = decode failed

    Indicator _indicator = Indicator::Needle;
    u16 _arrowColor = 0xFD20; // amber by default; override anytime
};
