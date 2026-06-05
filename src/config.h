#pragma once

#include "shorthand.h"
#include "geometry.h"   // Quaternion, for the orientation mount correction
#include "Color.h"      // color::pack, for the UI theme accent

namespace ESP32S3Pinout {

    constexpr u8 PIN_GPS_PPS = 1;
    constexpr u8 PIN_BNO055_RESET = 2;
    constexpr u8 PIN_BNO055_INT = 3;
    constexpr u8 PIN_BUTTON = 4;
    constexpr u8 PIN_I2C_SDA = 5;
    constexpr u8 PIN_I2C_SCL = 6;
    constexpr u8 PIN_GPS_TX_ESP_RX = 44;
    constexpr u8 PIN_GPS_RX_ESP_TX = 43;
    constexpr u8 PIN_SPI_SCK = 7;
    constexpr u8 PIN_SPI_DC = 8;
    constexpr u8 PIN_SPI_MOSI = 9;

}

namespace GpsConfig {
    constexpr u8 UART_NUM = 1;
    constexpr u32 BAUD = 9600;
    constexpr u32 FIX_TIMEOUT_MS = 5000;
    // If no chars arrive within this window the module is considered silent.
    // hasFix() returns false immediately; SATS shows "--".
    constexpr u32 TALKING_TIMEOUT_MS = 3000;
    constexpr double DEFAULT_DECLINATION_DEG = 2.02; // Tulsa, OK
    constexpr double RECOMPUTE_DECLINATION_DISTANCE = 10000.0; // 10 km
    constexpr u32 RECOMPUTE_DECLINATION_TIME = 600000; // 10 min
}


namespace BNO055Config {
    constexpr bool USE_EXTERNAL_CRYSTAL = true;
}

namespace BatteryConfig {
    // ModelGauge compensation byte (0x00-0xFF). Default 0x97.
    // Increase to shift SOC readings higher; decrease to shift lower.
    // Tune by comparing rested open-circuit voltage to a LiPo SOC table.
    constexpr u8  RCOMP  = 0xB0;

    // Full-charge voltage for this cell (V). When voltage >= V_FULL while
    // charging, the gauge is seeded with 100% so the ModelGauge algorithm
    // tracks from the correct endpoint instead of a 4.20V-based ceiling.
    constexpr f32 V_FULL = 4.10f;

    // Voltage-trend window for charging detection. If the cell voltage has risen
    // at least CHARGE_VTREND_V over the last CHARGE_VTREND_SAMPLES polls, the
    // device is flagged as charging even if CRATE hasn't gone positive yet.
    constexpr u8 CHARGE_VTREND_SAMPLES = 10; // * BATT_SOC_MS = 30 s window
    // Required mean-voltage difference between the newer and older halves of the
    // trend window. Uses half-mean comparison rather than endpoint delta so that a
    // slope reversal (falling -> rising) is detected even when the absolute voltage
    // rise is tiny (about 0.07 mV/poll at 20 mA net charge). 0.3 mV sits well above
    // ADC noise (about 0.05 mV on the half-mean) while the expected signal at 20 mA net
    // charge is about 0.35 mV and a full slope reversal produces about 0.75 mV.
    constexpr f32 CHARGE_VTREND_V = 0.0003f;

    // Battery dead state thresholds. No hardware kill switch exists for the
    // display backlight or I2C ICs, so at V_DEAD we do a software shutdown:
    // display SLPIN, BNO055 suspend, GPS power-save. Polls slowly until the
    // cell climbs back to V_RECOVER (hysteresis prevents thrashing), then
    // restarts the firmware for a clean re-init.
    constexpr f32 V_DEAD       = 3.40f;  // enter dead state below this (V)
    constexpr f32 V_RECOVER    = 3.55f;  // exit dead state above this (V)
    constexpr u32 DEAD_POLL_MS = 10000;  // voltage check interval while dead (ms)
}

// Cross-cutting tunables
//
// Central home for knobs that span more than one translation unit:
// angle/earth constants the navigation math shares, how the orientation
// is mounted and smoothed, the loop cadences, and the UI theme + animation
// timings. Renderer-internal layout (mesh ratios, pixel geometry, the color
// palette) deliberately stays local to Display.cpp - it isn't config, it's
// the look of one screen.

namespace MathConst {
    constexpr f64 DEG2RAD = 3.14159265358979323846 / 180.0;
    constexpr f64 RAD2DEG = 180.0 / 3.14159265358979323846;
}

namespace NavConfig {
    constexpr f64 EARTH_RADIUS_M = 6371000.0; // mean radius, haversine + bearing
}

namespace OrientationConfig {
    // Orientation low-pass, 0..1 (higher = snappier).
    constexpr f32 SMOOTH = 0.25f;

    // Sensor-to-body mounting rotation (body<-sensor). Identity means the
    // BNO055 axes already are the device axes. If the sensor is mounted rotated
    // relative to the screen, set this so that, held level with the screen's top
    // edge pointing at a heading, the rose reads that heading under the lubber.
    constexpr Quaternion MOUNT_CORRECTION{ 1.0f, 0.0f, 0.0f, 0.0f };
}

namespace RuntimeConfig {
    constexpr u32 FRAME_MS = 20; // render cadence (about 50 fps)
    constexpr u32 BATT_SOC_MS = 3000; // SOC poll cadence (chip updates about every 1 s, 3 s is plenty)
    constexpr u32 IMU_MS = 1000; // IMU debug print cadence
    constexpr u32 GPS_MS = 2000; // GPS debug print cadence
}

namespace WaypointDefaults {
    // default coordinates for the two waypoints
    //constexpr double A_LAT = 33.550371962796845;
    //constexpr double A_LON = -112.08868095959146;
    constexpr double A_LAT =  33.48389929090025;
    constexpr double A_LON = -112.04760507898322;

    constexpr double B_LAT =  33.507091077948886;
    constexpr double B_LON = -112.03774183856902;
}

namespace SlideshowConfig {
    // The home-waypoint "you've arrived" stage.
    //
    // Waypoint A is the whole point of the device, but the compass is only good
    // to about 50 ft - once you're standing on top of A the bearing and distance are
    // just GPS noise spinning the needle. So inside a 200 ft bubble around A we
    // hide the compass entirely and loop a folder of photos instead.
    //
    // Images are centered and fit to the 240x240 screen; each fades in/out over FADE_MS.
    // Drop images (.jpg/.jpeg/.png/.bmp) into data/slideshow/ and flash:
    //     pio run -t uploadfs
    constexpr double FEET_TO_M = 0.3048;
    constexpr double RADIUS_M = 200.0 * FEET_TO_M; // about 61 m: enter the stage
    constexpr double HYSTERESIS_M = 30.0 * FEET_TO_M; // walk this far back out to leave
    constexpr u32 FRAME_MS = 4000; // how long each photo is held (ms)
    constexpr u32 FADE_MS = 1000; // fade-in / fade-out duration (ms)
    constexpr const char* FOLDER = "/slideshow"; // LittleFS path scanned for images
}

namespace SleepConfig {
    // Deep-sleep triggers
    constexpr u32 IDLE_TIMEOUT_MS = 3UL * 60UL * 1000UL; // no activity -> sleep
    constexpr u32 SLEEP_HOLD_MS = 3000; // hold this long -> sleep now

    // Motion-based idle extension. Consecutive quaternion frames are compared;
    // if the dot product drops below this value the device is moving and the
    // idle timer resets. 0.9999 is about 1.6 degrees of rotation per 20 ms frame (about 80 deg/s).
    // Lower = more sensitive (catches slower movement); higher = less sensitive.
    constexpr f32 MOTION_STILL_DOT = 0.9999f;

    // After the 3-second hold darkens the screen, how long the button must stay
    // released before we commit to deep sleep. The Button class already debounces
    // the release edge; this window sits on top of that for extra safety.
    constexpr u32 SLEEP_RELEASE_SETTLE_MS = 500;

    // Charge-detection polling while asleep. Plugging in usually shakes the device
    // enough to trigger the BNO any-motion wake, but this timer is the safety net
    // for a quiet dock or cradle. Each wake does about 50 ms of I2C work then re-sleeps.
    constexpr u32 CHARGE_POLL_MS = 30000; // how often to check (ms)
    constexpr u32 GAUGE_WAKE_MS = 10; // settle after waking the MAX17048
}

namespace UiConfig {
    // One accent drives all UI chrome (lubber, battery ring, curved label,
    // press splash, calib pips, charging bolt). RGB565.
    constexpr u16 ACCENT = color::pack(0, 210, 200); // cool cyan-teal

    // Bottom-label timeline (ms): rise+fade-in, full hold, fade-out.
    constexpr u32 LBL_IN = 220;
    constexpr u32 LBL_HOLD = 1500;
    constexpr u32 LBL_OUT = 380;
    constexpr u32 LBL_LIFE = LBL_IN + LBL_HOLD + LBL_OUT;

    constexpr u32 SET_WINDOW = 4000; // waypoint-capture holds must land within this
}

// Serial debug output toggles
//
// Set any flag to false to silence that sensor's debug prints entirely.
// All prints are suppressed when false with zero runtime overhead (dead-code
// elimination on constexpr booleans).
namespace DebugConfig {
    constexpr bool IMU     = true; // BNO055: init result, calibration changes
    constexpr bool GPS     = true; // GPS: UART start, fix transitions, declination
    constexpr bool BATTERY = true; // MAX17048: init result, periodic V/SOC/rate
}
