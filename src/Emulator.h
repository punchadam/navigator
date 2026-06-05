#pragma once

#include "shorthand.h"  // u32, f32, etc.

// Emulator.h - tunable defaults for the bench-test emulator build.
//
// This file is included only by Emulator.cpp. To adjust emulator behaviour:
// edit the values in EmulatorConfig below, rebuild the [env:emulator] target.
//
// To remove the emulator entirely: just use [env:seeed_xiao_esp32s3]. This
// file and Emulator.cpp are only compiled when that env is selected.

namespace EmulatorConfig {

    // -- BNO055 emulator --

    // How fast the emulated heading sweeps, degrees per second.
    // Negative = counter-clockwise. 0 = stationary at 0 deg.
    constexpr float BNO_SWEEP_DEG_PER_SEC = 30.0f;

    // Peak-to-peak noise added to the heading on every quaternion read.
    // Set to 0 for a perfectly smooth sweep.
    constexpr float BNO_JITTER_DEG = 1.5f;

    // -- Battery emulator --

    // Starting state-of-charge (0-100 %).
    constexpr float BATT_START_PCT = 75.0f;

    // How fast the SOC falls when not charging, percent per hour.
    constexpr float BATT_DRAIN_RATE_PCT_HR = 12.0f;

    // How fast the SOC rises when charging, percent per hour.
    constexpr float BATT_CHARGE_RATE_PCT_HR = 30.0f;

    // Whether charging is on at startup.
    constexpr bool BATT_CHARGING_INIT = false;

    // -- GPS emulator --

    // Fixed position used for all emulated fixes (Phoenix, AZ).
    constexpr double GPS_LAT = 33.55058269423492;
    constexpr double GPS_LON = -112.08881000226809;
    constexpr double GPS_ALT_M = 340.0;

    // Simulated satellite count reported once a fix is acquired.
    constexpr u8 GPS_SATELLITES = 9;

    // Time after begin() before hasFix() returns true, ms.
    constexpr u32 GPS_FIX_DELAY_MS = 2500;

    // -- Serial status --

    // Interval between automatic status lines on Serial, ms.
    // Set to 0 to disable periodic printing (use '?' command instead).
    constexpr u32 STATUS_PRINT_MS = 2000;

} // namespace EmulatorConfig
