#pragma once

#include "shorthand.h"
#include "geometry.h"   // Quaternion, for the orientation mount correction
#include "Color.h"      // color::pack, for the UI theme accent

namespace ESP32S3Pinout {

    constexpr u8 PIN_GPS_PPS        = 1;
    constexpr u8 PIN_BNO055_RESET   = 2;
    constexpr u8 PIN_BNO055_INT     = 3;
    constexpr u8 PIN_BUTTON         = 4;
    constexpr u8 PIN_I2C_SDA        = 5;
    constexpr u8 PIN_I2C_SCL        = 6;
    constexpr u8 PIN_GPS_TX_ESP_RX  = 43;
    constexpr u8 PIN_GPS_RX_ESP_TX  = 44;
    constexpr u8 PIN_SPI_SCK        = 7;
    constexpr u8 PIN_SPI_DC         = 8;
    constexpr u8 PIN_SPI_MOSI       = 9;

}

namespace GpsConfig {
    constexpr u8 UART_NUM = 1;
    constexpr u32 BAUD = 9600;
    constexpr u32 FIX_TIMEOUT_MS = 5000;
    constexpr double DEFAULT_DECLINATION_DEG = 2.02;    // Tulsa, OK
    constexpr double RECOMPUTE_DECLINATION_DISTANCE = 10000.0;  // 10 km
    constexpr u32 RECOMPUTE_DECLINATION_TIME = 600000;          // 10 min
}

namespace MAX10748Config {
    constexpr u8 MAX17048_ADDR = 0x36;

    // register map
    constexpr u8 REG_VCELL  = 0x02; // 78.125 uV / LSB
    constexpr u8 REG_SOC    = 0x04; // (1/256) % / LSB
    constexpr u8 REG_MODE   = 0x06; // EnSleep / HibStat
    constexpr u8 REG_CONFIG = 0x0C; // RCOMP (MSB) | SLEEP/ALSC/ALRT/ATHD (LSB)
    constexpr u8 REG_CRATE  = 0x16; // 0.208 %/hr / LSB, signed
    constexpr u8 REG_CMD    = 0xFE; // command register (POR)

    // Datasheet scale factors
    constexpr f32 VCELL_LSB_V     = 0.000078125f; // 78.125 uV
    constexpr f32 SOC_LSB_PCT     = 1.0f / 256.0f;
    constexpr f32 CRATE_LSB_PCTHR = 0.208f;

    // MODE / CONFIG bits used here
    constexpr u16 MODE_ENSLEEP = 0x2000;
    constexpr u16 CFG_SLEEP    = 0x0080;

    // Command-register word that triggers a power-on reset.
    constexpr u16 CMD_POR = 0x5400;

}

namespace BNO055Config {

    constexpr u8 REG_PAGE_ID          = 0x07;
    constexpr u8 REG_CHIP_ID          = 0x00;
    constexpr u8 REG_EULER_H_LSB      = 0x1A; // 6 bytes: H, R, P (LSB/MSB each)
    constexpr u8 REG_QUAT_W_LSB       = 0x20; // 8 bytes: W, X, Y, Z (LSB/MSB each)
    constexpr u8 REG_CALIB_STAT       = 0x35;
    constexpr u8 REG_UNIT_SEL         = 0x3B;
    constexpr u8 REG_OPR_MODE         = 0x3D;
    constexpr u8 REG_PWR_MODE         = 0x3E;
    constexpr u8 REG_SYS_TRIGGER      = 0x3F;
    constexpr u8 REG_ACCEL_OFFSET     = 0x55; // start of the 22-byte calib profile

    constexpr u8 REG_INT_MASK         = 0x0F; // page 1
    constexpr u8 REG_INT_EN           = 0x10; // page 1
    constexpr u8 REG_ACC_AM_THRES     = 0x11; // page 1
    constexpr u8 REG_ACC_INT_SETTINGS = 0x12; // page 1

    constexpr u8 CHIP_ID_VALUE        = 0xA0;

    constexpr u8 PWR_MODE_NORMAL      = 0x00;
    constexpr u8 SYS_TRIGGER_RESET    = 0x20; // RST_SYS  (bit 5)
    constexpr u8 SYS_TRIGGER_RST_INT  = 0x40; // RST_INT  (bit 6)
    constexpr u8 SYS_TRIGGER_EXTCLK   = 0x80; // CLK_SEL  (bit 7) -> external crystal
    constexpr u8 UNIT_SEL_DEFAULTS    = 0x00; // deg, dps, m/s^2, Celsius, Windows orient

    constexpr u8 INT_ACC_AM           = 0x40; // any-motion (bit 6) in INT_EN / INT_MASK
    constexpr u8 ACC_INT_AM_AXES      = 0x1C; // any/no-motion X,Y,Z enables (bits 2-4)

    constexpr bool USE_EXTERNAL_CRYSTAL  = true;

    // datasheet timings with some room for sauce ig
    constexpr u32 BOOT_TIMEOUT_MS   = 900;  // power-on / post-reset boot (650 ms)
    constexpr u32 DELAY_TO_CONFIG   = 20;   // any fusion mode -> config (19 ms)
    constexpr u32 DELAY_FROM_CONFIG = 8;    // config -> any mode (7 ms)

}

// ===========================================================================
//  Cross-cutting tunables
//
//  The central home for the knobs that span more than one translation unit:
//  the angle/earth constants the navigation math shares, how the orientation
//  is mounted and smoothed, the loop cadences, and the UI theme + animation
//  timings. Renderer-internal layout (mesh ratios, pixel geometry, the color
//  palette) deliberately stays local to Display.cpp — it isn't config, it's
//  the look of one screen.
// ===========================================================================

namespace MathConst {
    constexpr f64 DEG2RAD = 3.14159265358979323846 / 180.0;
    constexpr f64 RAD2DEG = 180.0 / 3.14159265358979323846;
}

namespace NavConfig {
    constexpr f64 EARTH_RADIUS_M = 6371000.0;   // mean radius, haversine + bearing
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
    constexpr u32 FRAME_MS = 20;     // render cadence (~50 fps)
    constexpr u32 BATT_MS  = 3000;   // battery poll cadence
}

namespace UiConfig {
    // One accent drives all UI chrome (lubber, battery ring, curved label,
    // press splash, calib pips, charging bolt). RGB565.
    constexpr u16 ACCENT = color::pack(0, 210, 200);   // cool cyan-teal

    // Bottom-label timeline (ms): rise+fade-in, full hold, fade-out.
    constexpr u32 LBL_IN   = 220;
    constexpr u32 LBL_HOLD = 1500;
    constexpr u32 LBL_OUT  = 380;
    constexpr u32 LBL_LIFE = LBL_IN + LBL_HOLD + LBL_OUT;

    constexpr u32 SPLASH_MS  = 340;    // left-edge press splash lifetime
    constexpr u32 SET_WINDOW = 4000;   // waypoint-capture holds must land within this
}