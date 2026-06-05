// main.cpp — waypoint compass
//
// Pulls absolute orientation from the BNO055 and a position fix from the GPS,
// folds in magnetic declination, computes the great-circle bearing to a chosen
// waypoint, and hands the result to the Display via AppState.
//
// Division of labor:
//   main      — hardware + navigation math (orientation smoothing, declination
//               fold, bearing/distance). Owns the button and feeds AppState.
//   AppState  — the UI controller: mode, units, animation timing, the waypoint
//               table, charging logic, and every formatted HUD string.
//   Display   — a pure renderer: (qShow, markers, HudState) -> pixels, plus a
//               dedicated charging screen.
//
// The two world-up rotations that used to clutter this file are unchanged:
//   compass :  qTrue = yaw(-declination) * qDevice        (+Y == true North)
//   waypoint:  qShow = yaw(+bearing)     * qTrue           (+Y == bearing dir)
// The button cycles Compass -> Waypoint A -> Waypoint B -> Compass. Charging is
// automatic and overrides the display while the cable is in.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "shorthand.h"
#include "config.h"
#include "geometry.h"
#include "Orientation.h"
#include "GPS.h"
#include "Display.h"
#include "Button.h"
#include "Battery.h"
#include "Color.h"
#include "AppState.h"

using namespace ESP32S3Pinout;

// Cross-cutting tunables (cadences, smoothing, mount correction, angle/earth
// constants, theme + animation timings) all live in config.h now. Bring the
// math constants into scope so the navigation helpers below read naturally.
using namespace MathConst;
using OrientationConfig::SMOOTH;
using OrientationConfig::MOUNT_CORRECTION;
using RuntimeConfig::FRAME_MS;
using RuntimeConfig::BATT_MS;

// ---------------------------------------------------------------------------
//  Hardware
// ---------------------------------------------------------------------------
BNO055  imu(Wire, 0x28, PIN_BNO055_RESET, PIN_BNO055_INT);
GPS     gps;
Display display;
Battery battery;
Button  button(PIN_BUTTON, INPUT_PULLDOWN, HIGH);   // pulled down -> active HIGH

// ---------------------------------------------------------------------------
//  App state
// ---------------------------------------------------------------------------
AppState   app;
Quaternion g_orient      { 1, 0, 0, 0 };   // smoothed device orientation
bool       g_orientInit  = false;
bool       g_haveImu     = false;
bool       g_haveBattery = false;
float      g_battPct     = -1.0f;
float      g_battRate    = 0.0f;           // %/hr, signed
bool       g_charging    = false;

// ---------------------------------------------------------------------------
//  Navigation math
// ---------------------------------------------------------------------------
// yawQuat() — the angle->quaternion primitive — now lives in geometry.h and is
// shared with GPS::declinationQuat().

static Quaternion nlerp(const Quaternion& a, Quaternion b, float t) {
    if (dot(Vec3{a.x, a.y, a.z}, Vec3{b.x, b.y, b.z}) + a.w * b.w < 0.0f) {
        b = { -b.w, -b.x, -b.y, -b.z };
    }
    Quaternion r{ a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
                  a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
    return normalize(r);
}

// Initial true (geographic) great-circle bearing, degrees [0,360).
static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
    const double y = sin(dl) * cos(p2);
    const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) * RAD2DEG;
    if (b < 0.0) b += 360.0;
    return b;
}

// Great-circle distance, meters (haversine).
static double distanceM(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    const double dp = (lat2 - lat1) * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
    const double a  = sin(dp / 2) * sin(dp / 2)
                    + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return NavConfig::EARTH_RADIUS_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// ---------------------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    display.begin();
    gps.begin();
    app.begin(millis());

    g_haveImu = imu.begin(OpMode::Ndof);
    if (!g_haveImu) Serial.println("[imu] BNO055 not responding — static rose.");

    g_haveBattery = battery.begin(Wire, /*initBus=*/false);

    // Short press cycles the mode (+ fires the left-edge splash). Long press is
    // mode-specific; on Waypoint B, three holds capture the current fix.
    button.begin();
    button.onPress([] {
        const u32 t = millis();
        app.cycleMode(t);
        app.notePress(t);
    });
    button.onLongPress([] {
        const GpsFix f = gps.read();
        app.onLongPress(millis(), f.valid, f.latitude, f.longitude);
    });
}

// ---------------------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------------------
void loop() {
    gps.update();        // drain the UART faster than bytes arrive
    button.update();     // tight debounce timing

    const u32 now = millis();

    static u32 lastBatt = 0;
    if (g_haveBattery && now - lastBatt >= BATT_MS) {
        lastBatt    = now;
        g_battPct   = battery.percent();
        g_battRate  = battery.chargeRate();
        g_charging  = battery.charging();
    }

    static u32 lastFrame = 0;
    if (now - lastFrame < FRAME_MS) return;
    lastFrame = now;

    // --- orientation: read, mount-correct, smooth ------------------------------
    if (g_haveImu) {
        Quaternion q;
        if (imu.readQuaternion(q)) {
            q = normalize(multiply(q, MOUNT_CORRECTION));
            g_orient     = g_orientInit ? nlerp(g_orient, q, SMOOTH) : q;
            g_orientInit = true;
        }
    }

    // --- declination fold: device frame -> true-North world frame --------------
    // GPS owns the magnetic->true correction; we just left-multiply it.
    const Quaternion qTrue = normalize(multiply(gps.declinationQuat(), g_orient));

    const GpsFix fix = gps.read();

    // --- per-mode navigation ---------------------------------------------------
    const AppState::Waypoint* wp = app.activeWaypoint();   // nullptr == compass mode

    Quaternion qShow = qTrue;
    Marker     markers[2];
    size_t     markerCount = 0;
    bool       haveBrg     = false;
    double     distM       = 0.0;

    if (!wp) {
        // Compass: drop a marker on the rose at each waypoint's true bearing.
        if (fix.valid) {
            const AppState::Waypoint& A = app.waypoint(AppState::Mode::WaypointA);
            const AppState::Waypoint& B = app.waypoint(AppState::Mode::WaypointB);
            const double ba = bearingDeg(fix.latitude, fix.longitude, A.lat, A.lon);
            const double bb = bearingDeg(fix.latitude, fix.longitude, B.lat, B.lon);
            markers[markerCount++] = Marker{ float(ba * DEG2RAD), A.color, A.tag };
            markers[markerCount++] = Marker{ float(bb * DEG2RAD), B.color, B.tag };
        }
    } else if (fix.valid) {
        // Waypoint: yaw the true frame so +Y points down the bearing.
        const double b = bearingDeg(fix.latitude, fix.longitude, wp->lat, wp->lon);
        distM   = distanceM(fix.latitude, fix.longitude, wp->lat, wp->lon);
        haveBrg = true;
        qShow   = normalize(multiply(yawQuat(static_cast<float>(b * DEG2RAD)), qTrue));
    }

    // --- hand everything to the UI controller ----------------------------------
    AppState::Inputs in{};
    in.now             = now;
    in.haveImu         = g_haveImu;
    // Orientation calibration 0..3: the BNO055 system-calibration level.
    in.calib           = g_haveImu ? static_cast<u8>(imu.accuracy()) : 0;
    in.fixValid        = fix.valid;
    in.sats            = fix.satellites;
    in.haveBearing     = haveBrg;
    in.distanceM       = distM;
    in.haveBattery     = g_haveBattery;
    in.battPct         = g_battPct;
    in.chargeRatePctHr = g_battRate;
    in.charging        = g_charging;

    app.update(in);
    app.draw(display, qShow, markers, markerCount);
}