// main.cpp - waypoint compass
//
// Reads absolute orientation from the BNO055 and a GPS fix, folds in
// magnetic declination, computes the great-circle bearing to a waypoint,
// and hands the result to Display via AppState.
//
// Division of labor:
//   main     - hardware + navigation math (orientation smoothing, declination
//              fold, bearing/distance). Owns the button and feeds AppState.
//   AppState - UI controller: mode, units, animation timing, the waypoint
//              table, and every formatted HUD string.
//   Display  - pure renderer: (qShow, markers, HudState) -> pixels.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <Preferences.h>

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
#include <LittleFS.h>
#include "Slideshow.h"

using namespace ESP32S3Pinout;
using namespace MathConst;
using OrientationConfig::SMOOTH;
using OrientationConfig::MOUNT_CORRECTION;
using RuntimeConfig::FRAME_MS;
using RuntimeConfig::BATT_SOC_MS;
using RuntimeConfig::IMU_MS;
using RuntimeConfig::GPS_MS;

// -- Hardware --
BNO055 bno(Wire, 0x28, PIN_BNO055_RESET);
GPS gps;
Display display;
Battery battery;
Slideshow slideshow;
Button button(PIN_BUTTON, INPUT, HIGH); // pulled down -> active HIGH

// -- App state --
bool g_poweredOff = false;
bool g_batteryDead = false;
AppState app;

// -- NVS persistence (survives power loss, not just deep sleep) --
static void nvsSave() {
    Preferences p;
    p.begin("nav", /*readOnly=*/false);
    AppState::Persist s = app.save();
    p.putDouble("wpBLat", s.wpBLat);
    p.putDouble("wpBLon", s.wpBLon);
    p.end();
}

static void nvsLoad() {
    Preferences p;
    p.begin("nav", /*readOnly=*/true);
    AppState::Persist s;
    s.wpBLat = p.getDouble("wpBLat", WaypointDefaults::B_LAT);
    s.wpBLon = p.getDouble("wpBLon", WaypointDefaults::B_LON);
    p.end();
    app.restore(s);
}

Quaternion g_orient { 1, 0, 0, 0 }; // smoothed device orientation
bool g_orientInit = false;
bool g_haveImu = false;
u8 g_calibSys = 0; // BNO055 system calibration status 0..3
bool g_haveBattery = false;
float g_battPct = -1.0f;
float g_battVoltage = -1.0f;
float g_battRate = 0.0f;
bool g_charging = false;

// Calibration offsets - saved on power-off only when the BNO is fully
// calibrated; restored immediately after bno.begin() so the sensor starts
// pre-calibrated on the next boot. The "bnoOff" blob is 22 bytes (11*int16).
static void nvsBattSave(float pct, bool charging) {
    Preferences p;
    p.begin("nav", /*readOnly=*/false);
    p.putFloat("battPct", pct);
    p.putBool("charging", charging);
    p.end();
}

struct NvsBatt { float pct; bool charging; };
static NvsBatt nvsBattLoad() {
    Preferences p;
    p.begin("nav", /*readOnly=*/true);
    NvsBatt b;
    b.pct = p.getFloat("battPct", -1.0f);
    b.charging = p.getBool("charging", false);
    p.end();
    return b;
}

static void nvsCalibSave() {
    if (!g_haveImu) return;
    adafruit_bno055_offsets_t offsets;
    if (!bno.getCalibOffsets(offsets)) return; // not fully calibrated - keep existing NVS data
    Preferences p;
    p.begin("nav", /*readOnly=*/false);
    p.putBytes("bnoOff", &offsets, sizeof(offsets));
    p.end();
    if (DebugConfig::IMU) Serial.println("[imu] calibration offsets saved to NVS");
}

static void nvsCalibLoad() {
    if (!g_haveImu) return;
    Preferences p;
    p.begin("nav", /*readOnly=*/true);
    adafruit_bno055_offsets_t offsets;
    const size_t n = p.getBytes("bnoOff", &offsets, sizeof(offsets));
    p.end();
    if (n != sizeof(offsets)) return; // no saved data yet
    bno.setCalibOffsets(offsets);
    if (DebugConfig::IMU) Serial.println("[imu] calibration offsets restored from NVS");
}

// -- Navigation math --
static Quaternion nlerp(const Quaternion& a, Quaternion b, float t) {
    if (dot(Vec3{a.x, a.y, a.z}, Vec3{b.x, b.y, b.z}) + a.w * b.w < 0.0f) {
        b = { -b.w, -b.x, -b.y, -b.z };
    }
    Quaternion r{ a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
                  a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
    return normalize(r);
}

static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
    const double y = sin(dl) * cos(p2);
    const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) * RAD2DEG;
    if (b < 0.0) b += 360.0;
    return b;
}

static double distanceM(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    const double dp = (lat2 - lat1) * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
    const double a  = sin(dp / 2) * sin(dp / 2)
                    + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return NavConfig::EARTH_RADIUS_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// -- Setup --
void setup() {
    // Disable WiFi and Bluetooth radios immediately - never used, pure power waste.
    WiFi.mode(WIFI_OFF);

    Serial.begin(115200);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    display.begin();
    gps.begin();
    app.begin(millis());
    nvsLoad(); // overwrite defaults with persisted waypoint B + units
    slideshow.begin(SlideshowConfig::FOLDER);

    g_haveImu = bno.begin();
    if (DebugConfig::IMU)
        Serial.println(g_haveImu ? "[imu] BNO055 ok" : "[imu] BNO055 not found  (static rose)");
    nvsCalibLoad(); // restore saved calibration offsets (no-op if none saved yet)
    g_haveBattery = battery.begin(Wire, /*initBus=*/false);
    if (g_haveBattery) {
        const bool didReset = battery.didReset(); // clears RI bit regardless
        const NvsBatt nb = nvsBattLoad();
        if (nb.pct >= 0.0f) {
            // Always seed SOC from NVS, not just on a hard reset. The Adafruit
            // begin() may issue a QuickStart internally (OCV re-estimate) which
            // clears RI before didReset() sees it, causing a 5-10% drift even
            // when the chip didn't fully lose power. A short off/on cycle makes
            // NVS always more accurate than a cold OCV guess.
            battery.writeSoc(nb.pct);
            g_battPct  = nb.pct;
            g_charging = nb.charging;
            if (DebugConfig::BATTERY)
                Serial.printf("[bat] %s - restored SOC %.1f%%  charging=%s\n",
                              didReset ? "chip reset" : "boot", nb.pct,
                              g_charging ? "yes" : "no");
        }
    }

    button.begin();
    button.onShortPress([]{
        app.cycleMode(millis());
    });
    button.onLongPress([]{
        const GpsFix f = gps.read();
        app.onLongPress(millis(), f.valid, f.latitude, f.longitude);
        nvsSave(); // persist waypoint B / units immediately after each long press
    });
    button.onPowerOff([]{
        app.undoLongPress(millis());
        nvsSave();       // save the undone state so a power-off long press doesn't stick
        nvsCalibSave();  // persist BNO calibration offsets if fully calibrated
        if (g_haveBattery && g_battPct >= 0.0f) nvsBattSave(g_battPct, g_charging);
        display.sleep();
        if (g_haveImu) bno.suspend();
        gps.sleep();
        g_poweredOff = true;
        Serial.println("[power] sleep - display/gauge/IMU/GPS down, entering deep sleep on release");
    });

    // Woken from a power-off? The wake is a level-HIGH GPIO trigger, so the
    // button is still physically held as we boot. Swallow that press - wait for a
    // clean release before the loop starts ticking the button - so it doesn't
    // register as a stray short-press and cycle the mode the moment we wake.
    // rawPressed() reads the pin directly without advancing the state machine.
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("[power] wake - booted from deep sleep, swallowing wake press");
        while (button.rawPressed()) delay(10);
    }
}

// -- Loop --
void loop() {
    button.update();

    if (g_poweredOff) {
        // Power-off uses DEEP sleep, not light sleep. The slideshow put real data
        // in PSRAM (the second 240x240 decode sprite) and mounted LittleFS on the
        // SPI flash; light sleep on this S3 can't reliably restore the octal
        // flash/PSRAM on resume, so the core hung in esp_light_sleep_start() and
        // never woke (no GPIO wake, no timer wake - the symptom you saw). Deep
        // sleep sidesteps it: the chip powers down and a button press triggers a
        // full reboot through setup(), which re-inits the display, sensors and
        // filesystem from a clean state and restores the saved waypoint/units from
        // NVS. Display::sleep() is documented for exactly this (see Display.h).
        //
        // The button is still HIGH when power-off fires and the wake is level-HIGH,
        // so wait for a clean release before arming - otherwise the very hold that
        // triggered power-off would immediately wake us again. While still held we
        // just keep ticking button.update() (rule below) until the pin reads LOW.
        if (!button.rawPressed()) {
            // Debounce the release before arming the level-HIGH wake source.
            // Without this, a bounce spike on the falling edge immediately
            // triggers EXT1 and the chip wakes itself back up.
            delay(100);
            if (!button.rawPressed()) {
                esp_sleep_enable_ext1_wakeup(1ULL << PIN_BUTTON, ESP_EXT1_WAKEUP_ANY_HIGH);
                esp_deep_sleep_start(); // never returns; the next press reboots us
            }
        }
        return; // still held (or bounced) - keep looping until pin is stable low
    }

    if (g_batteryDead) {
        // Peripherals are already off. Poll the gauge slowly; restart when safe.
        // delay(100) between ticks keeps the CPU mostly idle without light sleep
        // (which hangs on this S3 with LittleFS mounted - same issue as power-off).
        static u32 lastDeadPoll = 0;
        const u32 t = millis();
        if (g_haveBattery && t - lastDeadPoll >= BatteryConfig::DEAD_POLL_MS) {
            lastDeadPoll = t;
            battery.trackVoltage();
            const float v = battery.voltage();
            if (DebugConfig::BATTERY) Serial.printf("[batt] dead poll %.2fV\n", v);
            if (v >= BatteryConfig::V_RECOVER) {
                Serial.printf("[batt] voltage recovered (%.2fV) - restarting\n", v);
                delay(50); // let serial flush before the restart
                ESP.restart();
            }
        }
        delay(100);
        return;
    }

    gps.update();

    const u32 now = millis();

    // -- battery SOC poll (3 s) --
    static u32 lastBattSoc = 0;
    if (now - lastBattSoc >= BATT_SOC_MS) {
        lastBattSoc = now;
        if (g_haveBattery) {
            battery.trackVoltage();
            g_battPct     = battery.percent();
            g_battVoltage = battery.voltage();
            g_battRate    = battery.chargeRate();
            // Saturating counter debounce: require 3 consecutive agreeing polls
            // (about 9 s) before flipping the charging flag. Filters transient noise
            // from the CRATE register without slowing down real plug/unplug events.
            // Pre-seeded from NVS on first tick so a reboot doesn't zero-cross.
            constexpr i8 CHARGE_THRESH = 3;
            static i8 chargeDebounce = g_charging ? CHARGE_THRESH : -CHARGE_THRESH;
            if (g_battRate > 0.5f || battery.voltageRising())
                chargeDebounce = min((i8)(chargeDebounce + 1), CHARGE_THRESH);
            else if (g_battRate < -1.0f)
                chargeDebounce = max((i8)(chargeDebounce - 1), (i8)-CHARGE_THRESH);
            if      (chargeDebounce >= CHARGE_THRESH)  g_charging = true;
            else if (chargeDebounce <= -CHARGE_THRESH) g_charging = false;

            // Seed gauge to 100% the first time voltage hits the full-charge
            // threshold while charging. Corrects the ModelGauge default 4.20V
            // ceiling to match this cell's actual 4.10V full-charge voltage.
            static bool socSeeded = false;
            if (!socSeeded && g_charging && g_battVoltage >= BatteryConfig::V_FULL) {
                battery.writeSoc(100.0f);
                socSeeded = true;
                if (DebugConfig::BATTERY) Serial.println("[batt] seeded SOC=100% at V_FULL");
            }

            // dead battery: no hardware kill switch, so software-shutdown everything
            if (!g_batteryDead && g_battVoltage > 0.0f && g_battVoltage < BatteryConfig::V_DEAD) {
                g_batteryDead = true;
                Serial.printf("[batt] dead (%.2fV < %.2fV) - shutting down\n",
                              g_battVoltage, BatteryConfig::V_DEAD);
                nvsSave();
                nvsCalibSave();
                nvsBattSave(g_battPct, g_charging);
                display.sleep();
                if (g_haveImu) bno.suspend();
                gps.sleep();
            }
        }
        if (DebugConfig::BATTERY) {
            if (g_haveBattery)
                Serial.printf("[batt] %.2fV  %.1f%%  %+.1f%%/hr  %s\n",
                              battery.voltage(), g_battPct, g_battRate,
                              g_charging ? "charging" : "discharging");
            else
                Serial.println("[batt] not found");
        }
    }

    // -- IMU debug print --
    static u32 lastImu = 0;
    if (DebugConfig::IMU && now - lastImu >= IMU_MS) {
        lastImu = now;
        if (!g_haveImu) {
            Serial.println("[imu] not found");
        } else if (!g_orientInit) {
            Serial.printf("[imu] calib=%u/3  waiting for first quaternion\n", g_calibSys);
        } else {
            float hdg = atan2f(2.0f * (g_orient.w * g_orient.z + g_orient.x * g_orient.y),
                               1.0f - 2.0f * (g_orient.y * g_orient.y + g_orient.z * g_orient.z));
            hdg *= float(MathConst::RAD2DEG);
            if (hdg < 0.0f) hdg += 360.0f;
            Serial.printf("[imu] calib=%u/3  hdg=%5.1f deg  q(%.4f %.4f %.4f %.4f)\n",
                          g_calibSys, hdg,
                          g_orient.w, g_orient.x, g_orient.y, g_orient.z);
        }
    }

    // -- frame gate --
    static u32 lastFrame = 0;
    if (now - lastFrame < FRAME_MS) return;
    lastFrame = now;

    // -- orientation: read, mount-correct, smooth --
    if (g_haveImu) {
        Quaternion q;
        if (bno.readQuaternion(q)) {
            q = normalize(multiply(q, MOUNT_CORRECTION));
            g_orient = g_orientInit ? nlerp(g_orient, q, SMOOTH) : q;
            g_orientInit = true;
        }
        // High-watermark: BNO055 sys calibration oscillates in NDOF mode;
        // once a level is reached, never show it as lower.
        static u8 calibHW = 0;
        const u8 raw = bno.calibSystemStatus();
        if (raw > calibHW) calibHW = raw;
        g_calibSys = calibHW;
    }

    // -- declination fold: device frame -> true-North world frame --
    const Quaternion qTrue = normalize(multiply(gps.declinationQuat(), g_orient));

    const GpsFix fix = gps.read();

    // -- GPS debug print --
    if (DebugConfig::GPS) {
        static u32 lastGps = 0;
        if (now - lastGps >= GPS_MS) {
            lastGps = now;
            const u32 chars = gps.charsProcessed();
            if (chars == 0) {
                Serial.printf("[gps] no data  (RX=%u TX=%u)\n",
                              ESP32S3Pinout::PIN_GPS_TX_ESP_RX, ESP32S3Pinout::PIN_GPS_RX_ESP_TX);
            } else if (!fix.valid) {
                Serial.printf("[gps] searching  sats=%u  chars=%lu\n",
                              fix.satellites, (unsigned long)chars);
            } else {
                Serial.printf("[gps] fix  sats=%u  %.6f, %.6f  alt=%.0fm  decl=%.2f deg%s\n",
                              fix.satellites, fix.latitude, fix.longitude, fix.altitudeM,
                              gps.declinationDegrees(),
                              gps.declinationValid() ? "" : " (est)");
            }
        }
    }

    // -- arrival stage: close to Waypoint A, compass gives up --
    // Independent of mode - A is the device's whole reason to exist, and within
    // about 200 ft the heading/distance to it are just GPS noise. So we drop the
    // navigation UI and loop the photo folder until you walk back out. The
    // hysteresis margin (exit radius > enter radius) stops it flickering when a
    // jittering fix straddles the boundary. No fix means we can't know the
    // distance, so we stay on the compass.
    {
        static bool inSlideshow = false;

        if (fix.valid) {
            const AppState::Waypoint& A = app.waypoint(AppState::Mode::WaypointA);
            const double dA = distanceM(fix.latitude, fix.longitude, A.lat, A.lon);
            if (!inSlideshow && dA <= SlideshowConfig::RADIUS_M) {
                inSlideshow = true;
                slideshow.rewind(now);
            } else if (inSlideshow && dA > SlideshowConfig::RADIUS_M + SlideshowConfig::HYSTERESIS_M) {
                inSlideshow = false;
            }
        } else {
            inSlideshow = false;
        }

        if (inSlideshow) {
            // showSlide caches the decoded image; fade requires a call every
            // frame. Decoding only happens when the path changes.
            const char* img = slideshow.frame(now, SlideshowConfig::FRAME_MS);
            display.showSlide(img, slideshow.fadeAlpha(now, SlideshowConfig::FRAME_MS));
            return; // compass fully suppressed while standing at the waypoint
        }
    }

    // -- per-mode navigation --
    const AppState::Waypoint* wp = app.activeWaypoint();

    Quaternion qShow = qTrue;
    Marker markers[2];
    size_t markerCount = 0;
    bool haveBrg = false;
    double distM = 0.0;

    if (!wp) {
        if (fix.valid) {
            const AppState::Waypoint& A = app.waypoint(AppState::Mode::WaypointA);
            const AppState::Waypoint& B = app.waypoint(AppState::Mode::WaypointB);
            const double ba = bearingDeg(fix.latitude, fix.longitude, A.lat, A.lon);
            const double bb = bearingDeg(fix.latitude, fix.longitude, B.lat, B.lon);
            markers[markerCount++] = Marker{ float(ba * DEG2RAD), A.color, A.tag };
            markers[markerCount++] = Marker{ float(bb * DEG2RAD), B.color, B.tag };
        }
    } else if (fix.valid) {
        const double b = bearingDeg(fix.latitude, fix.longitude, wp->lat, wp->lon);
        distM = distanceM(fix.latitude, fix.longitude, wp->lat, wp->lon);
        haveBrg = true;
        qShow = normalize(multiply(yawQuat(static_cast<float>(b * DEG2RAD)), qTrue));
    } else {
        const float a = float(now) * 0.003f;
        const Quaternion qYaw{ cosf(a * 0.5f), 0.0f, 0.0f, sinf(a * 0.5f) };
        qShow = normalize(multiply(qYaw, qTrue));
    }

    // -- hand everything to the UI controller --
    AppState::Inputs in{};
    in.now = now;
    in.haveImu = g_haveImu;
    in.calibSys = g_calibSys;
    in.fixValid = fix.valid;
    in.gpsTalking = gps.isTalking();
    in.sats = fix.satellites;
    in.haveBearing = haveBrg;
    in.distanceM = distM;
    in.haveBattery = g_haveBattery;
    in.battPct = g_battPct;
    in.battVoltage = g_battVoltage;
    in.charging = g_charging;

    app.update(in);
    app.draw(display, qTrue, qShow, markers, markerCount);
}
