// HelloWorld.cpp - minimal display diagnostic build.
//
// Exercises the GC9A01 directly via LGFX with no application code in the path:
// no AppState, no BNO055, no GPS, no Battery, no Button. If the display shows
// text here but not in the main build, the driver config is fine and the bug is
// in application startup ordering.
//
// Build target: [env:hello_world]
// Flash and check Serial @ 115200 for init confirmation.

#include <Arduino.h>
#include "LGFX_Config.hpp"  // defines LGFX, LGFX_USE_V1, pulls in LovyanGFX

static LGFX        lcd;
static LGFX_Sprite fb{ &lcd };

static constexpr int SCREEN = 240;
static constexpr int CX = SCREEN / 2;
static constexpr int CY = SCREEN / 2;

void setup() {
    Serial.begin(115200);
    Serial.println("[HW] init display");

    lcd.init();
    lcd.setRotation(0);
    lcd.setColorDepth(16);

    fb.setColorDepth(16);
    fb.setSwapBytes(true);
    fb.createSprite(SCREEN, SCREEN);

    fb.fillSprite(TFT_BLACK);
    fb.setTextColor(TFT_WHITE);
    fb.setTextDatum(textdatum_t::middle_center);
    fb.setTextSize(3);
    fb.drawString("Hello", CX, CY - 20);
    fb.drawString("World", CX, CY + 20);
    fb.pushSprite(0, 0);

    Serial.println("[HW] done");
}

void loop() {}
