#pragma once

#include <functional>
#include "shorthand.h"

//  single-button debounced driver with event callbacks
//
//  Usage:
//      Button btn(PIN, INPUT_PULLUP, LOW);   // active-low with internal pull-up
//      btn.onPress([]{ Serial.println("pressed"); });
//      btn.onRelease([]{ Serial.println("released"); });
//      btn.onLongPress([]{ Serial.println("long press!"); });
//      btn.onPowerOff([]{ /* blank display, sleep ICs */ });
//      btn.onWakeUp([]{ /* re-init display, wake ICs */ });
//
//      void loop() { btn.update(); }
//
//  Power-off flow:
//      Hold >= longPressMs  -> onLongPress fires (normal)
//      Hold >= powerOffMs   -> onPowerOff fires; enters PowerOff state
//      Release after that   -> PowerOffIdle; device is dark
//      Any new press        -> onWakeUp fires; press is consumed (no short/long callbacks)

class Button {
public:
    using Callback = std::function<void()>;

    explicit Button(u8 pin,
                    u8 pinMode = INPUT_PULLUP,
                    u8 activeLevel = LOW,
                    u32 debounceMs = 20,
                    u32 longPressMs = 600,
                    u32 powerOffMs = 3000);

    void begin();

    void update();

    // Fired once when first pressed after debounce
    Button& onPress(Callback cb)      { _onPress     = cb; return *this; }

    // Fired once on release only when the press was shorter than longPressMs.
    // Use this for mode-switch actions so holds don't accidentally cycle modes.
    Button& onShortPress(Callback cb) { _onShortPress = cb; return *this; }

    // Fired once when released after debounce
    Button& onRelease(Callback cb)    { _onRelease   = cb; return *this; }

    // Fired once if held longer than longPressMs (and shorter than powerOffMs)
    Button& onLongPress(Callback cb)  { _onLongPress = cb; return *this; }

    // Fired once when held past powerOffMs (only if onPowerOff is registered)
    Button& onPowerOff(Callback cb)   { _onPowerOff  = cb; return *this; }

    // Fired once when the first press after a power-off is confirmed; the press
    // itself is consumed and will not trigger short/long press callbacks
    Button& onWakeUp(Callback cb)     { _onWakeUp    = cb; return *this; }

    bool isPressed()  const;
    bool isReleased() const { return !isPressed(); }

    // Instantaneous, undebounced pin state. Use this (not isPressed(), which is
    // state-machine derived) when gating power decisions on the physical line -
    // e.g. only entering light sleep once the button is actually let go.
    bool rawPressed() const;

    // how long in current state (ms)
    u32 heldMs() const;

private:
    enum class State : u8 {
        Released,
        Debouncing,      // saw a change, waiting for bounce to settle
        Pressed,
        LongPressed,     // held past longPressMs; suppresses repeat long-press
        ReleaseDebounce, // button physically released, settling
        PowerOff,        // held past powerOffMs; waiting for physical release
        PowerOffIdle,    // released after power-off; waiting for a wake press
        PowerOffWake,    // wake press confirmed; waiting for release (no callbacks)
    };

    const u8  _pin;
    const u8  _pinModeVal;
    const u8  _activeLevel;
    const u32 _debounceMs;
    const u32 _longPressMs;
    const u32 _powerOffMs;

    State _state      = State::Released;
    bool  _rawPressed = false;
    u32   _changeTime = 0;   // millis() when last edge/debounce began
    u32   _stateEnter = 0;   // millis() when current State was entered

    Callback _onPress;
    Callback _onShortPress;
    Callback _onRelease;
    Callback _onLongPress;
    Callback _onPowerOff;
    Callback _onWakeUp;

    bool _wasLong  = false;
    bool _wakeEdge = false;  // true while debouncing a rising edge in PowerOffIdle

    bool readRaw() const;
    void enterState(State s);
    void fire(const Callback& cb) const;
};