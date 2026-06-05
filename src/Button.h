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
//
//      void loop() { btn.update(); }

class Button {
public:
    using Callback = std::function<void()>;

    explicit Button(u8 pin,
                    u8 pinMode = INPUT_PULLUP,
                    u8 activeLevel = LOW,
                    u32 debounceMs = 20,
                    u32 longPressMs = 600);

    void begin();

    void update();

    // Fired once when first pressed after debounce
    Button& onPress(Callback cb)     { _onPress     = cb; return *this; }

    // fired once when released after debounce
    Button& onRelease(Callback cb)   { _onRelease   = cb; return *this; }

    // fired once if held longer than longPressMs.
    Button& onLongPress(Callback cb) { _onLongPress = cb; return *this; }

    bool isPressed()  const { return _state == State::Pressed || _state == State::LongPressed; }
    bool isReleased() const { return !isPressed(); }

    // how long in current state (ms)
    u32 heldMs() const;

private:
    enum class State : u8 {
        Released,
        Debouncing,     // saw a change, waiting for bounce to settle
        Pressed,
        LongPressed,    // held past longPressMs; suppresses repeat long-press
        ReleaseDebounce // button physically released, settling
    };

    const u8  _pin;
    const u8  _pinModeVal;
    const u8  _activeLevel;
    const u32 _debounceMs;
    const u32 _longPressMs;

    State    _state       = State::Released;
    bool     _rawPressed  = false;  // raw (un-debounced) reading
    u32 _changeTime  = 0;      // millis() when last state transition began
    u32 _stateEnter  = 0;      // millis() when current State was entered

    Callback _onPress;
    Callback _onRelease;
    Callback _onLongPress;

    bool readRaw() const;
    void enterState(State s);
    void fire(const Callback& cb) const;
};