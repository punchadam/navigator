#include "Button.h"

Button::Button(u8 pin,
               u8 pm,
               u8 activeLevel,
               u32 debounceMs,
               u32 longPressMs)
    : _pin(pin)
    , _pinModeVal(pm)
    , _activeLevel(activeLevel)
    , _debounceMs(debounceMs)
    , _longPressMs(longPressMs)
{}

void Button::begin() {
    pinMode(_pin, _pinModeVal);
    _rawPressed = readRaw();
    enterState(State::Released);
}

void Button::update() {
    const bool raw = readRaw();

    switch (_state) {

    case State::Released:
        if (raw) {
            _changeTime = millis();
            enterState(State::Debouncing);
        }
        break;

    case State::Debouncing:
        if (!raw) {
            // bounced back, restart
            enterState(State::Released);
        } else if (millis() - _changeTime >= _debounceMs) {
            enterState(State::Pressed);
            fire(_onPress);
        }
        break;

    case State::Pressed:
        if (!raw) {
            _changeTime = millis();
            enterState(State::ReleaseDebounce);
        } else if (millis() - _stateEnter >= _longPressMs) {
            enterState(State::LongPressed);
            fire(_onLongPress);
        }
        break;

    case State::LongPressed:
        // wait for release bc long-press already fired
        if (!raw) {
            _changeTime = millis();
            enterState(State::ReleaseDebounce);
        }
        break;

    case State::ReleaseDebounce:
        if (raw) {
            // bounced back, treat as still held
            enterState(_stateEnter >= _longPressMs ? State::LongPressed
                                                    : State::Pressed);
        } else if (millis() - _changeTime >= _debounceMs) {
            enterState(State::Released);
            fire(_onRelease);
        }
        break;
    }
}

bool Button::readRaw() const {
    return digitalRead(_pin) == _activeLevel;
}

void Button::enterState(State s) {
    _state      = s;
    _stateEnter = millis();
}

void Button::fire(const Callback& cb) const {
    if (cb) cb();
}

u32 Button::heldMs() const {
    return millis() - _stateEnter;
}