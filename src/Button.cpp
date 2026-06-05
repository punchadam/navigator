#include "Button.h"

Button::Button(u8 pin,
               u8 pm,
               u8 activeLevel,
               u32 debounceMs,
               u32 longPressMs,
               u32 powerOffMs)
    : _pin(pin)
    , _pinModeVal(pm)
    , _activeLevel(activeLevel)
    , _debounceMs(debounceMs)
    , _longPressMs(longPressMs)
    , _powerOffMs(powerOffMs)
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
            _wasLong    = false;
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
            _wasLong = true;
            enterState(State::LongPressed);
            fire(_onLongPress);
        }
        break;

    case State::LongPressed:
        if (!raw) {
            _changeTime = millis();
            enterState(State::ReleaseDebounce);
        } else if (_onPowerOff && _powerOffMs > _longPressMs &&
                   millis() - _stateEnter >= _powerOffMs - _longPressMs) {
            enterState(State::PowerOff);
            fire(_onPowerOff);
        }
        break;

    case State::PowerOff:
        if (!raw) {
            enterState(State::PowerOffIdle);
        }
        break;

    case State::PowerOffIdle:
        if (raw) {
            if (!_wakeEdge) {
                _wakeEdge   = true;
                _changeTime = millis();
            } else if (millis() - _changeTime >= _debounceMs) {
                _wakeEdge = false;
                fire(_onWakeUp);
                enterState(State::PowerOffWake);
            }
        } else {
            _wakeEdge = false;
        }
        break;

    case State::PowerOffWake:
        // consume the wake press - go to Released with no callbacks
        if (!raw) {
            enterState(State::Released);
        }
        break;

    case State::ReleaseDebounce:
        if (raw) {
            // bounced back, treat as still held
            enterState(_stateEnter >= _longPressMs ? State::LongPressed
                                                    : State::Pressed);
        } else if (millis() - _changeTime >= _debounceMs) {
            enterState(State::Released);
            if (!_wasLong) fire(_onShortPress);
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

bool Button::rawPressed() const {
    return readRaw();
}

bool Button::isPressed() const {
    return _state == State::Pressed     ||
           _state == State::LongPressed ||
           _state == State::PowerOff    ||
           _state == State::PowerOffWake;
}

u32 Button::heldMs() const {
    return millis() - _stateEnter;
}