// Slideshow.cpp - picture-folder bookkeeping for the arrival stage.
//
// All this does is own a sorted list of image paths on LittleFS and step
// through them on a timer. The decode/blit/pan/fade lives in Display::showSlide;
// the proximity decision lives in main.

#include "Slideshow.h"
#include "config.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>
#include <strings.h>   // strcasecmp

namespace {

bool isImage(const char* name) {
    const char* dot = strrchr(name, '.');
    if (!dot) return false;
    return !strcasecmp(dot, ".jpg")  || !strcasecmp(dot, ".jpeg")
        || !strcasecmp(dot, ".png")  || !strcasecmp(dot, ".bmp");
}

// Basename of a path, tolerating cores whose File::name() already includes the
// directory and those that return a bare filename.
const char* baseName(const char* path) {
    const char* slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

} // namespace

bool Slideshow::begin(const char* dir) {
    // format-on-fail: a blank or first-boot flash still mounts cleanly (just with
    // zero images) instead of leaving the stage permanently dead.
    if (!LittleFS.begin(/*formatOnFail=*/true, /*basePath=*/"/littlefs",
                        /*maxOpenFiles=*/10,   /*partitionLabel=*/"littlefs")) {
        Serial.println("[slideshow] LittleFS mount FAILED - no photos available");
        _mounted = false;
        return false;
    }
    _mounted = true;
    _count   = 0;

    File d = LittleFS.open(dir);
    if (!d || !d.isDirectory()) {
        Serial.printf("[slideshow] folder %s missing - drop 240x240 images there + `pio run -t uploadfs`\n", dir);
        return true;
    }

    for (File f = d.openNextFile(); f && _count < MAX_IMAGES; f = d.openNextFile()) {
        if (f.isDirectory()) {
            Serial.printf("[slideshow] skip dir: %s\n", f.name());
            continue;
        }
        const char* raw  = f.name();
        const char* base = baseName(raw);
        if (!isImage(base)) {
            Serial.printf("[slideshow] skip (not image): %s\n", raw);
            continue;
        }
        snprintf(_paths[_count], MAX_PATH, "%s/%s", dir, base);
        Serial.printf("[slideshow] queued[%u]: %s\n", (unsigned)_count, _paths[_count]);
        ++_count;
    }

    // Alphabetical order so the loop is deterministic and the user controls
    // sequence by filename (01_..., 02_..., etc.). Insertion sort; the list is tiny.
    for (size_t i = 1; i < _count; ++i) {
        char key[MAX_PATH];
        snprintf(key, MAX_PATH, "%s", _paths[i]);
        size_t j = i;
        while (j > 0 && strcasecmp(_paths[j - 1], key) > 0) {
            snprintf(_paths[j], MAX_PATH, "%s", _paths[j - 1]);
            --j;
        }
        snprintf(_paths[j], MAX_PATH, "%s", key);
    }

    Serial.printf("[slideshow] %u image(s) loaded from %s\n", (unsigned)_count, dir);
    return true;
}

const char* Slideshow::frame(u32 now, u32 intervalMs) {
    if (_count == 0) return nullptr;

    if (_fresh) {
        _fresh       = false;
        _index       = 0;
        _lastAdvance = now;
    } else if (now - _lastAdvance >= intervalMs) {
        _lastAdvance = now;
        _index       = (_index + 1) % _count;
    }
    return _paths[_index];
}

float Slideshow::fadeAlpha(u32 now, u32 intervalMs) const {
    if (_count == 0 || _fresh || intervalMs == 0) return 0.0f;
    u32 elapsed = (u32)(now - _lastAdvance);
    if (elapsed > intervalMs) elapsed = intervalMs;
    // Clamp fade to half the interval so short FRAME_MS still works.
    const u32 fade = (SlideshowConfig::FADE_MS < intervalMs / 2)
                   ? SlideshowConfig::FADE_MS : intervalMs / 2;
    if (elapsed < fade)                return (float)elapsed / (float)fade;
    if (elapsed > intervalMs - fade)   return (float)(intervalMs - elapsed) / (float)fade;
    return 1.0f;
}

void Slideshow::rewind(u32 now) {
    _index       = 0;
    _lastAdvance = now;
    _fresh       = true;
}
