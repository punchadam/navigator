#pragma once

#include "shorthand.h"

// Slideshow - the "you've arrived" stage.
//
// When the device gets within a short radius of the home waypoint the compass
// is useless (see SlideshowConfig in config.h), so main hands the screen over
// to this. Slideshow owns exactly one concern: the picture folder on the
// LittleFS flash filesystem. It mounts the filesystem, scans the folder once at
// boot for images, and on each frame tells the caller which file should be on
// screen - looping the set on a fixed interval. It does no
// drawing (that is Display::showSlide) and no navigation (that is main).
//
// Lifecycle (call all three every frame while in the stage):
//      slideshow.begin("/slideshow");          // once, in setup()
//      ...
//      const char* img = slideshow.frame(now, FRAME_MS);
//      display.showSlide(img, slideshow.fadeAlpha(now, FRAME_MS));
class Slideshow {
public:
    // Mount LittleFS and scan `dir` for image files. Safe to call once at boot.
    // Returns true if the filesystem mounted (even when the folder is empty or
    // missing - frame() will simply return nullptr in that case).
    bool begin(const char* dir);

    bool   ready() const { return _mounted; }
    size_t count() const { return _count; }

    // Which image should be on screen right now. Advances to the next file once
    // `intervalMs` has elapsed since the last change and loops at the end.
    // `now` is millis(). Returns nullptr when the folder holds no images.
    // Call once per frame while in the stage.
    const char* frame(u32 now, u32 intervalMs);

    // Fade brightness: 0.0 = black, 1.0 = full image.
    // Ramps 0->1 over the first FADE_MS and 1->0 over the last FADE_MS.
    float fadeAlpha(u32 now, u32 intervalMs) const;

    // Restart the loop from the first image. Call when (re)entering the stage so
    // every approach opens on the same photo rather than wherever it left off.
    void rewind(u32 now);

private:
    static constexpr size_t MAX_IMAGES = 64;
    static constexpr size_t MAX_PATH   = 64;

    char   _paths[MAX_IMAGES][MAX_PATH] = {};
    size_t _count       = 0;
    size_t _index       = 0;
    u32    _lastAdvance = 0;
    bool   _mounted     = false;
    bool   _fresh       = true;   // first frame() after a rewind shows index 0 now
};
