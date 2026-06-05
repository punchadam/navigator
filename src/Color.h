#pragma once

#include <cstdint>
#include "shorthand.h"

// RGB565 color math, shared by the 2-D Canvas and the 3-D renderer. One home
// for packing, brightness scaling, and 565<->888 conversion, so the same
// formula isn't copy-pasted across translation units.
namespace color {

// 8-bit-per-channel color: the un-quantized base for lit 3-D facets.
struct Rgb { u8 r, g, b; };

inline float clamp01(float f) { return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f); }

// Pack 8-bit channels into RGB565 (the panel's native format). constexpr so it
// can seed compile-time color tables.
constexpr u16 pack(u8 r, u8 g, u8 b) {
  return u16(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Expand RGB565 back to RGB888, low bits replicated to fill the range.
inline Rgb unpack(u16 c) {
  const u8 r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
  return { u8((r << 3) | (r >> 2)),
           u8((g << 2) | (g >> 4)),
           u8((b << 3) | (b >> 2)) };
}

// Scale an RGB565 color's brightness by f in [0,1].
inline u16 scale(u16 c, float f) {
  f = clamp01(f);
  const u8 r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
  return u16((u8(r * f) << 11) | (u8(g * f) << 5) | u8(b * f));
}

// Shade an RGB888 base by f in [0,1], returning RGB565.
inline u16 shade(const Rgb& c, float f) {
  f = clamp01(f);
  return pack(u8(c.r * f), u8(c.g * f), u8(c.b * f));
}

} // namespace color