#pragma once

#include <cstdint>
#include <math.h>
#include "LGFX_Config.hpp"
#include "shorthand.h"
#include "Color.h"

// A thin 2-D drawing surface over the display's off-screen framebuffer.
// Coordinates are screen pixels: origin top-left, +x right, +y down. This is
// deliberately *not* the 3-D pipeline — nothing here is projected or tilted, it
// draws flat on the glass, on top of the finished compass frame.
//
// You never construct this yourself. Display hands one to your overlay
// callback (see Display::Overlay), already pointed at the live frame:
//
//     disp.render(q, markers, n, [&](Canvas& ui) {
//         ui.fillRoundRect(8, 8, 96, 26, 6, ui.rgb(0, 0, 0));
//         ui.text("WAYPOINT", 14, 14, ui.rgb(255, 200, 60));
//         ui.arrow(ui.centerX(), 210, ui.centerX(), 160, ui.rgb(0, 255, 180));
//     });
//
// Everything draws into the same sprite the 3-D scene used, so your UI
// composites over it and ships to the panel in the same pushSprite().
class Canvas {
public:
  explicit Canvas(LGFX_Sprite& fb) : _fb(fb) {}

  // --- surface info ----------------------------------------------------------
  int width()   const { return _fb.width(); }
  int height()  const { return _fb.height(); }
  int centerX() const { return _fb.width()  / 2; }
  int centerY() const { return _fb.height() / 2; }

  // --- color -----------------------------------------------------------------
  // Ergonomic alias for color::pack so overlay code reads ui.rgb(r, g, b).
  static constexpr u16 rgb(u8 r, u8 g, u8 b) {
    return color::pack(r, g, b);
  }

  // --- fills / clears --------------------------------------------------------
  // Paint the whole frame (e.g. a mode that wants a pure-2-D screen).
  void fill(u16 c) { _fb.fillSprite(c); }

  // --- primitives ------------------------------------------------------------
  void pixel(int x, int y, u16 c)                          { _fb.drawPixel(x, y, c); }
  void line(int x0, int y0, int x1, int y1, u16 c)         { _fb.drawLine(x0, y0, x1, y1, c); }
  void hLine(int x, int y, int w, u16 c)                   { _fb.drawFastHLine(x, y, w, c); }
  void vLine(int x, int y, int h, u16 c)                   { _fb.drawFastVLine(x, y, h, c); }

  void rect(int x, int y, int w, int h, u16 c)             { _fb.drawRect(x, y, w, h, c); }
  void fillRect(int x, int y, int w, int h, u16 c)         { _fb.fillRect(x, y, w, h, c); }
  void roundRect(int x, int y, int w, int h, int r, u16 c) { _fb.drawRoundRect(x, y, w, h, r, c); }
  void fillRoundRect(int x, int y, int w, int h, int r, u16 c) { _fb.fillRoundRect(x, y, w, h, r, c); }

  void circle(int x, int y, int r, u16 c)                  { _fb.drawCircle(x, y, r, c); }
  void fillCircle(int x, int y, int r, u16 c)              { _fb.fillCircle(x, y, r, c); }

  void triangle(int x0, int y0, int x1, int y1, int x2, int y2, u16 c) {
    _fb.drawTriangle(x0, y0, x1, y1, x2, y2, c);
  }
  void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, u16 c) {
    _fb.fillTriangle(x0, y0, x1, y1, x2, y2, c);
  }

  // Filled ring / wedge. Angles in degrees, 0 = 3 o'clock, growing clockwise
  // (LovyanGFX convention). r0 = inner radius, r1 = outer.
  void wedge(int x, int y, int r0, int r1, float a0, float a1, u16 c) {
    _fb.fillArc(x, y, r0, r1, a0, a1, c);
  }

  // --- text ------------------------------------------------------------------
  template <typename Font>
  void setFont(const Font* f)          { _fb.setFont(f); }
  void setTextSize(float s)            { _fb.setTextSize(s); }
  void setTextDatum(textdatum_t d)     { _fb.setTextDatum(d); }
  int  textWidth(const char* s)        { return _fb.textWidth(s); }
  int  fontHeight()                    { return _fb.fontHeight(); }

  // Transparent background.
  void text(const char* s, int x, int y, u16 fg) {
    _fb.setTextColor(fg);
    _fb.drawString(s, x, y);
  }
  // Opaque background.
  void text(const char* s, int x, int y, u16 fg, u16 bg) {
    _fb.setTextColor(fg, bg);
    _fb.drawString(s, x, y);
  }

  // --- a flat 2-D arrow helper (handy for mode glyphs / direction hints) -----
  // Draws the shaft (x0,y0)->(x1,y1) with a filled head at the (x1,y1) end.
  void arrow(int x0, int y0, int x1, int y1, u16 c,
             float headLen = 12.0f, float headHalfW = 7.0f) {
    _fb.drawLine(x0, y0, x1, y1, c);
    float dx = float(x1 - x0), dy = float(y1 - y0);
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-3f) return;
    float ux = dx / len, uy = dy / len;                  // unit shaft direction
    float bx = x1 - ux * headLen, by = y1 - uy * headLen; // head base center
    float px = -uy, py = ux;                              // perpendicular
    _fb.fillTriangle(x1, y1,
                     int(bx + px * headHalfW), int(by + py * headHalfW),
                     int(bx - px * headHalfW), int(by - py * headHalfW), c);
  }

  // --- curved text along the dial's bezel (for a round panel) ----------------
  // Lays `s` along a circular arc of `radius` px about (cx,cy), centered on
  // `centerDeg` (clockwise from 12 o'clock). up=true orients glyphs for the TOP
  // of the dial (reads L->R across the top); up=false for the BOTTOM (reads
  // L->R across the bottom, smile-side up). Each glyph is rendered to a scratch
  // sprite and rotated tangent to the arc.
  //
  // Cost: one small sprite alloc per call + a rotated blit per glyph. Fine for a
  // couple of short HUD strings per frame on an S3; don't feed it paragraphs.
  template <typename Font>
  void arcText(const char* s, int cx, int cy, float radius, float centerDeg,
               u16 fg, bool up, const Font* font, float tracking = 1.0f) {
    if (!s || !*s) return;

    // Chroma key for transparent compositing. Built-in GFX fonts are 1-bit, so
    // there are no in-between colors to bleed the key — just don't pass pure
    // green as `fg`.
    constexpr u16 KEY = 0x07E0;

    LGFX_Sprite glyph(&_fb);
    glyph.setColorDepth(16);
    glyph.setFont(font);
    const int box = glyph.fontHeight() + 8;        // square, holds the widest glyph
    glyph.createSprite(box, box);
    glyph.setFont(font);
    glyph.setTextDatum(textdatum_t::middle_center);
    glyph.setPivot(box * 0.5f, box * 0.5f);

    const float pxToDeg = 180.0f / (float(M_PI) * radius);   // arc length -> angle
    const float dir     = up ? 1.0f : -1.0f;

    // Total angular span, to center the string on centerDeg.
    float total = 0.0f;
    for (const char* p = s; *p; ++p) {
      char c[2] = { *p, 0 };
      total += (glyph.textWidth(c) + tracking) * pxToDeg;
    }

    float ang = centerDeg - dir * total * 0.5f;    // leading edge
    for (const char* p = s; *p; ++p) {
      char c[2] = { *p, 0 };
      const float w   = (glyph.textWidth(c) + tracking) * pxToDeg;
      const float mid = ang + dir * w * 0.5f;      // this glyph's center angle
      const float t   = mid * float(M_PI) / 180.0f;

      const float gx  = cx + radius * sinf(t);
      const float gy  = cy - radius * cosf(t);
      const float rot = up ? mid : (mid - 180.0f); // upright at the dial reference

      glyph.fillSprite(KEY);
      glyph.setTextColor(fg);
      glyph.drawString(c, box / 2, box / 2);
      // Trailing arg is the transparent color. (LovyanGFX angle is clockwise; if
      // your build curves the wrong way, negate `rot`.)
      glyph.pushRotateZoom(&_fb, gx, gy, rot, 1.0f, 1.0f, KEY);

      ang += dir * w;
    }
    glyph.deleteSprite();
  }

  // Escape hatch: raw sprite for anything not wrapped above.
  LGFX_Sprite& raw() { return _fb; }

private:
  LGFX_Sprite& _fb;
};