// Display.cpp - 3D level-compass renderer for a round TFT on ESP32-S3.
//
// Pure renderer: a function of (qShow, markers, HudState) -> pixels. All
// smoothing, declination folding, unit choice, animation clocks and string
// formatting happen upstream (main + AppState); the only state this class owns
// is the off-screen framebuffer plus which pointer to draw and the arrow color.
//
// Model: a gimballed compass. The rose (degree ring + cardinals + markers)
// floats on the world-horizontal plane, so it foreshortens in 3D as the device
// tilts and its North tick always points at true North. A raised, shaded
// pointer - the two-tone needle (compass) or a single-color arrow (waypoint) -
// points at world North / the bearing. A device-fixed lubber line at the top
// lets you read your heading off the rotating rose. A flat 2-D HUD is then
// composited on top: a battery ring hugging the bezel, a curved label that
// rises and fades on mode changes, distance, status, and gesture feedback.
//
// World axes: +Y = North, +X = East, +Z = Up. The screen is the device's XY
// plane; body +Z is the screen normal pointing out toward the viewer.
// A marker at compass angle a (0 = North, clockwise) sits on the level ring at
// world (sin a, cos a, 0).

#include "Display.h"
#include "Color.h"
#include "config.h"

#include <LittleFS.h>
#include <string.h>
#include <strings.h>   // strcasecmp

using color::Rgb;

namespace {

// -- geometry constants (pixels, and unit-ring units where noted) --
constexpr int   SCREEN = 240;
constexpr float CX = SCREEN * 0.5f; // ring center, pixels
constexpr float CY = SCREEN * 0.5f;
constexpr float R  = 104.0f; // ring radius, pixels

constexpr float TICK_MINOR_IN = 0.92f; // unit-ring radii for tick endpoints
constexpr float MARK_TICK_IN  = 0.84f;
constexpr float RING_RHO      = 1.00f;
constexpr float CARDINAL_RHO  = 0.78f; // N/E/S/W letters, inside the ring
constexpr float LABEL_RHO     = 0.72f;
constexpr float MARKER_RHO    = 1.05f; // markers sit just outside the ring

// Needle (unit-ring units; raw-projected so it keeps true 3-D foreshortening).
constexpr float N_LEN = 0.66f; // tip distance from center
constexpr float N_WID = 0.12f; // half-width at the shoulders
constexpr float N_HGT = 0.16f; // crest height above the level plane

// Arrow (waypoint pointer): a single raised, faceted blade pointing at +Y.
constexpr float A_LEN    = 0.72f; // tip distance from center
constexpr float A_HEADL  = 0.34f; // arrowhead length (tip back to barbs)
constexpr float A_HEADW  = 0.27f; // barb half-width
constexpr float A_SHAFTW = 0.085f; // shaft half-width
constexpr float A_TAIL   = 0.34f; // shaft length behind center
constexpr float A_HGT    = 0.16f; // crest height above the level plane

constexpr int MAX_MARKERS = 24;

// Legibility band: full 3-D foreshortening square-on, re-inflated circle edge-on.
constexpr float FLAT_LO = 0.15f;
constexpr float FLAT_HI = 0.45f;

// -- HUD geometry (pixels) --
constexpr float BEZEL_R   = 118.0f; // battery ring outer edge
constexpr float RING_W    = 3.0f; // battery ring thickness
constexpr float ARC_TXT_R = 96.0f; // curved label/distance settled radius
constexpr float ARC_TXT_LO= 132.0f; // ...where it starts, below the bezel

// -- colors (RGB565) --
const u16 COL_BG    = color::pack(  8, 10, 14);
const u16 COL_RING  = color::pack(120,130,140);
const u16 COL_MINOR = color::pack( 70, 78, 88);
const u16 COL_HUB   = color::pack(225,225,225);
const u16 COL_WARN  = color::pack(255, 80, 70); // low battery / no fix
const u16 COL_GOOD  = color::pack( 80,230,130); // full charge

// Needle facet base colors (RGB888, shaded then converted at draw time).
const Rgb NEEDLE_NORTH{ 226, 75, 74 };  // red north half
const Rgb NEEDLE_SOUTH{ 205,206,196 };  // pale south half

// Light direction in the rose's screen frame: upper-left, toward the viewer.
const Vec3 LIGHT = { -0.35f, 0.45f, 0.82f };

// -- small float / vector helpers --
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
inline float smoothstepf(float lo, float hi, float x) {
  float t = clampf((x - lo) / (hi - lo), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
inline Vec3 vsub(const Vec3& a, const Vec3& b) { return { a.x-b.x, a.y-b.y, a.z-b.z }; }
inline Vec3 vnorm(const Vec3& v) {
  float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
  float s = (n > 0.0f) ? 1.0f / n : 0.0f;
  return { v.x*s, v.y*s, v.z*s };
}

inline float depthDim(float depth) {
  float t = clampf(depth * 0.5f + 0.5f, 0.0f, 1.0f); // depth (-1..1) -> (0..1)
  return lerpf(0.45f, 1.0f, t);
}
inline float lambert(const Vec3& nScreen) {
  float diff = nScreen.x*LIGHT.x + nScreen.y*LIGHT.y + nScreen.z*LIGHT.z;
  return clampf(diff, 0.0f, 1.0f) * 0.6f + 0.4f;
}
inline Vec3 levelPoint(float angle, float rho) { // compass angle, in-plane radius
  return { rho * sinf(angle), rho * cosf(angle), 0.0f };
}

// -- projection --
void projLevel(const Quaternion& invQ, const Vec3& world, float readability,
               float& sx, float& sy, float& depth) {
  Vec3 b = rotate(invQ, world); // world -> screen frame
  depth = b.z;
  float rawLen = sqrtf(b.x*b.x + b.y*b.y);
  float u = b.x, v = b.y;
  if (rawLen > 1e-4f) {
    float rho = sqrtf(world.x*world.x + world.y*world.y); // full-circle radius
    float len = lerpf(rho, rawLen, readability); // toward circle as r->0
    float k = len / rawLen;
    u = b.x * k; v = b.y * k;
  }
  sx = CX + R * u;
  sy = CY - R * v; // flip: screen y grows downward
}

void projRaw(const Quaternion& invQ, const Vec3& world, float& sx, float& sy, float& depth) {
  Vec3 b = rotate(invQ, world);
  sx = CX + R * b.x;
  sy = CY - R * b.y;
  depth = b.z;
}

// -- shared 3-D mesh renderer --
struct Tri { Vec3 a, b, c, n; Rgb color; }; // n = outward normal in body space

void renderMesh(LGFX_Sprite& fb, const Quaternion& invQ, const Tri* tris, int n) {
  constexpr int MAX = 32; // needle = 4, arrow = 20
  if (n > MAX) n = MAX;

  struct Proj { float ax, ay, bx, by, cx, cy, z; } p[MAX];
  for (int i = 0; i < n; ++i) {
    float ad, bd, cd;
    projRaw(invQ, tris[i].a, p[i].ax, p[i].ay, ad);
    projRaw(invQ, tris[i].b, p[i].bx, p[i].by, bd);
    projRaw(invQ, tris[i].c, p[i].cx, p[i].cy, cd);
    p[i].z = (ad + bd + cd) / 3.0f;
  }

  int order[MAX];
  for (int i = 0; i < n; ++i) order[i] = i;
  for (int i = 0; i < n - 1; ++i)
    for (int j = i + 1; j < n; ++j)
      if (p[order[j]].z < p[order[i]].z) { int t = order[i]; order[i] = order[j]; order[j] = t; }

  for (int k = 0; k < n; ++k) {
    const int i = order[k];
    const Vec3 ns = vnorm(rotate(invQ, tris[i].n)); // body -> screen frame
    const u16 c = color::shade(tris[i].color, lambert(ns));
    fb.fillTriangle(int(p[i].ax), int(p[i].ay),
                    int(p[i].bx), int(p[i].by),
                    int(p[i].cx), int(p[i].cy), c);
  }
}

// -- the rose (degree ring) --
void drawRing(LGFX_Sprite& fb, const Quaternion& invQ, float readability) {
  for (int deg = 0; deg < 360; deg += 10) {
    const float rad   = deg * float(M_PI) / 180.0f;
    const bool  major = (deg % 90 == 0);
    const float inRho = major ? MARK_TICK_IN : TICK_MINOR_IN;
    const u16 col = major ? COL_RING : COL_MINOR;

    float xi, yi, di, xo, yo, doo;
    projLevel(invQ, levelPoint(rad, inRho),    readability, xi, yi, di);
    projLevel(invQ, levelPoint(rad, RING_RHO), readability, xo, yo, doo);
    fb.drawLine(int(xi), int(yi), int(xo), int(yo), color::scale(col, depthDim(doo)));
  }
}

// -- cardinal letters, riding the rose --
// N/E/S/W sit on the level plane, so they foreshorten and rotate with the rose.
// In compass mode N reads at the top; in waypoint mode the frame is yawed to the
// bearing, so the letters quietly show where the real directions have gone.
// North is tinted to match the needle's red so it always reads as "north".
void drawCardinals(LGFX_Sprite& fb, const Quaternion& invQ, float readability) {
  static const char* L[4] = { "N", "E", "S", "W" };
  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
  for (int i = 0; i < 4; ++i) {
    const float ang = i * float(M_PI) / 2.0f; // 0=N, 90=E, 180=S, 270=W
    float x, y, d;
    projLevel(invQ, levelPoint(ang, CARDINAL_RHO), readability, x, y, d);
    if (readability < 0.7f && d < -0.25f) continue; // hide the far side in 3-D
    const u16 base = (i == 0) ? color::pack(NEEDLE_NORTH.r, NEEDLE_NORTH.g, NEEDLE_NORTH.b)
                               : COL_RING;
    fb.setTextColor(color::scale(base, depthDim(d)));
    fb.drawString(L[i], int(x), int(y));
  }
}

// -- the 3-D needle --
void drawNeedle(LGFX_Sprite& fb, const Quaternion& invQ) {
  const Vec3 V[5] = {
    levelPoint(0.0f,            N_LEN),   // 0: North tip      (+Y)
    levelPoint(float(M_PI),     N_LEN),   // 1: South tip      (-Y)
    levelPoint(-float(M_PI)/2,  N_WID),   // 2: West shoulder  (-X)
    levelPoint( float(M_PI)/2,  N_WID),   // 3: East shoulder  (+X)
    Vec3{ 0.0f, 0.0f, N_HGT }            // 4: crest apex     (+Z)
  };
  struct FaceDef { int a, b, c; Rgb col; };
  const FaceDef fd[4] = {
    { 0, 3, 4, NEEDLE_NORTH },  { 0, 2, 4, NEEDLE_NORTH },
    { 1, 3, 4, NEEDLE_SOUTH },  { 1, 2, 4, NEEDLE_SOUTH },
  };
  Tri tris[4];
  for (int i = 0; i < 4; ++i) {
    const Vec3& A = V[fd[i].a]; const Vec3& B = V[fd[i].b]; const Vec3& C = V[fd[i].c];
    Vec3 n = cross(vsub(B, A), vsub(C, A));
    const Vec3 centroid = { (A.x+B.x+C.x)/3.0f, (A.y+B.y+C.y)/3.0f, (A.z+B.z+C.z)/3.0f };
    if (dot(n, centroid) < 0.0f) { n.x = -n.x; n.y = -n.y; n.z = -n.z; }
    tris[i] = { A, B, C, n, fd[i].col };
  }
  renderMesh(fb, invQ, tris, 4);
}

// -- the 3-D arrow --
void drawArrow(LGFX_Sprite& fb, const Quaternion& invQ, u16 color565) {
  const Rgb base = color::unpack(color565);
  constexpr int RIM = 7;
  const float yNeck = A_LEN - A_HEADL;
  const Vec3 rim[RIM] = {
    { 0.0f,      A_LEN,  0.0f },   // 0 tip
    { A_HEADW,   yNeck,  0.0f },   // 1 right barb
    { A_SHAFTW,  yNeck,  0.0f },   // 2 right neck
    { A_SHAFTW, -A_TAIL, 0.0f },   // 3 right tail
    {-A_SHAFTW, -A_TAIL, 0.0f },   // 4 left tail
    {-A_SHAFTW,  yNeck,  0.0f },   // 5 left neck
    {-A_HEADW,   yNeck,  0.0f },   // 6 left barb
  };
  Vec3 vb[RIM], vt[RIM];
  for (int i = 0; i < RIM; ++i) { vb[i] = rim[i]; vt[i] = { rim[i].x, rim[i].y, A_HGT }; }
  const Vec3 nUp{ 0,0, 1 }, nDn{ 0,0,-1 };
  Tri tris[6 + 2 * RIM]; int nt = 0;
  tris[nt++] = { vt[0], vt[1], vt[6], nUp, base };
  tris[nt++] = { vt[2], vt[3], vt[4], nUp, base };
  tris[nt++] = { vt[2], vt[4], vt[5], nUp, base };
  tris[nt++] = { vb[0], vb[6], vb[1], nDn, base };
  tris[nt++] = { vb[2], vb[4], vb[3], nDn, base };
  tris[nt++] = { vb[2], vb[5], vb[4], nDn, base };
  for (int i = 0; i < RIM; ++i) {
    const int j = (i + 1) % RIM;
    const Vec3 wn = vnorm(Vec3{ -(rim[j].y - rim[i].y), rim[j].x - rim[i].x, 0.0f });
    tris[nt++] = { vb[i], vb[j], vt[j], wn, base };
    tris[nt++] = { vb[i], vt[j], vt[i], wn, base };
  }
  renderMesh(fb, invQ, tris, nt);
}

// -- markers at their true world bearings --
void drawMarkers(LGFX_Sprite& fb, const Quaternion& invQ, float readability,
                 const Marker* markers, size_t count) {
  const size_t n = count < size_t(MAX_MARKERS) ? count : size_t(MAX_MARKERS);
  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
  for (size_t i = 0; i < n; ++i) {
    float mx, my, md;
    projLevel(invQ, levelPoint(markers[i].angleRad, MARKER_RHO), readability, mx, my, md);
    if (readability < 0.7f && md < -0.25f) continue;
    fb.fillCircle(int(mx), int(my), 3, color::scale(markers[i].color, depthDim(md)));
    if (readability > 0.2f && markers[i].label) {
      float lx, ly, ld;
      projLevel(invQ, levelPoint(markers[i].angleRad, LABEL_RHO), readability, lx, ly, ld);
      fb.setTextColor(color::scale(markers[i].color, depthDim(ld)));
      fb.drawString(markers[i].label, int(lx), int(ly));
    }
  }
}

// -- 2-D HUD (flat, composited on top of the finished 3-D frame) --

// Apple-Watch-style progress ring at the very edge: a dim full track plus an
// accent fill that sweeps clockwise from 12 o'clock, with rounded end caps.
// LovyanGFX arc angles: 0 = 3 o'clock, growing clockwise; 12 o'clock = 270.
// push: 0 = settled at normal radius, 1 = slid off the glass edge.
// The glass clips at radius 120; BEZEL_R=118 sits 2 px inside, so 8 px of
// outward offset carries the entire ring (r0 at 121) past the glass boundary.
void drawBatteryRing(LGFX_Sprite& fb, float pct, u16 accent, float push) {
  if (pct < 0.0f || push >= 1.0f) return;
  pct = clampf(pct, 0.0f, 100.0f);
  const float off = push * 8.0f;
  const float r0 = BEZEL_R - RING_W + off, r1 = BEZEL_R + off;
  const u16 track = color::scale(accent, 0.16f);
  fb.fillArc(CX, CY, r0, r1, 0, 360, track);

  const u16 fill = (pct < 15.0f) ? COL_WARN : accent;
  const float start = 270.0f, sweep = pct * 3.6f, end = start + sweep;
  if (end <= 360.0f) {
    fb.fillArc(CX, CY, r0, r1, start, end, fill);
  } else {
    fb.fillArc(CX, CY, r0, r1, start, 360.0f,     fill);
    fb.fillArc(CX, CY, r0, r1, 0.0f,  end - 360.f, fill);
  }
  const float rm = (r0 + r1) * 0.5f;
  const int   cap = int((r1 - r0) * 0.5f + 0.5f);
  auto capDot = [&](float deg) {
    const float t = deg * float(M_PI) / 180.0f;
    fb.fillCircle(int(CX + rm * cosf(t)), int(CY + rm * sinf(t)), cap, fill);
  };
  capDot(start); capDot(end);
}

// Device-fixed lubber wedge at top center; you read heading against it.
void drawLubber(LGFX_Sprite& fb, u16 accent) {
  fb.fillTriangle(int(CX) - 4, 3, int(CX) + 4, 3, int(CX), 12, accent);
}

// Center hub: a small accent ring around a dark core.
void drawHub(LGFX_Sprite& fb, u16 accent) {
  fb.fillCircle(int(CX), int(CY), 3, accent);
  fb.fillCircle(int(CX), int(CY), 1, COL_BG);
}

// Status: "X SATS" curved at the 7:30 position on the bezel (225 deg).
// Hidden entirely when no GPS data is being received.
// Fades in as the battery ring fades out (statusAlpha = battRingPush).
void drawStatus(Canvas& ui, const HudState& h) {
  if (h.statusAlpha < 0.01f || !h.gpsTalking) return;

  char sb[12];
  if (h.sats == 0) snprintf(sb, sizeof(sb), "SEARCHING");
  else             snprintf(sb, sizeof(sb), "%u SATS", (unsigned)h.sats);
  const u16 satCol = color::scale(
    h.fixValid ? color::pack(150, 160, 170) : COL_WARN,
    h.statusAlpha);
  ui.arcText(sb, int(CX), int(CY), 105.0f, 225.0f, satCol, /*up=*/false, &fonts::Font2);
}

// Battery voltage + SOC, curved at the 4:00 position on the bezel (125 deg) - symmetric to SATS.
// Fades behind the battery ring like the SATS label (statusAlpha = battRingPush).
void drawBattInfo(Canvas& ui, const HudState& h) {
  if (h.battPct < 0.0f || h.battVoltage < 0.0f || h.statusAlpha < 0.01f) return;

  char buf[16];
  snprintf(buf, sizeof(buf), "%d%% %.2fV", int(h.battPct + 0.5f), h.battVoltage);

  const u16 col = color::scale(
    (h.battPct < 15.0f) ? COL_WARN : color::pack(150, 160, 170),
    h.statusAlpha);
  ui.arcText(buf, int(CX), int(CY), 105.0f, 125.0f, col, /*up=*/false, &fonts::Font2);
}

// Center hint: context-sensitive "hold to ..." label over a dark puck. Appears
// and fades with the same timing envelope as the bottom mode label. The puck
// erases the compass elements in the center so the text is always legible.
// The full-screen puck is drawn earlier in render() so the battery ring,
// lubber, and hub composite on top; only the text lives here.
void drawCenterHint(LGFX_Sprite& fb, const HudState& h) {
  if (h.centerHintAlpha < 0.02f || !h.centerHint) return;
  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
  if (h.battPct >= 0.0f) {
    char buf[24];
    if (h.charging)
      snprintf(buf, sizeof(buf), "%d%% - CHARGING", int(h.battPct + 0.5f));
    else
      snprintf(buf, sizeof(buf), "%d%%", int(h.battPct + 0.5f));
    const u16 battCol = (h.battPct < 15.0f)
                      ? color::scale(COL_WARN, h.centerHintAlpha)
                      : color::scale(h.accent, h.centerHintAlpha);
    fb.setTextColor(battCol);
    fb.drawString(buf, int(CX), int(CY) - 18);
  }
  fb.setTextColor(color::scale(h.accent, h.centerHintAlpha));
  fb.drawString(h.centerHint, int(CX), int(CY));
}

// The waypoint-set gesture (B mode): "SET TARGET" with 1/3..3/3 pips, shown
// front-and-center while a capture is in progress.
void drawSetGesture(Canvas& ui, const HudState& h) {
  if (!h.setArmed) return;
  LGFX_Sprite& fb = ui.raw();
  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
  fb.setTextColor(h.accent);
  fb.drawString("SET TARGET", int(CX), int(CY) + 30);
  const int n = 3, gap = 14, x0 = int(CX) - (n - 1) * gap / 2, y = int(CY) + 48;
  for (int i = 0; i < n; ++i) {
    const bool lit = h.setCount > i;
    if (lit) fb.fillCircle(x0 + i * gap, y, 4, h.accent);
    else     fb.drawCircle(x0 + i * gap, y, 4, color::scale(h.accent, 0.4f));
  }
}

// Calibration pips: 3 small circles at 6 o'clock, between the mode-label arc
// (r=96) and the battery ring (r=118). Each filled circle represents one step
// of system calibration (0..3).
void drawCalibPips(LGFX_Sprite& fb, const HudState& h) {
  if (!h.haveImu || h.statusAlpha < 0.01f) return;
  constexpr int N   = 3;
  constexpr int GAP = 10;
  constexpr int PIP = 3;
  constexpr int Y   = int(CY) + 107; // sits between label arc (96) and bezel ring (118)
  const int x0 = int(CX) - (N - 1) * GAP / 2;
  for (int i = 0; i < N; ++i) {
    const int x = x0 + i * GAP;
    if (h.calibSys > i) fb.fillCircle(x, Y, PIP, color::scale(h.accent, 0.6f * h.statusAlpha));
    else                 fb.drawCircle(x, Y, PIP, color::scale(h.accent, 0.35f * h.statusAlpha));
  }
}

// Bottom bezel text. The mode label rises from below the rim and fades on a
// switch; once it clears, the distance settles into the same arc, quieter. Both
// use a 1-bit GFX font on purpose - arcText chroma-keys pure green, and an
// anti-aliased face (Orbitron/Roboto) would fringe the blend pixels.
void drawBottomArc(Canvas& ui, const HudState& h) {
  constexpr float CENTER_DEG = 180.0f; // 6 o'clock, smile-side up
  const auto* font = &fonts::FreeSansBold9pt7b;

  if (h.labelAlpha > 0.02f && h.label) {
    const float radius = lerpf(ARC_TXT_R, ARC_TXT_LO, h.labelRise);
    ui.arcText(h.label, int(CX), int(CY), radius, CENTER_DEG,
               color::scale(h.accent, h.labelAlpha), /*up=*/false, font);
    return;
  }

  // label gone -> distance (or a no-fix nudge) takes the spot, dimmed
  if (h.noFix) {
    ui.arcText("NO GPS", int(CX), int(CY), ARC_TXT_R, CENTER_DEG,
               COL_WARN, false, font);
  } else if (h.showDistance && h.distance && h.distance[0]) {
    ui.arcText(h.distance, int(CX), int(CY), ARC_TXT_R, CENTER_DEG,
               color::scale(h.accent, 0.6f), false, font);
  }
}

void drawHud(LGFX_Sprite& fb, const HudState& h) {
  Canvas ui(fb);
  drawBatteryRing(fb, h.battPct, h.accent, h.battRingPush);
  drawStatus(ui, h);
  drawBattInfo(ui, h);
  drawBottomArc(ui, h);
  drawCalibPips(fb, h);
  drawSetGesture(ui, h);
  drawCenterHint(fb, h);
}

} // namespace

// Image-header dimension readers - each opens the file once, reads only the
// necessary header bytes, and returns {w, h} in pixels ({0,0} on failure).
// Avoids a full decode just to learn the image dimensions.

struct Dims { int w, h; };

static Dims readJpegDims(fs::FS& vfs, const char* path) {
    File f = vfs.open(path, "r");
    if (!f) return {0, 0};
    Dims d = {0, 0};
    uint8_t b[2];
    // Scan forward for any SOF marker (baseline C0, progressive C2, etc.)
    while (f.available() > 8 && d.w == 0) {
        if (f.read(b, 1) != 1 || b[0] != 0xFF) continue;
        if (f.read(b, 1) != 1) break;
        const uint8_t marker = b[1];
        if (marker == 0xFF) { f.seek(f.position() - 1); continue; }
        if ((marker >= 0xC0 && marker <= 0xC3) ||
            (marker >= 0xC5 && marker <= 0xC7) ||
            (marker >= 0xC9 && marker <= 0xCB) ||
            (marker >= 0xCD && marker <= 0xCF)) {
            // SOF: 2-byte length, 1-byte precision, 2-byte height, 2-byte width
            uint8_t hdr[7];
            if (f.read(hdr, 7) == 7) {
                d.h = (hdr[3] << 8) | hdr[4];
                d.w = (hdr[5] << 8) | hdr[6];
            }
        } else {
            // Skip segment: next 2 bytes are segment length (includes those 2 bytes)
            if (f.read(b, 2) != 2) break;
            const int len = (b[0] << 8) | b[1];
            if (len > 2) f.seek(f.position() + len - 2);
        }
    }
    f.close();
    return d;
}

static Dims readPngDims(fs::FS& vfs, const char* path) {
    File f = vfs.open(path, "r");
    if (!f) return {0, 0};
    uint8_t buf[24];
    const int n = f.read(buf, 24);
    f.close();
    if (n < 24) return {0, 0};
    // PNG IHDR: 8-byte sig + 4-byte chunk len + 4-byte "IHDR" + 4-byte W + 4-byte H
    return {
        (buf[16] << 24) | (buf[17] << 16) | (buf[18] << 8) | buf[19],
        (buf[20] << 24) | (buf[21] << 16) | (buf[22] << 8) | buf[23]
    };
}

static Dims readBmpDims(fs::FS& vfs, const char* path) {
    File f = vfs.open(path, "r");
    if (!f) return {0, 0};
    uint8_t buf[26];
    const int n = f.read(buf, 26);
    f.close();
    if (n < 26) return {0, 0};
    // BMP DIB header: width at 18-21, height at 22-25 (little-endian i32; height
    // may be negative for top-down bitmaps - take abs)
    int w = buf[18] | (buf[19] << 8) | (buf[20] << 16) | (buf[21] << 24);
    int h = buf[22] | (buf[23] << 8) | (buf[24] << 16) | (buf[25] << 24);
    if (h < 0) h = -h;
    return {w, h};
}

void Display::begin() {
  // CS is hard-wired to GND (always asserted) and RST is on a 10k pullup with
  // no GPIO control. During firmware flashing or brown-out, ESP32 GPIO pins
  // float while the bootloader runs, and the GC9A01 latches those SPI glitches
  // as commands - leaving the panel in an unknown state before we get here.
  //
  // Fix: init twice with a gap. The first call configures the SPI peripheral
  // and sends SWRESET, but the panel may be too confused to act on it. After
  // 150 ms the panel's own state machine has settled; the second init sends
  // SWRESET again over a working bus and runs the full command sequence on a
  // clean panel. Without a GPIO-controlled RST pin this is the most reliable
  // way to guarantee a good start after flash or power loss.
  //
  // Long-term fix: wire a free GPIO to the GC9A01 RST pin and set
  // cfg.pin_rst in LGFX_Config.hpp - LovyanGFX will then strobe RST low
  // before every init, making boot-glitch corruption impossible.
  delay(150);
  lcd.init();   // sets up SPI bus; SWRESET may land on a dirty panel
  delay(150);   // let SWRESET expire / panel POR settle
  lcd.init();   // second pass: panel is clean, init sequence lands correctly
  lcd.setRotation(0);
  lcd.setColorDepth(16);

  fb.setColorDepth(16);
  fb.setSwapBytes(true); // match panel byte order (else R/B swap)

  // createSprite needs about 115 KB of contiguous heap. It tries internal SRAM first
  // (DMA-capable), then PSRAM if the build has it enabled. If it returns null
  // every subsequent draw call is a no-op and pushSprite sends nothing - the
  // display stays black while the rest of the app runs normally.
  void* buf = fb.createSprite(SCREEN, SCREEN);
  Serial.printf("[display] fb sprite %s  free=%lu  largest=%lu\n",
                buf ? "OK" : "FAILED (OOM - display will be black)",
                (unsigned long)ESP.getFreeHeap(),
                (unsigned long)ESP.getMaxAllocHeap());

  // SCREEN*SCREEN decode buffer for slideshow images. Images are scaled to
  // zoom-to-fill at decode time, so no oversized buffer is needed.
  _imgBuf.setColorDepth(16);
  _imgBuf.setSwapBytes(true);
  void* ibuf = _imgBuf.createSprite(SCREEN, SCREEN);
  Serial.printf("[display] imgBuf %s  %dx%d  free=%lu\n",
                ibuf ? "OK" : "FAILED (OOM - slideshow will be black)",
                _imgBuf.width(), _imgBuf.height(),
                (unsigned long)ESP.getFreeHeap());
  _imgPath[0] = '\0'; // sprite buffer was reallocated; force re-decode on next showSlide
  _imgW = 0;
  _imgH = 0;

  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
}

void Display::sleep() {
    fb.fillSprite(0);
    fb.pushSprite(0, 0);
    lcd.sleep();       // send SLPIN to the GC9A01
    lcd.waitDisplay(); // flush DMA before we power down
}

void Display::showSlide(const char* path, float fadeAlpha) {
  // Bind to the fs::FS base so LGFX picks the (fs::FS&) file-decode overloads.
  fs::FS& vfs = LittleFS;

  // -- decode once per image change --
  // Decode into the PSRAM buffer; subsequent frames for the same image just
  // blit a centered window - no re-decode.
  if (path && strcmp(path, _imgPath) != 0) {
    bool ok = false;
    Dims dims = {0, 0};
    const char* dot = strrchr(path, '.');
    if (dot && (!strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg")))
      dims = readJpegDims(vfs, path);
    else if (dot && !strcasecmp(dot, ".png"))
      dims = readPngDims(vfs, path);
    else if (dot && !strcasecmp(dot, ".bmp"))
      dims = readBmpDims(vfs, path);

    // Zoom-to-fill: scale so the shorter dimension fills SCREEN, then center-crop.
    // The negative x/y positions the scaled image so the center lands at (0,0)
    // of the sprite; LGFX clips the overdraw naturally.
    float scale = 1.0f;
    int   dx = 0, dy = 0;
    if (dims.w > 0 && dims.h > 0) {
      scale = fmaxf((float)SCREEN / dims.w, (float)SCREEN / dims.h);
      dx = (int)(-(dims.w * scale - SCREEN) / 2.0f);
      dy = (int)(-(dims.h * scale - SCREEN) / 2.0f);
    }

    _imgBuf.fillSprite(0);
    if (dot && (!strcasecmp(dot, ".jpg") || !strcasecmp(dot, ".jpeg")))
      ok = _imgBuf.drawJpgFile(vfs, path, dx, dy, 0, 0, 0, 0, scale, scale);
    else if (dot && !strcasecmp(dot, ".png"))
      ok = _imgBuf.drawPngFile(vfs, path, dx, dy, 0, 0, 0, 0, scale, scale);
    else if (dot && !strcasecmp(dot, ".bmp"))
      ok = _imgBuf.drawBmpFile(vfs, path, dx, dy, 0, 0, 0, 0, scale, scale);

    _imgW = ok ? dims.w : -1;
    _imgH = ok ? dims.h : -1;
    strncpy(_imgPath, path, sizeof(_imgPath) - 1);
    _imgPath[sizeof(_imgPath) - 1] = '\0';
    Serial.printf("[display] slide: %s  %dx%d  ok=%d  buf=%dx%d\n",
                  path, dims.w, dims.h, (int)ok, _imgBuf.width(), _imgBuf.height());
  }

  // -- build frame: blit centered window into fb --
  fb.fillSprite(0);

  if (path && _imgW > 0 && _imgH > 0) {
    _imgBuf.pushSprite(&fb, 0, 0);
  } else {
    fb.fillSprite(COL_BG);
    fb.setFont(&fonts::Font2);
    fb.setTextDatum(textdatum_t::middle_center);
    fb.setTextColor(COL_RING);
    fb.drawString(path ? "IMAGE ERROR" : "NO IMAGES", int(CX), int(CY));
  }

  // -- fade to/from black via integer brightness scaling --
  // fb has setSwapBytes(true) so each uint16_t in the buffer is big-endian
  // RGB565. We swap to host byte order, scale R/G/B, then swap back.
  // About 57k iterations with 3 integer multiplies each is 2-3 ms on the S3.
  // Skip fade for error states (no images / decode failure) so the message
  // is always visible - Slideshow::fadeAlpha() returns 0 when count==0 which
  // would otherwise silently overwrite the "NO IMAGES" text with black.
  fadeAlpha = clampf(fadeAlpha, 0.0f, 1.0f);
  if (path && _imgW > 0 && fadeAlpha < 0.999f) {
    if (fadeAlpha <= 0.0f) {
      fb.fillSprite(0);
    } else {
      const int ia = (int)(fadeAlpha * 256.0f); // 1..255
      auto* buf = static_cast<uint16_t*>(fb.getBuffer());
      if (buf) {
        const int n = SCREEN * SCREEN;
        for (int i = 0; i < n; ++i) {
          // bswap16: convert big-endian panel pixel -> host-order RGB565
          const uint16_t c  = (uint16_t)((buf[i] >> 8) | (buf[i] << 8));
          const int r = (c >> 11) & 0x1F;
          const int g = (c >>  5) & 0x3F;
          const int b =  c        & 0x1F;
          const uint16_t out = (uint16_t)(((r * ia >> 8) << 11) |
                                          ((g * ia >> 8) <<  5) |
                                           (b * ia >> 8));
          buf[i] = (uint16_t)((out >> 8) | (out << 8)); // back to big-endian
        }
      }
    }
  }

  fb.pushSprite(0, 0);
}

void Display::render(const Quaternion& qRose, const Quaternion& qArrow,
                     const Marker* markers, size_t count, const HudState& hud) {
  fb.fillSprite(COL_BG);

  const Quaternion invRose  = conjugate(qRose);  // true-north frame for ring/cardinals
  const Quaternion invArrow = conjugate(qArrow); // bearing frame for the pointer

  const Vec3  vUp = rotate(invRose, Vec3{ 0.0f, 0.0f, 1.0f });
  const float readability = smoothstepf(FLAT_LO, FLAT_HI, vUp.z);

  drawRing(fb, invRose, readability);
  drawCardinals(fb, invRose, readability);

  if (_indicator == Indicator::Arrow) {
    drawArrow(fb, invArrow, _arrowColor);
  } else {
    drawNeedle(fb, invArrow);
    drawMarkers(fb, invRose, readability, markers, count);
  }

  // Lubber and hub sit behind the dark overlay so they disappear when the hint
  // is active, reducing clutter while the center text is on screen.
  drawLubber(fb, hud.accent);
  drawHub(fb, hud.accent);

  // Full-screen dark overlay: in front of the compass, behind all HUD chrome.
  // Radius 120 fills the entire round display aperture. Only active during the
  // center-hint animation so the compass dims when the hint/battery text appear.
  if (hud.centerHintAlpha > 0.02f)
    fb.fillCircle(int(CX), int(CY), 120, COL_BG);

  drawHud(fb, hud);

  fb.pushSprite(0, 0);
}
