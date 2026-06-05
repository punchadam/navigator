// Display.cpp — 3D level-compass renderer for a round TFT on ESP32-S3.
//
// Pure renderer: a function of (qShow, markers, HudState) -> pixels. All
// smoothing, declination folding, unit choice, animation clocks and string
// formatting happen upstream (main + AppState); the only state this class owns
// is the off-screen framebuffer plus which pointer to draw and the arrow color.
//
// Model: a gimballed compass. The rose (degree ring + cardinals + markers)
// floats on the world-horizontal plane, so it foreshortens in 3D as the device
// tilts and its North tick always points at true North. A raised, shaded
// pointer — the two-tone needle (compass) or a single-color arrow (waypoint) —
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

using color::Rgb;

namespace {

// ---- geometry constants (pixels, and unit-ring units where noted) ----------
constexpr int   SCREEN = 240;
constexpr float CX = SCREEN * 0.5f;      // ring center, pixels
constexpr float CY = SCREEN * 0.5f;
constexpr float R  = 104.0f;             // ring radius, pixels

constexpr float TICK_MINOR_IN = 0.92f;   // unit-ring radii for tick endpoints
constexpr float MARK_TICK_IN  = 0.84f;
constexpr float RING_RHO      = 1.00f;
constexpr float CARDINAL_RHO  = 0.78f;   // N/E/S/W letters, inside the ring
constexpr float LABEL_RHO     = 0.72f;
constexpr float MARKER_RHO    = 1.05f;   // markers sit just outside the ring

// Needle (unit-ring units; raw-projected so it keeps true 3-D foreshortening).
constexpr float N_LEN = 0.66f;           // tip distance from center
constexpr float N_WID = 0.12f;           // half-width at the shoulders
constexpr float N_HGT = 0.16f;           // crest height above the level plane

// Arrow (waypoint pointer): a single raised, faceted blade pointing at +Y.
constexpr float A_LEN    = 0.72f;        // tip distance from center
constexpr float A_HEADL  = 0.34f;        // arrowhead length (tip back to barbs)
constexpr float A_HEADW  = 0.27f;        // barb half-width
constexpr float A_SHAFTW = 0.085f;       // shaft half-width
constexpr float A_TAIL   = 0.34f;        // shaft length behind center
constexpr float A_HGT    = 0.16f;        // crest height above the level plane

constexpr int   MAX_MARKERS = 24;

// Legibility band: full 3-D foreshortening square-on, re-inflated circle edge-on.
constexpr float FLAT_LO = 0.15f;
constexpr float FLAT_HI = 0.45f;

// ---- HUD geometry (pixels) --------------------------------------------------
constexpr float BEZEL_R   = 118.0f;      // battery ring outer edge
constexpr float RING_W    = 5.0f;        // battery ring thickness
constexpr float ARC_TXT_R = 96.0f;       // curved label/distance settled radius
constexpr float ARC_TXT_LO= 132.0f;      // ...where it starts, below the bezel

// ---- colors (RGB565) --------------------------------------------------------
const u16 COL_BG     = color::pack(  8, 10, 14);
const u16 COL_RING   = color::pack(120,130,140);
const u16 COL_MINOR  = color::pack( 70, 78, 88);
const u16 COL_HUB    = color::pack(225,225,225);
const u16 COL_WARN   = color::pack(255, 80, 70);   // low battery / no fix
const u16 COL_GOOD   = color::pack( 80,230,130);   // full charge

// Needle facet base colors (RGB888, shaded then converted at draw time).
const Rgb NEEDLE_NORTH{ 226, 75, 74 };   // red north half
const Rgb NEEDLE_SOUTH{ 205,206,196 };   // pale south half

// Light direction in the rose's screen frame: upper-left, toward the viewer.
const Vec3 LIGHT = { -0.35f, 0.45f, 0.82f };

// ---- small float / vector helpers -------------------------------------------
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
  float t = clampf(depth * 0.5f + 0.5f, 0.0f, 1.0f);   // depth ~[-1,1] -> [0,1]
  return lerpf(0.45f, 1.0f, t);
}
inline float lambert(const Vec3& nScreen) {
  float diff = nScreen.x*LIGHT.x + nScreen.y*LIGHT.y + nScreen.z*LIGHT.z;
  return clampf(diff, 0.0f, 1.0f) * 0.6f + 0.4f;
}
inline Vec3 levelPoint(float angle, float rho) {  // compass angle, in-plane radius
  return { rho * sinf(angle), rho * cosf(angle), 0.0f };
}

// ---- projection -------------------------------------------------------------
void projLevel(const Quaternion& invQ, const Vec3& world, float readability,
               float& sx, float& sy, float& depth) {
  Vec3 b = rotate(invQ, world);          // world -> screen frame
  depth = b.z;
  float rawLen = sqrtf(b.x*b.x + b.y*b.y);
  float u = b.x, v = b.y;
  if (rawLen > 1e-4f) {
    float rho = sqrtf(world.x*world.x + world.y*world.y);  // full-circle radius
    float len = lerpf(rho, rawLen, readability);            // toward circle as r->0
    float k = len / rawLen;
    u = b.x * k; v = b.y * k;
  }
  sx = CX + R * u;
  sy = CY - R * v;                       // flip: screen y grows downward
}

void projRaw(const Quaternion& invQ, const Vec3& world, float& sx, float& sy, float& depth) {
  Vec3 b = rotate(invQ, world);
  sx = CX + R * b.x;
  sy = CY - R * b.y;
  depth = b.z;
}

// ---- shared 3-D mesh renderer -----------------------------------------------
struct Tri { Vec3 a, b, c, n; Rgb color; };   // n = outward normal in body space

void renderMesh(LGFX_Sprite& fb, const Quaternion& invQ, const Tri* tris, int n) {
  constexpr int MAX = 32;                      // needle = 4, arrow = 20
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
    const Vec3 ns = vnorm(rotate(invQ, tris[i].n));   // body -> screen frame
    const u16 c = color::shade(tris[i].color, lambert(ns));
    fb.fillTriangle(int(p[i].ax), int(p[i].ay),
                    int(p[i].bx), int(p[i].by),
                    int(p[i].cx), int(p[i].cy), c);
  }
}

// ---- the rose (degree ring) -------------------------------------------------
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

// ---- cardinal letters, riding the rose --------------------------------------
// N/E/S/W sit on the level plane, so they foreshorten and rotate with the rose.
// In compass mode N reads at the top; in waypoint mode the frame is yawed to the
// bearing, so the letters quietly show where the real directions have gone.
// North is tinted to match the needle's red so it always reads as "north".
void drawCardinals(LGFX_Sprite& fb, const Quaternion& invQ, float readability) {
  static const char* L[4] = { "N", "E", "S", "W" };
  fb.setFont(&fonts::Font2);
  fb.setTextDatum(textdatum_t::middle_center);
  for (int i = 0; i < 4; ++i) {
    const float ang = i * float(M_PI) / 2.0f;     // 0=N, 90=E, 180=S, 270=W
    float x, y, d;
    projLevel(invQ, levelPoint(ang, CARDINAL_RHO), readability, x, y, d);
    if (readability < 0.7f && d < -0.25f) continue; // hide the far side in 3-D
    const u16 base = (i == 0) ? color::pack(NEEDLE_NORTH.r, NEEDLE_NORTH.g, NEEDLE_NORTH.b)
                                   : COL_RING;
    fb.setTextColor(color::scale(base, depthDim(d)));
    fb.drawString(L[i], int(x), int(y));
  }
}

// ---- the 3-D needle ---------------------------------------------------------
void drawNeedle(LGFX_Sprite& fb, const Quaternion& invQ) {
  const Vec3 V[5] = {
    levelPoint(0.0f,            N_LEN),   // 0: North tip      (+Y)
    levelPoint(float(M_PI),     N_LEN),   // 1: South tip      (-Y)
    levelPoint(-float(M_PI)/2,  N_WID),   // 2: West shoulder  (-X)
    levelPoint( float(M_PI)/2,  N_WID),   // 3: East shoulder  (+X)
    Vec3{ 0.0f, 0.0f, N_HGT }             // 4: crest apex     (+Z)
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

// ---- the 3-D arrow ----------------------------------------------------------
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

// ---- markers at their true world bearings -----------------------------------
void drawMarkers(LGFX_Sprite& fb, const Quaternion& invQ, float readability,
                 const Marker* markers, size_t count) {
  const size_t n = count < size_t(MAX_MARKERS) ? count : size_t(MAX_MARKERS);
  fb.setFont(&fonts::Font2);               // pin the marker-label font (was implicit)
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

// ============================================================================
//  2-D HUD  (flat, composited on top of the finished 3-D frame)
// ============================================================================

// Apple-Watch-style progress ring at the very edge: a dim full track plus an
// accent fill that sweeps clockwise from 12 o'clock, with rounded end caps.
// LovyanGFX arc angles: 0 = 3 o'clock, growing clockwise; 12 o'clock = 270.
void drawBatteryRing(LGFX_Sprite& fb, float pct, u16 accent) {
  if (pct < 0.0f) return;
  pct = clampf(pct, 0.0f, 100.0f);
  const float r0 = BEZEL_R - RING_W, r1 = BEZEL_R;
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
  auto dot = [&](float deg) {
    const float t = deg * float(M_PI) / 180.0f;
    fb.fillCircle(int(CX + rm * cosf(t)), int(CY + rm * sinf(t)), cap, fill);
  };
  dot(start); dot(end);
}

// Device-fixed lubber wedge at top center; you read heading against it.
void drawLubber(LGFX_Sprite& fb, u16 accent) {
  fb.fillTriangle(int(CX) - 8, 3, int(CX) + 8, 3, int(CX), 22, accent);
}

// Center hub: a small accent ring around a dark core.
void drawHub(LGFX_Sprite& fb, u16 accent) {
  fb.fillCircle(int(CX), int(CY), 5, accent);
  fb.fillCircle(int(CX), int(CY), 2, COL_BG);
}

// Bottom-left: GPS sats, plus orientation calibration as three pips (0..3).
void drawStatus(LGFX_Sprite& fb, const HudState& h) {
  fb.setFont(&fonts::Font2);
  char sb[12];
  snprintf(sb, sizeof(sb), "SAT %u", (unsigned)h.sats);
  fb.setTextDatum(textdatum_t::bottom_left);
  fb.setTextColor(h.fixValid ? color::pack(150,160,170) : COL_WARN);
  fb.drawString(sb, 10, SCREEN - 6);

  // calib pips, climbing the left edge; lit ones in accent, rest dim.
  const int x = 12, y0 = SCREEN - 34;
  for (int i = 0; i < 3; ++i) {
    const bool lit = h.calib > i;            // calib 0..3
    const u16 c = lit ? h.accent : color::scale(h.accent, 0.18f);
    fb.fillCircle(x, y0 - i * 7, 2, c);
  }
}

// Left-edge splash answering the physical button (a quick inward bloom).
void drawPressSplash(LGFX_Sprite& fb, float s, u16 accent) {
  if (s <= 0.01f) return;
  const float r1 = BEZEL_R, r0 = lerpf(BEZEL_R, 84.0f, s);   // 180 = 9 o'clock
  fb.fillArc(CX, CY, r0, r1, 152, 208, color::scale(accent, s));
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

// Bottom bezel text. The mode label rises from below the rim and fades on a
// switch; once it clears, the distance settles into the same arc, quieter. Both
// use a 1-bit GFX font on purpose — arcText chroma-keys pure green, and an
// anti-aliased face (Orbitron/Roboto) would fringe the blend pixels.
void drawBottomArc(Canvas& ui, const HudState& h) {
  constexpr float CENTER_DEG = 180.0f;          // 6 o'clock, smile-side up
  const auto* font = &fonts::FreeSansBold9pt7b;

  if (h.labelAlpha > 0.02f && h.label) {
    const float radius = lerpf(ARC_TXT_R, ARC_TXT_LO, h.labelRise);
    ui.arcText(h.label, int(CX), int(CY), radius, CENTER_DEG,
               color::scale(h.accent, h.labelAlpha), /*up=*/false, font);
    return;
  }

  // label gone -> distance (or a no-fix nudge) takes the spot, dimmed
  if (h.noFix) {
    ui.arcText("NO FIX", int(CX), int(CY), ARC_TXT_R, CENTER_DEG,
               COL_WARN, false, font);
  } else if (h.showDistance && h.distance && h.distance[0]) {
    ui.arcText(h.distance, int(CX), int(CY), ARC_TXT_R, CENTER_DEG,
               color::scale(h.accent, 0.6f), false, font);
  }
}

void drawHud(LGFX_Sprite& fb, const HudState& h) {
  Canvas ui(fb);
  drawBatteryRing(fb, h.battPct, h.accent);
  drawLubber(fb, h.accent);
  drawHub(fb, h.accent);
  drawStatus(fb, h);
  drawBottomArc(ui, h);
  drawSetGesture(ui, h);
  drawPressSplash(fb, h.pressSplash, h.accent);
}

// ---- the charging bolt ------------------------------------------------------
// A six-point bolt silhouette (T, right shoulder, right inner, bottom, left
// inner, left shoulder) tiled by four triangles: two flags + a two-tri spine.
// Normalized to roughly [-0.5,0.5] x [-1,1], then scaled and centered.
void drawBolt(LGFX_Sprite& fb, float cx, float cy, float sx, float sy, u16 c) {
  const int Tx  = int(cx + 0.20f * sx), Ty  = int(cy - 1.00f * sy);  // top
  const int RSx = int(cx + 0.50f * sx), RSy = int(cy - 0.15f * sy);  // right shoulder
  const int RIx = int(cx + 0.08f * sx), RIy = int(cy - 0.10f * sy);  // right inner
  const int Bx  = int(cx - 0.20f * sx), By  = int(cy + 1.00f * sy);  // bottom
  const int LIx = int(cx - 0.08f * sx), LIy = int(cy + 0.10f * sy);  // left inner
  const int LSx = int(cx - 0.50f * sx), LSy = int(cy + 0.15f * sy);  // left shoulder

  fb.fillTriangle(Tx, Ty, RSx, RSy, RIx, RIy, c);   // right flag
  fb.fillTriangle(Tx, Ty, RIx, RIy, LIx, LIy, c);   // upper spine
  fb.fillTriangle(RIx, RIy, Bx, By, LIx, LIy, c);   // lower spine
  fb.fillTriangle(LIx, LIy, LSx, LSy, Tx, Ty, c);   // left flag
}

} // namespace

// ----------------------------------------------------------------------------
void Display::begin() {
  lcd.init();
  lcd.setRotation(0);
  lcd.setColorDepth(16);

  fb.setColorDepth(16);
  fb.setSwapBytes(true);                 // match panel byte order (else R/B swap)
  fb.createSprite(SCREEN, SCREEN);       // ~115 KB, keep in internal SRAM
  fb.setFont(&fonts::Font2);             // sane default; every drawer sets its own
  fb.setTextDatum(textdatum_t::middle_center);
}

void Display::render(const Quaternion& qShow, const Marker* markers, size_t count,
                     const HudState& hud) {
  fb.fillSprite(COL_BG);

  const Quaternion invQ = conjugate(qShow);           // world -> screen frame

  const Vec3  vUp = rotate(invQ, Vec3{ 0.0f, 0.0f, 1.0f });
  const float readability = smoothstepf(FLAT_LO, FLAT_HI, vUp.z);

  drawRing(fb, invQ, readability);
  drawCardinals(fb, invQ, readability);

  if (_indicator == Indicator::Arrow) {
    drawArrow(fb, invQ, _arrowColor);
  } else {
    drawNeedle(fb, invQ);
    drawMarkers(fb, invQ, readability, markers, count);
  }

  drawHud(fb, hud);

  fb.pushSprite(0, 0);
}

void Display::renderCharging(const ChargeState& cs) {
  fb.fillSprite(COL_BG);

  const u16 bolt = cs.full ? COL_GOOD : cs.accent;

  // progress ring (reuses the HUD's battery ring) + a soft bolt glow behind.
  drawBatteryRing(fb, cs.pct, bolt);
  drawBolt(fb, CX, CY - 8, 66.0f, 54.0f, color::scale(bolt, 0.22f));  // halo
  drawBolt(fb, CX, CY - 8, 60.0f, 48.0f, bolt);                       // bolt

  fb.setTextDatum(textdatum_t::middle_center);

  fb.setFont(&fonts::Font2);
  fb.setTextColor(color::scale(bolt, 0.7f));
  fb.drawString(cs.line1, int(CX), int(CY) - 78);

  fb.setFont(&fonts::Orbitron_Light_24);
  fb.setTextColor(bolt);
  fb.drawString(cs.line2, int(CX), int(CY) + 62);

  fb.setFont(&fonts::Font2);
  fb.setTextColor(color::pack(150, 160, 170));
  fb.drawString(cs.line3, int(CX), int(CY) + 90);

  fb.pushSprite(0, 0);
}