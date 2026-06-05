#pragma once

#include <math.h>

struct Vec3 { float x, y, z; };
struct Quaternion { float w, x, y, z; };

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y*b.y + a.z*b.z;
}

inline Quaternion conjugate(const Quaternion& q) {
    return { q.w, -q.x, -q.y, -q.z };
}

inline Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
             a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
             a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
             a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}

// fast q*v form
inline Vec3 rotate(const Quaternion& q, const Vec3& v) {
    Vec3 u{ q.x, q.y, q.z };
    Vec3 t = {  2*(u.y*v.z - u.z*v.y), 2*(u.z*v.x - u.x*v.z), 2*(u.x*v.y - u.y*v.x) };
    return { v.x + q.w*t.x + (u.y*t.z - u.z*t.y),
           v.y + q.w*t.y + (u.z*t.x - u.x*t.z),
           v.z + q.w*t.z + (u.x*t.y - u.y*t.x) };
}

inline Quaternion normalize(const Quaternion& q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    float s = (n > 0) ? 1.0f / n : 0.0f;
    return { q.w*s, q.x*s, q.y*s, q.z*s };
}

// Rotation about world up (+Z) by `rad`, right-handed (CCW looking down +Z).
// The one place an angle becomes a quaternion: declination fold and bearing
// fold both build their world-up yaw through here.
inline Quaternion yawQuat(float rad) {
    const float h = rad * 0.5f;
    return { cosf(h), 0.0f, 0.0f, sinf(h) };
}