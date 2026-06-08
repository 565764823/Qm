#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <cmath>

namespace math {
inline int sign(float v) {
    return (v > 0.0f) ? 1 : ((v < 0.0f) ? -1 : 0);
}

inline float rad2deg(float rad) {
    return rad * 180.0f / static_cast<float>(M_PI);
}

inline float deg2rad(float deg) {
    return deg * static_cast<float>(M_PI) / 180.0f;
}


inline float normalizeAngle(float ang) {
    while (ang > 180.0f)  ang -= 360.0f;
    while (ang < -180.0f) ang += 360.0f;
    return ang;
}
inline float clamp(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

inline float clampSym(float val, float maxVal) {
    return clamp(val, -maxVal, maxVal);
}

inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}
}  

#endif  
