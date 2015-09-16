#pragma once

#include <cmath>
#include "vec.h"

namespace isect2d {

struct Vec2 {
    float x, y;

    Vec2(const Vec2& other) {
        x = other.x;
        y = other.y;
    }

    Vec2(float _x = 0, float _y = 0) {
        x = _x;
        y = _y;
    }

    Vec2 operator+(const Vec2& _b) const {
        return Vec2(x + _b.x, y + _b.y);
    }

    Vec2 operator-(const Vec2& _b) const {
        return Vec2(x - _b.x, y - _b.y);
    }

    Vec2 operator*(float _b) const {
        return Vec2(x * _b, y * _b);
    }

    float operator[](const int i) const {
        return i == 0 ? x : y;
    }

    float length() const {
        return sqrt(x * x + y * y);
    }

    Vec2& normalize() {
        return *this = *this * (1 / length());
    }

    float dot(const Vec2& _b) const {
        return x * _b.x + y * _b.y;
    }

    Vec2 perp() const {
        return Vec2(-y, x);
    }
};

inline bool operator==(const Vec2& lh, const Vec2& rh) {
    return lh.x == rh.x && lh.y == rh.y;
}

inline bool operator!=(const Vec2& lh, const Vec2& rh) {
    return !(lh == rh);
}

template<>
inline float dot(const Vec2& _v, const Vec2& _b) {
  return _v.x * _b.x + _v.y * _b.y;
}

template<>
inline Vec2 normalize(const Vec2& _v) {
  return _v * (1.0f / _v.length());
}

}
