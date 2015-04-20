#pragma once

#include <cmath>

namespace isect2d {

struct Vec2 {
    float x, y;

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

inline Vec2 project(const Vec2& _p, const Vec2& _axis) {
    float l = _axis.length();
    return _axis * _p.dot(_axis) * (1 / (l * l));
}

}
