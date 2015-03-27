#ifndef ISECT2D_H
#define ISECT2D_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

namespace isect2d {
struct OBB;
}

static bool intersect(const isect2d::OBB& _a, const isect2d::OBB& _b);

static std::set<std::pair<int, int>> intersect(const isect2d::OBB* _obbs1, const size_t _size1,
                                               const isect2d::OBB* _obbs2, const size_t _size2);

namespace isect2d {

struct Vec2 {
    double x, y;

    Vec2(double _x = 0, double _y = 0) {
        x = _x;
        y = _y;
    }

    Vec2 operator+(const Vec2& _b) const {
        return Vec2(x + _b.x, y + _b.y);
    }

    Vec2 operator-(const Vec2& _b) const {
        return Vec2(x - _b.x, y - _b.y);
    }

    Vec2 operator*(double _b) const {
        return Vec2(x * _b, y * _b);
    }

    double length() const {
        return sqrt(x * x + y * y);
    }

    Vec2& normalize() {
        return *this = *this * (1 / length());
    }

    double dot(const Vec2& _b) const {
        return x * _b.x + y * _b.y;
    }

    Vec2 perp() const {
        return Vec2(-y, x);
    }
};

inline Vec2 project(const Vec2& _p, const Vec2& _axis) {
    double l = _axis.length();
    return _axis * _p.dot(_axis) * (1 / (l * l));
}

struct OBB {

    OBB(double _cx, double _cy, double _a, double _w, double _h) {
        Vec2 center(_cx, _cy);
        Vec2 x( cos(_a), sin(_a));
        Vec2 y(-sin(_a), cos(_a));

        x = x * (_w / 2);
        y = y * (_h / 2);

        m_quad[0] = center - x - y; // lower-left
        m_quad[1] = center + x - y; // lower-right
        m_quad[2] = center + x + y; // uper-right
        m_quad[3] = center - x + y; // uper-left

        perpAxes();
    }

    void move(const double _px, const double _py) {
        Vec2 t(_px, _py);

        for (int i = 0; i < 4; ++i) {
            m_quad[i] = m_quad[i] + t;
        }

        perpAxes();
    }

    void perpAxes() {
        m_axes[0] = (m_quad[2] - m_quad[3]).normalize();
        m_axes[1] = (m_quad[2] - m_quad[1]).normalize();
    }

    const Vec2* getQuad() const {
        return m_quad;
    }

    const Vec2* getAxes() const{
        return m_axes;
    }

private:
    Vec2 m_quad[4];
    Vec2 m_axes[2];
};

}

static isect2d::Vec2 projectToAxis(const isect2d::OBB& _obb, const isect2d::Vec2& axis) {
    double inf = std::numeric_limits<double>::infinity();
    double min = inf;
    double max = -inf;

    const isect2d::Vec2* p = _obb.getQuad();

    for (int i = 0; i < 4; ++i) {
        isect2d::Vec2 proj = project(p[i], axis);
        double d = proj.dot(axis);

        min = std::min(min, d);
        max = std::max(max, d);
    }

    return isect2d::Vec2(min, max);
}

static bool intersect(const isect2d::OBB& _a, const isect2d::OBB& _b) {
    bool overlaps[2] = { false };
    const isect2d::Vec2* aaxes = _a.getAxes();
    const isect2d::Vec2* baxes = _b.getAxes();

    for (int i = 0; i < 2; ++i) {
        isect2d::Vec2 aproj = projectToAxis(_a, aaxes[i]);
        isect2d::Vec2 bproj = projectToAxis(_b, baxes[i]);

        if (bproj.x <= aproj.y && bproj.y >= aproj.x) {
            overlaps[i] = true;
        }
    }

    return overlaps[0] && overlaps[1];
}

static std::set<std::pair<int, int>> intersect(const isect2d::OBB* _obbs1, const size_t _size1,
                                               const isect2d::OBB* _obbs2, const size_t _size2) {

}

#endif

