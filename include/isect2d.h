#ifndef ISECT2D_H
#define ISECT2D_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <iostream>

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

bool operator==(const Vec2& lh, const Vec2& rh) {
    return lh.x == rh.x && lh.y == rh.y;
}

bool operator!=(const Vec2& lh, const Vec2& rh) {
    return !(lh == rh);
}

inline Vec2 project(const Vec2& _p, const Vec2& _axis) {
    double l = _axis.length();
    return _axis * _p.dot(_axis) * (1 / (l * l));
}

struct OBB {

    OBB(double _cx, double _cy, double _a, double _w, double _h) :
        m_center(Vec2(_cx, _cy)), m_angle(_a), m_width(_w), m_height(_h) {
        update();
    }

    void move(const double _px, const double _py) {
        m_center = Vec2(_px, _py);
        update();
    }

    void rotate(float _angle) {
        m_angle = _angle;
        update();
    }

    const Vec2* getQuad() const {
        return m_quad;
    }

    const Vec2* getAxes() const{
        return m_axes;
    }

    const Vec2& getCenter() const {
        return m_center;
    }

    double getAngle() const {
        return m_angle;
    }

    double getWidth() const {
        return m_width;
    }

    double getHeight() const {
        return m_height;
    }

    double radius() const {
        Vec2 extent(m_width, m_height);
        return extent.length();
    }

private:

    void perpAxes() {
        m_axes[0] = (m_quad[2] - m_quad[3]).normalize();
        m_axes[1] = (m_quad[2] - m_quad[1]).normalize();
    }

    void update() {
        Vec2 x( cos(m_angle), sin(m_angle));
        Vec2 y(-sin(m_angle), cos(m_angle));

        x = x * (m_width / 2);
        y = y * (m_height / 2);

        m_quad[0] = m_center - x - y; // lower-left
        m_quad[1] = m_center + x - y; // lower-right
        m_quad[2] = m_center + x + y; // uper-right
        m_quad[3] = m_center - x + y; // uper-left

        perpAxes();
    }

    Vec2 m_quad[4];
    Vec2 m_axes[2];
    Vec2 m_center;
    double m_angle;
    double m_width;
    double m_height;
};

struct AABB : OBB {
    AABB(double _cx, double _cy, double _w, double _h) : OBB(_cx, _cy, 0.0, _w, _h) {}

    void rotate(float _angle) = delete;
};

bool operator==(const OBB& lh, const OBB& rh) {
    return lh.getCenter() == rh.getCenter() && lh.getAngle() == rh.getAngle();
}

bool operator!=(const OBB& lh, const OBB& rh) {
    return !(lh == rh);
}

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

static bool axisCollide(const isect2d::OBB& _a, const isect2d::OBB& _b, const isect2d::Vec2* axes) {
    for (int i = 0; i < 2; ++i) {
        isect2d::Vec2 aproj = projectToAxis(_a, axes[i]);
        isect2d::Vec2 bproj = projectToAxis(_b, axes[i]);

        if (bproj.x > aproj.y || bproj.y < aproj.x) {
            return false;
        }
    }

    return true;
}

static bool intersect(const isect2d::OBB& _a, const isect2d::OBB& _b) {
    const isect2d::Vec2& c1 = _a.getCenter();
    const isect2d::Vec2& c2 = _b.getCenter();
    const isect2d::Vec2& d = c2 - c1;
    double r = std::max(_a.radius(), _b.radius());

    return d.length() <= r && axisCollide(_a, _b, _a.getAxes()) && axisCollide(_a, _b, _b.getAxes());
}

static std::set<std::pair<int, int>> intersect(const isect2d::OBB* _obbs1, const size_t _size1,
                                               const isect2d::OBB* _obbs2, const size_t _size2) {
    std::set<std::pair<int, int>> pairs;

    // bruteforce
    for (int i = 0; i < _size1; ++i) {
        const isect2d::OBB& obb1 = _obbs1[i];
        for (int j = i + 1; j < _size2; ++j) {
            const isect2d::OBB& obb2 = _obbs2[j];

            if (obb1 != obb2) {
                if (intersect(obb1, obb2)) {
                    pairs.insert({ i, j });
                }
            }
        }
    }

    return pairs;
}

#endif

