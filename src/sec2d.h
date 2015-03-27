#ifndef SEC2D_H
#define SEC2D_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

namespace sec2d {
struct OBB;
}

bool intersect(const sec2d::OBB& _a, const sec2d::OBB& _b);

std::set<std::pair<int, int>> intersect(const sec2d::OBB* _obbs1, const size_t _size1,
                                        const sec2d::OBB* _obbs2, const size_t _size2);

#endif

#ifdef SEC2D_IMPLEMENTATION

namespace sec2d {

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

    Vec2 mult(const Vec2& _b) const {
        return Vec2(x * _b.x, y * _b.y);
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

    double operator%(Vec2& _b) { // cross
        return x * _b.y - _b.x * y;
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
        
        quad[0] = center - x - y; // lower-left
        quad[1] = center + x - y; // lower-right
        quad[2] = center + x + y; // uper-right
        quad[3] = center - x + y; // uper-left
        
        perpAxes();
    }
    
    void move(const double _px, const double _py) {
        Vec2 t(_px, _py);
        
        for (int i = 0; i < 4; ++i) {
            quad[i] = quad[i] + t;
        }
        
        perpAxes();
    }
    
    void perpAxes() {
        axes[0] = Vec2(quad[2].x - quad[3].x, quad[2].y - quad[3].y);
        axes[1] = Vec2(quad[2].x - quad[1].x, quad[2].y - quad[1].y);
    }
    
    const Vec2* getQuad() const {
        return quad;
    }
    
    const Vec2* getAxes() const{
        return axes;
    }
    
private:
    Vec2 quad[4];
    Vec2 axes[2];
};

}

sec2d::Vec2 projectToAxis(const sec2d::OBB& _obb, const sec2d::Vec2& axis) {
    double inf = std::numeric_limits<double>::infinity();
    double min = inf;
    double max = -inf;
    
    const sec2d::Vec2* p = _obb.getQuad();
    
    for (int i = 0; i < 4; ++i) {
        sec2d::Vec2 proj = project(p[i], axis);
        double d = proj.dot(axis);
        
        min = std::min(min, d);
        max = std::max(max, d);
    }
    
    return sec2d::Vec2(min, max);
}

bool intersect(const sec2d::OBB& _a, const sec2d::OBB& _b) {
    bool overlaps[2] = { false };
    const sec2d::Vec2* aaxes = _a.getAxes();
    const sec2d::Vec2* baxes = _b.getAxes();

    for (int i = 0; i < 2; ++i) {
        sec2d::Vec2 aproj = projectToAxis(_a, aaxes[i]);
        sec2d::Vec2 bproj = projectToAxis(_b, baxes[i]);

        if (bproj.x <= aproj.y && bproj.y >= aproj.x) {
            overlaps[i] = true;
        }
    }

    return overlaps[0] && overlaps[1];
}

std::set<std::pair<int, int>> intersect(const sec2d::OBB* _obbs1, const size_t _size1, const sec2d::OBB* _obbs2, const size_t _size2) {

}

#endif

