#pragma once

#include <algorithm>
#include <array>
#include "vec.h"

namespace isect2d {

template<typename V>
struct OBB {

    OBB() : m_width(0), m_height(0) {}

    OBB(float _cx, float _cy, float _a, float _w, float _h) :
        m_width(_w), m_height(_h),
        m_centroid(V(_cx, _cy)) {

        rotate(_a);
        update();
    }

    OBB(V _center, V _normal, float _w, float _h) :
        m_width(_w), m_height(_h),
        m_centroid(_center),
        m_axes(_normal) {

        update();
    }

    void move(const float _px, const float _py) {
        m_centroid = V(_px, _py);

        update();
    }

    void rotate(float _angle) {
        float cosa = cos(-_angle);
        float sina = sin(-_angle);

        m_axes = { -cosa, sina };

        update();
    }

    float getAngle() const {
        return -atan2(-m_axes.y, m_axes.x);
    }

    const std::array<V, 4>& getQuad() const {
        return m_quad;
    }

    V getAxes() const{
        return m_axes;
    }

    V getCentroid() const {
        return m_centroid;
    }

    float getWidth() const {
        return m_width;
    }

    float getHeight() const {
        return m_height;
    }

    float radius() const {
        V extent(m_width, m_height);

        return extent.length();
    }

    AABB<V> getExtent() const {
        float inf = std::numeric_limits<double>::infinity();
        float aabb[4] = { inf, inf, -inf, -inf };

        for (int i = 0; i < 4; ++i) {
            aabb[0] = std::min<float>(m_quad[i].x, aabb[0]);
            aabb[1] = std::min<float>(m_quad[i].y, aabb[1]);
            aabb[2] = std::max<float>(m_quad[i].x, aabb[2]);
            aabb[3] = std::max<float>(m_quad[i].y, aabb[3]);
        }

        return { aabb[0], aabb[1], aabb[2], aabb[3] };
    }

private:

    void update() {
        V x = m_axes * (m_width / 2);
        V y = V{ -m_axes.y, m_axes.x } * (m_height / 2);

        m_quad[0] = m_centroid - x - y; // lower-left
        m_quad[1] = m_centroid + x - y; // lower-right
        m_quad[2] = m_centroid + x + y; // uper-right
        m_quad[3] = m_centroid - x + y; // uper-left
    }

    float m_width;
    float m_height;
    V m_centroid;

    V m_axes;
    std::array<V,4> m_quad;

};

template<typename V>
inline bool operator==(const OBB<V>& lh, const OBB<V>& rh) {
    return lh.getCentroid() == rh.getCentroid() && lh.m_axes == rh.m_axes;
}

template<typename V>
inline bool operator!=(const OBB<V>& lh, const OBB<V>& rh) {
    return !(lh == rh);
}

template<typename V>
static std::pair<typename V::value_type, typename V::value_type> projectToAxis(const OBB<V>& _obb, const V& axis) {
    using Value = typename V::value_type;

    Value inf = std::numeric_limits<Value>::infinity();
    Value min = inf;
    Value max = -inf;

    const std::array<V, 4>& p = _obb.getQuad();

    for (int i = 0; i < 4; ++i) {
        Value d = dot(p[i], axis);

        min = std::min(min, d);
        max = std::max(max, d);
    }

    return { min, max };
}

template<typename V>
inline static bool axisCollide(const OBB<V>& _a, const OBB<V>& _b, V axes) {

    using Value = typename V::value_type;
    using Result = std::pair<Value, Value>;

    Result aproj = projectToAxis(_a, axes);
    Result bproj = projectToAxis(_b, axes);

    if (aproj.second < bproj.first || bproj.second < aproj.first) {
        return false;
    }

    axes = V{-axes.y, axes.x};

    aproj = projectToAxis(_a, axes);
    bproj = projectToAxis(_b, axes);

    if (aproj.second < bproj.first || bproj.second < aproj.first) {
        return false;
    }

    return true;
}

template<typename V>
inline static bool intersect(const OBB<V>& _a, const OBB<V>& _b) {
    return axisCollide(_a, _b, _a.getAxes()) && axisCollide(_a, _b, _b.getAxes());
}

}
