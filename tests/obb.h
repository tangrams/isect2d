#pragma once

#include "isect2d.h"
#include <algorithm>

struct OBB {

    OBB(float _cx, float _cy, float _a, float _w, float _h) :
        m_center(isect2d::Vec2(_cx, _cy)), m_angle(_a), m_width(_w), m_height(_h) {

        update();
    }

    void move(const float _px, const float _py) {
        m_center = isect2d::Vec2(_px, _py);

        update();
    }

    void rotate(float _angle) {
        m_angle = _angle;

        update();
    }

    const isect2d::Vec2* getQuad() const {
        return m_quad;
    }

    const isect2d::Vec2* getAxes() const{
        return m_axes;
    }

    const isect2d::Vec2& getCenter() const {
        return m_center;
    }

    float getAngle() const {
        return m_angle;
    }

    float getWidth() const {
        return m_width;
    }

    float getHeight() const {
        return m_height;
    }

    float radius() const {
        isect2d::Vec2 extent(m_width, m_height);

        return extent.length();
    }

    isect2d::AABB getExtent() const {
        float inf = std::numeric_limits<double>::infinity();
        float aabb[4] = { inf, inf, -inf, -inf };

        for (int i = 0; i < 4; ++i) {
            aabb[0] = std::min<float>(m_quad[i].x, aabb[0]);
            aabb[1] = std::min<float>(m_quad[i].y, aabb[1]);
            aabb[2] = std::max<float>(m_quad[i].x, aabb[2]);
            aabb[3] = std::max<float>(m_quad[i].y, aabb[3]);
        }

        return isect2d::AABB(aabb[0], aabb[1], aabb[2], aabb[3]);
    }

private:

    void perpAxes() {
        m_axes[0] = (m_quad[2] - m_quad[3]).normalize();
        m_axes[1] = (m_quad[2] - m_quad[1]).normalize();
    }

    void update() {
        isect2d::Vec2 x( cos(m_angle), sin(m_angle));
        isect2d::Vec2 y(-sin(m_angle), cos(m_angle));

        x = x * (m_width / 2);
        y = y * (m_height / 2);

        m_quad[0] = m_center - x - y; // lower-left
        m_quad[1] = m_center + x - y; // lower-right
        m_quad[2] = m_center + x + y; // uper-right
        m_quad[3] = m_center - x + y; // uper-left

        perpAxes();
    }

    float m_width;
    float m_height;
    float m_angle;

    isect2d::Vec2 m_axes[2];
    isect2d::Vec2 m_center;
    isect2d::Vec2 m_quad[4];

};

bool operator==(const OBB& lh, const OBB& rh) {
    return lh.getCenter() == rh.getCenter() && lh.getAngle() == rh.getAngle();
}

bool operator!=(const OBB& lh, const OBB& rh) {
    return !(lh == rh);
}

static isect2d::Vec2 projectToAxis(const OBB& _obb, const isect2d::Vec2& axis) {
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

static bool axisCollide(const OBB& _a, const OBB& _b, const isect2d::Vec2* axes) {
    for (int i = 0; i < 2; ++i) {
        isect2d::Vec2 aproj = projectToAxis(_a, axes[i]);
        isect2d::Vec2 bproj = projectToAxis(_b, axes[i]);

        if (bproj.x > aproj.y || bproj.y < aproj.x) {
            return false;
        }
    }

    return true;
}

static bool intersect(const OBB& _a, const OBB& _b) {
    return axisCollide(_a, _b, _a.getAxes()) && axisCollide(_a, _b, _b.getAxes());
}


