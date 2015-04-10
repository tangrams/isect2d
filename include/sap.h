#pragma once

#include "aabb.h"
#include <vector>

namespace isect2d {
    
enum class EndPointType {
    MIN, MAX
};

// 8 bytes for end points
struct EndPoint {
    int m_data;
    float m_value;
};
    
struct Box {
    AABB* m_aabb;
    
    EndPoint* m_min[2];
    EndPoint* m_max[2];
};
    
struct SAP {
public:
    SAP() {}
    void add(const Box& _aabb);
    void remove(const Box& _aabb);

private:
    std::vector<EndPoint*> m_endPoints;
    
};

}