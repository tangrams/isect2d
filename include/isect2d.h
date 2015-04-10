#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <iostream>

#include "aabb.h"
#include "vec2.h"
#include "bvh.h"
#include "sap.h"

//#define BRUTEFORCE

static std::set<std::pair<int, int>> intersect(const std::vector<isect2d::AABB>& _aabbs) {
    std::set<std::pair<int, int>> pairs;
    
    if (_aabbs.size() == 0) {
        return pairs;
    }
    
#ifdef BRUTEFORCE
    for (int i = 0; i < _aabbs.size(); ++i) {
        for (int j = i + 1; j < _aabbs.size(); ++j) {
            if (_aabbs[i].intersect(_aabbs[j])) {
                pairs.insert({ i, j });
            }
        }
    }
#else
    isect2d::BVH bvh(_aabbs);
    
    if (!bvh.getRoot()) {
        return pairs;
    }
    
    isect2d::BVHNode* node;
    std::stack<isect2d::BVHNode*> todo;
    
    for (int i = 0; i < _aabbs.size(); ++i) {
        const isect2d::AABB& aabb = _aabbs[i];
        todo.push(bvh.getRoot());
        
        while (todo.size() != 0) {
            node = todo.top();
            todo.pop();
            
            if (!node) {
                continue;
            }
            
            if (node->isLeaf()) {
                if (aabb != *node->m_aabb && aabb.intersect(*node->m_aabb)) {
                    auto it = std::find(_aabbs.begin() + i, _aabbs.end(), *node->m_aabb);
                    pairs.insert({ i, it - _aabbs.begin() });
                }
            } else {
                if (aabb.intersect(*node->m_proxy)) {
                    todo.push(node->m_leftChild);
                    todo.push(node->m_rightChild);
                }
            }
        }
    }
#endif

    return pairs;
}

