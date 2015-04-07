#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <iostream>

#include "aabb.h"
#include "vec2.h"
#include "bvh.h"

//#define BRUTEFORCE

static std::set<std::pair<int, int>> intersect(const std::vector<isect2d::AABB>& _aabbs) {
    std::set<std::pair<int, int>> pairs;

    int nbIter = 0;
    
#ifdef BRUTEFORCE
    for (int i = 0; i < _aabbs.size(); ++i) {
        for (int j = i + 1; j < _aabbs.size(); ++j) {
            nbIter++;
            if (_aabbs[i].intersect(_aabbs[j])) {
                pairs.insert({ i, j });
            }
        }
    }
#else
    isect2d::BVH bvh(_aabbs);
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
            
            if (node->m_proxy && aabb.intersect(*node->m_proxy)) {
                todo.push(node->m_leftChild);
                todo.push(node->m_rightChild);
            }
            
            nbIter++;
            
            if (node->isLeaf() && aabb != *node->m_aabb) {
                if (aabb.intersect(*node->m_aabb)) {
                    auto it = std::find(_aabbs.begin(), _aabbs.end(), *node->m_aabb);
                    pairs.insert({ i, it - _aabbs.begin() });
                }
            }
        }
    }
#endif

    return pairs;
}

