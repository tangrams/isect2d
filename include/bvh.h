#pragma once

#include "aabb.h"
#include <vector>
#include <algorithm>
#include <stack>

namespace isect2d {

struct BVHNode {
    AABB* m_proxy;
    AABB* m_aabb;
    BVHNode* m_leftChild;
    BVHNode* m_rightChild;
    
    bool isLeaf() {
        return m_aabb != nullptr;
    }
    
};

class CentroidSorter {
    
    private:
    
        Dimension m_dim;

    public:
    
        CentroidSorter(Dimension _dim) : m_dim(_dim) {}

        bool operator() (AABB* _a, AABB* _b) {
            Vec2 c1 = _a->getCentroid();
            Vec2 c2 = _b->getCentroid();

            return c1[m_dim] < c2[m_dim];
        }

};

struct BVH {

    BVH(std::vector<AABB*>& _aabbs) {
        build(_aabbs);
    }
    
    BVHNode* getRoot() {
        return m_root;
    }
    
    ~BVH() {
        BVHNode* node;
        std::stack<BVHNode*> todo;
        todo.push(m_root);
        
        while (todo.size() != 0) {
            node = todo.top();
            todo.pop();
            
            if (!node)
                continue;
            
            if (node->m_proxy) {
                delete node->m_proxy;
            }
            
            delete node;
            
            todo.push(node->m_leftChild);
            todo.push(node->m_rightChild);
        }
    }

private:
    
    BVHNode* m_root;

    void build(std::vector<AABB*>& _aabbs) {
        m_root = subdivide(_aabbs, _aabbs.begin(), _aabbs.end());
    }

    BVHNode* subdivide(std::vector<AABB*>& _aabbs, std::vector<AABB*>::iterator _begin, std::vector<AABB*>::iterator _end) {

        // leaf
        if (_begin == _end) {
            return new BVHNode { nullptr, *_begin, nullptr, nullptr };
        }

        AABB* extent = new AABB();

        for (auto it = _begin; it != _end; ++it) {
            *extent = unionAABB(*extent, *(*it));
        }

        // find the separating axis dimension (SAH)
        Dimension dim = extent->maxExtent();
        Vec2 split = (extent->m_min[dim] + extent->m_max[dim]) * 0.5f;
        float length = split.length();

        // partition
        std::sort(_begin, _end, CentroidSorter(dim));

        int isplit = 0;
        for (auto it = _begin; it != _end; ++it) {
            if ((*it)->getCentroid()[dim] < length) {
                isplit++;
            } else {
                break;
            }
        }

        BVHNode* leftChild = subdivide(_aabbs, _begin, _begin + isplit);
        BVHNode* rightChild = subdivide(_aabbs, _begin + isplit + 1, _end);

        return new BVHNode { extent, nullptr, leftChild, rightChild };
    }
    
};

}
