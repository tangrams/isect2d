#pragma once

#include "aabb.h"
#include <list>
#include <vector>
#include <unordered_map>
#include <memory>
#include <map>
#include <algorithm>

#include <iostream>

#define EPSILON 0.00001

namespace isect2d {

    struct EndPoint {
        void* m_boxID; //same as userData
        float m_value;
        bool m_isMin = true;

        bool operator<(const EndPoint& _other) {
            return (m_value - _other.m_value) < EPSILON;
        }

        bool operator==(const EndPoint& _other) {
            return (m_boxID == _other.m_boxID && m_isMin == _other.m_isMin);
        }

        bool operator>(const EndPoint& _other) {
            return !(*this < _other);
        }

        EndPoint(void* _boxID, float _m_value,bool _m_isMin) :
            m_boxID(_boxID), m_value(_m_value), m_isMin(_m_isMin)
        {}

        EndPoint(EndPoint&& _other) :
            m_boxID(std::move(_other.m_boxID)), m_value(std::move(_other.m_value)), m_isMin(std::move(_other.m_isMin))
        {}

    };

    struct Box {
        std::shared_ptr<EndPoint> m_min_x;
        std::shared_ptr<EndPoint> m_max_x;

        std::shared_ptr<EndPoint> m_min_y;
        std::shared_ptr<EndPoint> m_max_y;

        Box(void* _boxID, std::shared_ptr<EndPoint> _min_x, std::shared_ptr<EndPoint> _max_x, std::shared_ptr<EndPoint> _min_y, std::shared_ptr<EndPoint> _max_y) :
            m_min_x(_min_x), m_max_x(_max_x), m_min_y(_min_y), m_max_y(_max_y)
        {}

        Box(Box&& _other) :
            m_min_x(std::move(_other.m_min_x)), m_max_x(std::move(_other.m_max_x)),
            m_min_y(std::move(_other.m_min_y)), m_max_y(std::move(_other.m_max_y))
        {}

    };

    struct SAP {

    public:

        SAP() {}

        void swapEndPoints(const int _ep1Index, const int _ep2Index, const Dimension& _dim) {
            auto epTmp = std::move(m_sortList[_dim][_ep1Index]);
            m_sortList[_dim][_ep1Index] = std::move(m_sortList[_dim][_ep2Index]);
            m_sortList[_dim][_ep2Index] = std::move(epTmp);
        }

        void updateAxesCollision(const int& _i1, const int& _i2, const std::unordered_map<void*, AABB>& _aabbs, const Dimension& _dim) {
            const auto& ep1 = m_sortList[_dim][_i1];
            const auto& ep2 = m_sortList[_dim][_i2];

            if ((_i1 > _i2 && ep1->m_isMin && !ep2->m_isMin) || (_i1 < _i2 && !ep1->m_isMin && ep2->m_isMin)) {
                auto it1 = m_pairs.find({ ep1->m_boxID, ep2->m_boxID });
                auto it2 = m_pairs.find({ ep2->m_boxID, ep1->m_boxID });
                if (it1 == m_pairs.end() && it2 == m_pairs.end()) {
                    if (_aabbs.at(ep1->m_boxID).intersect(_aabbs.at(ep2->m_boxID))) {
                        m_pairs.insert({ ep1->m_boxID, ep2->m_boxID });
                    }
                }
            } else if ((_i1 > _i2 && !ep1->m_isMin && ep2->m_isMin) || (_i1 < _i2 && ep1->m_isMin && !ep2->m_isMin)) {
                auto it1 = m_pairs.find({ ep1->m_boxID, ep2->m_boxID });
                auto it2 = m_pairs.find({ ep2->m_boxID, ep1->m_boxID });
                if (it1 != m_pairs.end()) {
                    m_pairs.erase(it1);
                }
                if (it2 != m_pairs.end()) {
                    m_pairs.erase(it2);
                }
            }
        }

        void sort(int _epIndex, const std::unordered_map<void*, AABB>& _aabbs, const Dimension& _dim) {
            while (_epIndex > 0 && *m_sortList[_dim][_epIndex] < *m_sortList[_dim][_epIndex - 1]) {
                updateAxesCollision(_epIndex, _epIndex - 1, _aabbs, _dim);
                swapEndPoints(_epIndex, _epIndex - 1, _dim);
                _epIndex--;
            }
            while (_epIndex < (m_sortList[_dim].size() - 1) && *m_sortList[_dim][_epIndex] > *m_sortList[_dim][_epIndex + 1]) {
                updateAxesCollision(_epIndex, _epIndex + 1, _aabbs, _dim);
                swapEndPoints(_epIndex, _epIndex + 1, _dim);
                _epIndex++;
            }
        }

        void insertEndPoints(const std::unordered_map<void*, AABB>& _aabbs, const std::unordered_map<void*, AABB>::iterator& _aabbItr) {
            std::shared_ptr<EndPoint> epx1(new EndPoint(_aabbItr->first, _aabbItr->second.m_min.x, true));
            std::shared_ptr<EndPoint> epx2(new EndPoint(_aabbItr->first, _aabbItr->second.m_max.x, false));
            std::shared_ptr<EndPoint> epy1(new EndPoint(_aabbItr->first, _aabbItr->second.m_min.y, true));
            std::shared_ptr<EndPoint> epy2(new EndPoint(_aabbItr->first, _aabbItr->second.m_max.y, false));

            for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                std::shared_ptr<EndPoint>* ep1, *ep2;

                switch (dim) {
                    case Dimension::X:
                        ep1 = &epx1;
                        ep2 = &epx2;
                        break;
                    case Dimension::Y:
                        ep1 = &epy1;
                        ep2 = &epy2;
                        break;
                }

                if (m_sortList[dim].size() == 0) {
                    m_sortList[dim].push_back(*ep1);
                    m_sortList[dim].push_back(*ep2);
                } else if (*(*ep2) < *(*m_sortList[dim].begin())) {
                    m_sortList[dim].insert(m_sortList[dim].begin(), *ep2);
                    m_sortList[dim].insert(m_sortList[dim].begin(), *ep1);
                } else {
                    m_sortList[dim].push_back(*ep1);
                    sort(m_sortList[dim].size() - 1, _aabbs, (Dimension) dim);
                    m_sortList[dim].push_back(*ep2);
                    sort(m_sortList[dim].size() - 1, _aabbs, (Dimension) dim);
                }
            }

            std::unique_ptr<Box> box(new Box(_aabbItr->first, epx1, epx2, epy1, epy2));
            m_boxes[_aabbItr->first] = std::move(box);
        }

        void updateEndPoints(std::unordered_map<void*, AABB>& _aabbs) {
            for (auto& box : m_boxes) {
                const auto& aabb = _aabbs.find(box.first);

                if(aabb != _aabbs.end()) {
                    box.second->m_min_x->m_value = aabb->second.m_min.x;
                    box.second->m_max_x->m_value = aabb->second.m_max.x;
                    box.second->m_min_y->m_value = aabb->second.m_min.y;
                    box.second->m_max_y->m_value = aabb->second.m_max.y;
                } else {
                    for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                        std::remove_if(m_sortList[dim].begin(), m_sortList[dim].end(), [&aabb](std::shared_ptr<EndPoint>& _ep) {
                            return (aabb->first == _ep->m_boxID);
                        });
                    }

                    m_boxes.erase(aabb->first);
                }
            }

            for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                for (int i = 0; i < m_sortList[dim].size(); ++i) {
                    sort(i, _aabbs, (Dimension) dim);
                }
            }

            for (auto aabbItr = _aabbs.begin(); aabbItr != _aabbs.end(); aabbItr++) {
                if (m_boxes.find(aabbItr->first) == m_boxes.end()) {
                    insertEndPoints(_aabbs, aabbItr);
                }
            }
        }

        void intersect(std::unordered_map<void*, AABB>& _aabbs) {
            if (m_sortList[Dimension::X].size() == 0) {
                for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                    m_sortList[dim].reserve(2 * _aabbs.size());
                }
            }
            updateEndPoints(_aabbs);
        }

        const std::set<std::pair<void*, void*>>& getPairs() const {
            return m_pairs;
        }

        void clearSAP() {
            for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                m_sortList[dim].clear();
            }

            m_boxes.clear();
            m_pairs.clear();
        }

        std::vector<std::shared_ptr<EndPoint>> m_sortList[Dimension::MAX_DIM];

    private:
        std::set<std::pair<void*, void*>> m_pairs;
        std::unordered_map<void*, std::unique_ptr<Box>> m_boxes;

    };

}
