#pragma once

#include "aabb.h"
#include <list>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include <iostream>

#define EPSILON 0.00001

/*
 * NOTE: Implement Boxes as a std::unordered_map instead of a std::list
 * NOTE: Implement SortListX as a std::vector instead of a std::list and compare the performance
 */

namespace isect2d {

    struct EndPoint {
        void* boxID; //same as userData
        float m_value;
        bool m_isMin = true;

        bool operator<(const EndPoint& _other) {
            return ( (m_value - _other.m_value) < EPSILON);
        }

        bool operator==(const EndPoint& _other) {
            return (boxID == _other.boxID && m_isMin == _other.m_isMin);
        }

        bool operator>(const EndPoint& _other) {
            return !(*this < _other);
        }

        EndPoint(void* _boxID, float _m_value,bool _m_isMin) : boxID(_boxID), m_value(_m_value), m_isMin(_m_isMin) {}
        
        EndPoint(EndPoint&& _other) : boxID(std::move(_other.boxID)), m_value(std::move(_other.m_value)), m_isMin(std::move(_other.m_isMin)) {}

    };
    
    struct Box {
        std::shared_ptr<EndPoint> m_min; //just do x
        std::shared_ptr<EndPoint> m_max; //just do x


        Box(void* _boxID, std::shared_ptr<EndPoint> _m_min, std::shared_ptr<EndPoint> _m_max) : m_min(_m_min), m_max(_m_max) {}

        Box(Box&& _other) : m_min(std::move(_other.m_min)), m_max(std::move(_other.m_max)) {}

    };

    struct SAP {
        
    public:
        
        SAP() {
            m_boxes.clear();
            m_sortListX.clear();
        }
        
        void swapEPs(const int _ep1Index, const int _ep2Index) {
            auto epTmp = std::move(m_sortListX[_ep1Index]);
            m_sortListX[_ep1Index] = std::move(m_sortListX[_ep2Index]);
            m_sortListX[_ep2Index] = std::move(epTmp);
        }
        
        void addPair(const int& _i1, const int& _i2, const std::unordered_map<void*, AABB>& _aabbs) {
            
            if(_i1 != _i2) {
                if( (_i1 > _i2 && m_sortListX[_i1]->m_isMin && !m_sortListX[_i2]->m_isMin) ||
                    (_i1 < _i2 && !m_sortListX[_i1]->m_isMin && m_sortListX[_i2]->m_isMin)) {
                    
                    if(_aabbs.at(m_sortListX[_i1]->boxID).intersect(_aabbs.at(m_sortListX[_i2]->boxID))) {
                        if(m_pairs.find({m_sortListX[_i2]->boxID, m_sortListX[_i1]->boxID}) == m_pairs.end()) {
                            
                            // TODO : test on both Y / X axes using counter to know whether the box collide on one axis
                            m_pairs.insert({ m_sortListX[_i1]->boxID, m_sortListX[_i2]->boxID });
                        }
                    }
                    
                } else if((_i1 > _i2 && !m_sortListX[_i1]->m_isMin && m_sortListX[_i2]->m_isMin) ||
                          (_i1 < _i2 && m_sortListX[_i1]->m_isMin && !m_sortListX[_i2]->m_isMin)) {
                    m_pairs.erase({m_sortListX[_i1]->boxID, m_sortListX[_i2]->boxID});
                }
            }
        }
        void updateSortedPositions(int _epIndex, const std::unordered_map<void*, AABB>& _aabbs) {
            while(_epIndex > 0 && _epIndex < m_sortListX.size() && *m_sortListX[_epIndex] < *m_sortListX[_epIndex-1]) {
                addPair(_epIndex, _epIndex-1, _aabbs);
                swapEPs(_epIndex, _epIndex-1);
                _epIndex--;
            }
            while(_epIndex < (m_sortListX.size()-1) && *m_sortListX[_epIndex] > *m_sortListX[_epIndex+1]) {
                addPair(_epIndex, _epIndex+1, _aabbs);
                swapEPs(_epIndex, _epIndex+1);
                _epIndex++;
            }
        }
        
        void addPoint(const std::unordered_map<void*, AABB>& _aabbs, const std::unordered_map<void*, AABB>::iterator& _aabbItr) {
            
            std::shared_ptr<EndPoint> ep1(new EndPoint(_aabbItr->first, _aabbItr->second.m_min.x, true));
            std::shared_ptr<EndPoint> ep2(new EndPoint(_aabbItr->first, _aabbItr->second.m_max.x, false));
            
            if(m_sortListX.size() == 0) {
                m_sortListX.push_back(ep1);
                m_sortListX.push_back(ep2);
            } else if(*ep2 < *(*m_sortListX.begin())) {
                m_sortListX.insert(m_sortListX.begin(), ep2);
                m_sortListX.insert(m_sortListX.begin(), ep1);
            } else {
                m_sortListX.push_back(ep1);
                updateSortedPositions(m_sortListX.size()-1, _aabbs);
                m_sortListX.push_back(ep2);
                updateSortedPositions(m_sortListX.size()-1, _aabbs);
            }
            
            std::unique_ptr<Box> box(new Box(_aabbItr->first, ep1, ep2));
            m_boxes[_aabbItr->first] = std::move(box);
        }
        
        void updatePoints(std::unordered_map<void*, AABB>& _aabbs) {
            for(auto& box : m_boxes) {
                const auto& aabb = _aabbs.find(box.first);
                
                if(aabb != _aabbs.end()) {
                    
                    box.second->m_min->m_value = aabb->second.m_min.x;
                    box.second->m_max->m_value = aabb->second.m_max.x;
                
                } else {
                    std::remove_if(m_sortListX.begin(), m_sortListX.end(), [&aabb](std::shared_ptr<EndPoint>& _ep) {
                        return (aabb->first == _ep->boxID );
                    });
                    
                    m_boxes.erase(aabb->first);
                }
            }
            
            for (int i = 0; i < m_sortListX.size(); ++i) {
                updateSortedPositions(i, _aabbs);
            }

            for(auto aabbItr = _aabbs.begin(); aabbItr != _aabbs.end(); aabbItr++) {
                auto box = m_boxes.find(aabbItr->first);
                if(box == m_boxes.end()) {
                    addPoint(_aabbs, aabbItr);
                }
            }
        }

        void intersect(std::unordered_map<void*, AABB>& _aabbs) {
            if(m_sortListX.size() == 0) {
                m_sortListX.reserve(2 * _aabbs.size());
            }
            updatePoints(_aabbs);
            
            // TODO : loop over counters and insert only if collide on Y + X
        }
        
        std::set<std::pair<void*, void*>>& getPairs() {
            return m_pairs;
        }
        
        void clearSAP() {
            m_sortListX.clear();
            m_boxes.clear();
            m_pairs.clear();
        }

    private:
        std::vector<std::shared_ptr<EndPoint>> m_sortListX;
        std::unordered_map<void*,std::unique_ptr<Box>> m_boxes;
        std::set<std::pair<void*, void*>> m_pairs;
    };

}

