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

    // 8 bytes for end points
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
            Boxes.clear();
            SortListX.clear();
        }
        
        void swapEPs(const int _ep1Index, const int _ep2Index) {
            auto epTmp = std::move(SortListX[_ep1Index]);
            SortListX[_ep1Index] = std::move(SortListX[_ep2Index]);
            SortListX[_ep2Index] = std::move(epTmp);
        }
        
        void addPair(const int& _i1, const int& _i2, const std::unordered_map<void*, AABB>& _aabbs) {
            
            if( _i1 != _i2 ) {
                if( (_i1 > _i2 && SortListX[_i1]->m_isMin && !SortListX[_i2]->m_isMin) ||
                    (_i1 < _i2 && !SortListX[_i1]->m_isMin && SortListX[_i2]->m_isMin) ) {
                    
                    if(_aabbs.at(SortListX[_i1]->boxID).intersect(_aabbs.at(SortListX[_i2]->boxID))) {
                        if(Pairs.find({SortListX[_i2]->boxID, SortListX[_i1]->boxID}) == Pairs.end()) {
                            
                            // TODO : test on both Y / X axes using counter to know whether the box collide on one axis
                            Pairs.insert({SortListX[_i1]->boxID, SortListX[_i2]->boxID});
                        }
                    }
                    
                } else if((_i1 > _i2 && !SortListX[_i1]->m_isMin && SortListX[_i2]->m_isMin) ||
                          (_i1 < _i2 && SortListX[_i1]->m_isMin && !SortListX[_i2]->m_isMin) ) {
                    Pairs.erase({SortListX[_i1]->boxID, SortListX[_i2]->boxID});
                }
            }
        }
        void updateSortedPositions(int _epIndex, const std::unordered_map<void*, AABB>& _aabbs) {
            while(_epIndex > 0 && _epIndex < SortListX.size() && *SortListX[_epIndex] < *SortListX[_epIndex-1]) {
                addPair(_epIndex, _epIndex-1, _aabbs);
                swapEPs(_epIndex, _epIndex-1);
                _epIndex--;
            }
            while(_epIndex < (SortListX.size()-1) && *SortListX[_epIndex] > *SortListX[_epIndex+1]) {
                addPair(_epIndex, _epIndex+1, _aabbs);
                swapEPs(_epIndex, _epIndex+1);
                _epIndex++;
            }
        }
        
        void addPoint(const std::unordered_map<void*, AABB>& _aabbs, const std::unordered_map<void*, AABB>::iterator& _aabbItr) {
            
            std::shared_ptr<EndPoint> ep1(new EndPoint(_aabbItr->first, _aabbItr->second.m_min.x, true));
            std::shared_ptr<EndPoint> ep2(new EndPoint(_aabbItr->first, _aabbItr->second.m_max.x, false));
            if(SortListX.size() == 0) {
                SortListX.push_back(ep1);
                SortListX.push_back(ep2);
            } else if(*ep2 < *(*SortListX.begin())) {
                SortListX.insert(SortListX.begin(), ep2);
                SortListX.insert(SortListX.begin(), ep1);
            } else {
                    SortListX.push_back(ep1);
                    updateSortedPositions(SortListX.size()-1, _aabbs);
                    SortListX.push_back(ep2);
                    updateSortedPositions(SortListX.size()-1, _aabbs);
            }
            std::unique_ptr<Box> box(new Box(_aabbItr->first, ep1, ep2));
            Boxes[_aabbItr->first] = std::move(box);
        }
        
        void updatePoints(std::unordered_map<void*, AABB>& _aabbs) {
            for(auto& box : Boxes) {
                const auto& aabb = _aabbs.find(box.first);
                
                if(aabb != _aabbs.end()) {
                    
                    box.second->m_min->m_value = aabb->second.m_min.x;
                    box.second->m_max->m_value = aabb->second.m_max.x;
                
                } else {
                    std::remove_if( SortListX.begin(), SortListX.end(), [&aabb](std::shared_ptr<EndPoint>& _ep) { return (aabb->first == _ep->boxID ); });
                    Boxes.erase(aabb->first);
                }
            }
            
            for (int i = 0; i < SortListX.size(); ++i) {
                updateSortedPositions(i, _aabbs);
            }

            for(auto aabbItr = _aabbs.begin(); aabbItr != _aabbs.end(); aabbItr++) {
                auto box = Boxes.find(aabbItr->first);
                if(box == Boxes.end()) {
                    addPoint(_aabbs, aabbItr);
                }
            }
        }

        void intersect(std::unordered_map<void*, AABB>& _aabbs) {
            total = 0.0f;
            if(SortListX.size() == 0) {
                SortListX.reserve(_aabbs.size());
            }
            updatePoints(_aabbs);
            
            // TODO : loop over counters and insert only if collide on Y + X
            
            std::cout << std::endl << " method time " << total << " ms" << std::endl;
            total = 0.0f;
        }
        
        std::set<std::pair<void*, void*>>& getPairs() {
            return Pairs;
        }
        
        void clearSAP() {
            SortListX.clear();
            Boxes.clear();
            Pairs.clear();
        }

    private:
        float total;
        std::vector<std::shared_ptr<EndPoint>> SortListX;
        std::unordered_map<void*,std::unique_ptr<Box>> Boxes;
        std::set<std::pair<void*, void*>> Pairs;
    };

}

