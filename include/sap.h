#pragma once

#include "aabb.h"
#include <vector>
#include <list>
#include <memory>
#include <algorithm>

#include <iostream>

#define EPSILON 0.0001

/*
 * NOTE: Implement Boxes as a std::map instead of a std::list
 * NOTE: Implement SortListX as a std::vector instead of a std::list and compare the performance
 */

namespace isect2d {

    // 8 bytes for end points
    struct EndPoint {
        void* boxID; //same as userData
        float m_value;
        int m_aabbIndex;
        bool m_isMin = true;

        bool operator<(EndPoint _other) {
            return ( (m_value - _other.m_value) < EPSILON);
        }

        bool operator==(EndPoint _other) {
            return (boxID == _other.boxID && m_isMin == _other.m_isMin);
        }

        bool operator>(EndPoint _other) {
            return !(*this < _other);
        }

        EndPoint(void* _boxID, float _m_value, int _aabbIndex, bool _m_isMin) : boxID(_boxID), m_value(_m_value), m_aabbIndex(_aabbIndex), m_isMin(_m_isMin) {}

    };
    
    struct Box {
        void* boxID; //same as userData
    
        std::shared_ptr<EndPoint> m_min; //just do x
        std::shared_ptr<EndPoint> m_max; //just do x


        Box(void* _boxID, std::shared_ptr<EndPoint> _m_min, std::shared_ptr<EndPoint> _m_max) : boxID(_boxID), m_min(_m_min), m_max(_m_max) {}

        Box(Box&& _other) : boxID(std::move(_other.boxID)), m_min(std::move(_other.m_min)), m_max(std::move(_other.m_max)) {}

    };

    struct SAP {
    public:

        SAP() {
            Boxes.clear();
            SortListX.clear();
        }

        void updateSortedPositions(const std::list<std::shared_ptr<EndPoint>>::iterator& _epItr) {
            if(_epItr != SortListX.begin() && std::next(_epItr) != SortListX.end()) {
                auto prev = std::prev(_epItr);
                while(*(*_epItr) < *(*prev)) {
                    std::swap(*_epItr, *prev);
                    prev = std::prev(_epItr);
                }
                auto next = std::next(_epItr);
                while(*(*_epItr) > *(*next)) {
                    std::swap(*_epItr, *next);
                    next = std::next(_epItr);
                }
            } else if(_epItr == SortListX.begin()) {
                auto next = std::next(_epItr);
                while(*(*_epItr) > *(*next)) {
                    std::swap(*_epItr, *next);
                    next = std::next(_epItr);
                }
            } else {
                auto prev = std::prev(_epItr);
                while(*(*_epItr) < *(*prev)) {
                    std::swap(*_epItr, *prev);
                    prev = std::prev(_epItr);
                }
            }
        }

        void addPoint(std::vector<AABB>& _aabbs, std::vector<AABB>::iterator& _aabb) {
            auto aabbIndex = _aabb - _aabbs.begin();
            std::shared_ptr<EndPoint> ep1(new EndPoint(_aabb->m_userData, aabbIndex, _aabb->m_min.x, true));
            std::shared_ptr<EndPoint> ep2(new EndPoint(_aabb->m_userData, aabbIndex, _aabb->m_max.x, false));
            if(SortListX.size() == 0) {
                SortListX.push_front(ep2);
                SortListX.push_front(ep1);
            }
            else {
                if(*ep1 > *(SortListX.back())) {
                    SortListX.push_back(ep1);
                    SortListX.push_back(ep2);
                } else if(*ep2 < *(SortListX.front())) {
                    SortListX.push_front(ep2);
                    SortListX.push_front(ep1);
                } else {
                    SortListX.push_front(ep1);
                    updateSortedPositions(SortListX.begin());
                    SortListX.push_front(ep2);
                    updateSortedPositions(SortListX.begin());
                }
            }
            std::unique_ptr<Box> box(new Box(_aabb->m_userData, ep1, ep2));
            Boxes.push_back(std::move(box));
        }

        void updatePoints(std::vector<AABB>& _aabbs) {
            for(auto& box : Boxes) {
                const auto aabb = std::find_if(_aabbs.begin(), _aabbs.end(),
                                                [&](AABB& _aabb) {
                                                    return (_aabb.m_userData == box->boxID);
                                                });

                if(aabb != _aabbs.end()) {
                    auto aabbIndex = aabb - _aabbs.begin();
                    auto itr = std::find_if(SortListX.begin(), SortListX.end(), [&box](std::shared_ptr<EndPoint>& _ep) {return (_ep == box->m_min); });
                    box->m_min->m_value = aabb->m_min.x;
                    box->m_min->m_aabbIndex = aabbIndex;
                    updateSortedPositions(itr);
                    itr = std::find_if(SortListX.begin(), SortListX.end(), [&box](std::shared_ptr<EndPoint>& _ep) {return (_ep == box->m_max); });
                    box->m_max->m_value = aabb->m_max.x;
                    box->m_max->m_aabbIndex = aabbIndex;
                    updateSortedPositions(itr);
                } else {
                    SortListX.remove_if( [&aabb](std::shared_ptr<EndPoint>& _ep) { return (aabb->m_userData == _ep->boxID ); });
                    Boxes.remove(box);
                }
            }

            for(auto aabbItr = _aabbs.begin(); aabbItr != _aabbs.end(); aabbItr++) {
                auto box = std::find_if(Boxes.begin(), Boxes.end(),
                                        [&aabbItr](std::unique_ptr<Box>& box) {
                                            return (box->boxID == aabbItr->m_userData);
                                        });

                if(box == Boxes.end()) {
                    addPoint(_aabbs, aabbItr);
                }
            }
            
        }

        void calPairs(std::vector<AABB>& _aabbs, std::set<std::pair<int, int>>& pairs) {
            for(auto itr = SortListX.begin(); itr != SortListX.end(); itr++) {
                auto& ep1 = *itr;
                if(!(ep1->m_isMin)) {
                    continue;
                } else {
                    for(auto jitr = std::next(itr); jitr != SortListX.end(); jitr++) {
                        auto& ep2 = *jitr;
                        if(ep1->boxID == ep2->boxID) {
                            break;
                        } else if(!(ep2->m_isMin)) {
                            continue;
                        } else {
                            if (_aabbs[ep1->m_aabbIndex].intersect(_aabbs[ep2->m_aabbIndex])) {
                                pairs.insert({ep1->m_aabbIndex, ep2->m_aabbIndex });
                            }
                        }
                    }
                }
            }
        }

        std::set<std::pair<int, int>> intersect(std::vector<AABB>& _aabbs) {
            std::set<std::pair<int, int>> pairs;
            updatePoints(_aabbs);
            calPairs(_aabbs, pairs);
            return pairs;
        }

    private:
        std::list< std::shared_ptr<EndPoint> > SortListX;
        std::list< std::unique_ptr<Box> > Boxes;
    };

}

