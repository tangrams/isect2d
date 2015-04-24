#pragma once

#include "aabb.h"
#include <vector>
#include <list>
#include <memory>
#include <algorithm>

#define EPSILON 0.0001

namespace isect2d {

    // 8 bytes for end points
    struct EndPoint {
        void* boxID; //same as userData
        float m_value;
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

        EndPoint(void* _boxID, float _m_value, bool _m_isMin) : boxID(_boxID), m_value(_m_value), m_isMin(_m_isMin) {}

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

        SAP() {}

        std::set<std::pair<int, int>> intersect(std::vector<AABB>& _aabbs) {
            std::set<std::pair<int, int>> pairs;
            removePoints(_aabbs);
            updatePoints(_aabbs);
            sort();
            intersect();
            return pairs;
        }

        void removePoints(std::vector<AABB>& _aabbs) {
            for(auto& box : Boxes) {
                const auto aabb = std::find_if(_aabbs.begin(), _aabbs.end(),
                                            [&](AABB& _aabb) {
                                                return (_aabb.m_userData == box->boxID);
                                            });

                Boxes.remove(box);
                SortListX.remove_if( [&aabb](std::shared_ptr<EndPoint>& _ep) { return( aabb->m_userData == _ep->boxID ); });
            }
        }

        void updatePoints(std::vector<AABB>& _aabbs) {
            for(auto& aabb : _aabbs) {

                auto box = std::find_if(Boxes.begin(), Boxes.end(),
                                        [&](std::unique_ptr<Box>& box) {
                                            return (box->boxID == aabb.m_userData);
                                        });
                if(box != Boxes.end()) {
                    (*box)->m_min->m_value = aabb.m_min.x;
                    (*box)->m_max->m_value = aabb.m_max.x;
                } else {
                    addPoint(aabb);
                }
            }
        }

        void addPoint(const AABB& _aabb) {
            std::shared_ptr<EndPoint> ep1(new EndPoint(_aabb.m_userData, _aabb.m_min.x, true));
            std::shared_ptr<EndPoint> ep2(new EndPoint(_aabb.m_userData, _aabb.m_max.x, false));
            if(*ep1 > *(SortListX.back())) {
                SortListX.push_back(ep1);
                SortListX.push_back(ep2);
            } else {
                SortListX.push_front(ep2);
                SortListX.push_front(ep1);
            }
            std::unique_ptr<Box> box(new Box(_aabb.m_userData, ep1, ep2));
            Boxes.push_back(std::move(box));
        }

        void sort() {
            for(auto itr = SortListX.begin(); itr != SortListX.end(); itr++) {
                auto savedEP = *itr;
                auto jitr = itr;
                auto prev = jitr;
                while(jitr != SortListX.begin() && *(*(--prev)) > *savedEP) {
                    *jitr = *prev;
                    --jitr;
                }
                *jitr = savedEP;
            }
        }

        void intersect() {

        }

    private:
        std::list< std::shared_ptr<EndPoint> > SortListX;
        std::list< std::unique_ptr<Box> > Boxes;
    };

}

