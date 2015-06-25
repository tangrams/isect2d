
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <stack>

#include "Box2D.h"
#include "isect2d.h"

#define N_BOX 2000
#define AREA

float width = 800;
float height = 600;
float dpiRatio = 1;

std::vector<isect2d::OBB> obbs;

b2World* world;

float rand_0_1(float scale) {
    return ((float)rand() / (float)(RAND_MAX)) * scale;
}

void update() {
    static float time = 0.f;
    time += 0.016f;
    int i = 0;

    for (auto& obb : obbs) {
        float r1 = rand_0_1(10);
        float r2 = rand_0_1(20);
        float r3 = rand_0_1(M_PI);

        auto centroid = obb.getCentroid();

        if (++i % 2 == 0) {
            obb.move(centroid.x, centroid.y + .02 * cos(time * 0.25) * r2);
            obb.rotate(cos(r3) * 0.1 + obb.getAngle());
        } else {
            obb.move(centroid.x + 0.1 * cos(time) * r1, centroid.y);
            obb.rotate(cos(r2) * 0.1 + obb.getAngle());
        }
    }
}

void initBBoxes() {

    int n = N_BOX;
    float boxSize = 5;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> xDistribution(-350.0,350.0);
    std::uniform_real_distribution<double> yDistribution(-250.0,250.0);
    std::uniform_real_distribution<double> boxScaleDist(-2.0f,2.0f);
    for(int i = 0; i < n; i++) {
        float boxSizeFactorW = boxScaleDist(generator);
        float boxSizeFactorH = boxScaleDist(generator);
        float xVal = xDistribution(generator) + width/2.0f;
        float yVal = yDistribution(generator) + height/2.0f;
        float angle = yVal/(xVal+1.0f);
        obbs.push_back(isect2d::OBB(xVal, yVal, angle+M_PI*i/4, boxSize-boxSizeFactorW, boxSize-boxSizeFactorH));
    }

}

class ContactListener : public b2ContactListener {
    void BeginContact(b2Contact* contact) {}
    void EndContact(b2Contact* contact) {}
};

ContactListener contactListener;

void init() {

    initBBoxes();

    // init box2d
    world = new b2World({ 0.f, 0.f });

    world->SetContactListener(&contactListener);

    for (auto& obb : obbs) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(obb.getCentroid().x, obb.getCentroid().y);
        bodyDef.userData = &obb;
        b2Body *body = world->CreateBody(&bodyDef);

        obb.m_userData = (void*)body;
        auto aabb = obb.getExtent();

        b2PolygonShape shape;
        shape.SetAsBox(aabb.getMax().x - aabb.getMin().x, aabb.getMax().y - aabb.getMin().y);

        b2FixtureDef shapeDef;
        shapeDef.shape = &shape;
        body->CreateFixture(&shapeDef);
    }
}

void isct2d() {
    // grid broad phase
    std::vector<isect2d::AABB> aabbs;
    std::set<std::pair<int, int>> pairs;
    {
        const clock_t beginBroadPhaseTime = clock();

        for (auto& obb : obbs) {
            auto aabb = obb.getExtent();
            aabb.m_userData = (void*)&obb;
            aabbs.push_back(aabb);
        }
        pairs = intersect(aabbs, {4, 4}, {800, 600});

        std::cout << "grid broadphase: " << (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000 << "ms ";
    }

    // narrow phase
    {
        clock_t narrowTime = 0;

        for (auto pair : pairs) {
            clock_t beginNarrowTime;

            auto obb1 = obbs[pair.first];
            auto obb2 = obbs[pair.second];

            // narrow phase
            beginNarrowTime = clock();
            bool isect = intersect(obb1, obb2);
            narrowTime += (clock() - beginNarrowTime);
        }

        std::cout << "narrowphase: " << (float(narrowTime) / CLOCKS_PER_SEC) * 1000 << "ms" << std::endl;
    }
}

void box2d() {

    clock_t begin = clock();
    world->Step(0.016, 0, 0);
    clock_t end = clock();
    std::cout << "whole loop box2d: " << (float(end - begin) / CLOCKS_PER_SEC) * 1000 << "ms" << std::endl;

    for(b2Body *b = world->GetBodyList(); b; b=b->GetNext()) {
        if (b->GetUserData() != NULL) {
            isect2d::OBB *obb = (isect2d::OBB *)b->GetUserData();
            b2Vec2 b2Position = b2Vec2(obb->getCentroid().x, obb->getCentroid().y);
            b->SetTransform(b2Position, obb->getAngle());
        }
    }
}

void render() {

    while (true) {
        update();

        isct2d();
        box2d();
    }
}

int main() {

    init();
    render();

    return 0;
}

