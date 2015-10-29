#include "isect2d.h"
#include "vec2.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <stack>

#include <GLFW/glfw3.h>

#ifdef USE_GLM
#include <glm/glm.hpp>
#endif

//#define CIRCLES
#define N_CIRCLES 500

#define AREA
#define N_BOX 2000

GLFWwindow* window;
float width = 800;
float height = 600;
float dpiRatio = 1;

bool isPause = false;

#ifdef USE_GLM
using Vec2 = glm::vec2;
#else
using Vec2 = isect2d::Vec2;
#endif

using OBB = isect2d::OBB<Vec2>;
using AABB = isect2d::AABB<Vec2>;

std::vector<OBB> obbs;

float rand_0_1(float scale) {
    return ((float)rand() / (float)(RAND_MAX)) * scale;
}

void update() {
    double time = glfwGetTime();

    if(!isPause) {
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
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(key == 'P' && action == GLFW_PRESS) {
        isPause = !isPause;
    }
}

void initBBoxes() {

#if defined CIRCLES
    int n = N_BOX;
    float o = (2 * M_PI) / n;
    float size = 200;
    float boxSize = n / (0.4 * n);

    for (int i = 0; i < n; ++i) {
        float r = rand_0_1(20);

        obbs.push_back(OBB(cos(o * i) * size + width / 2,
                    sin(o * i) * size + height / 2, r,
                    r + boxSize * 8, r * boxSize / 3 + boxSize));
    }
#elif defined AREA
    int n = N_BOX;
    float boxWidth = 10;
    float boxHeight = 5;

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
        obbs.push_back(OBB(xVal, yVal, angle+M_PI*i/4,
                           boxWidth-boxSizeFactorW,
                           boxHeight-boxSizeFactorH));
    }
#else
    int n = 10;
    float o = (2 * M_PI) / n;
    float size = 50;
    float boxSize = 15;

    for (int i = 0; i < n; ++i) {
        float r = rand_0_1(20);

        obbs.push_back(OBB(cos(o * i) * size + width / 2,
                    sin(o * i) * size + height / 2, r,
                    r + boxSize * 8, r * boxSize / 3 + boxSize));
    }
#endif

}

void init() {
    glfwInit();

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    window = glfwCreateWindow(width, height, "isect2d", NULL, NULL);

    if (!window) {
        glfwTerminate();
    }

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glfwSetKeyCallback(window, keyCallback);

    dpiRatio = fbWidth / width;

    glfwMakeContextCurrent(window);

    initBBoxes();
}

void line(float sx, float sy, float ex, float ey) {
    glBegin(GL_LINES);
    glVertex2f(sx, sy);
    glVertex2f(ex, ey);
    glEnd();
}

void cross(float x, float y, float size = 3) {
    line(x - size, y, x + size, y);
    line(x, y - size, x, y + size);
}

void drawAABB(const AABB& _aabb) {
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMin().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMin().y);
    line(_aabb.getMax().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMax().y, _aabb.getMax().x, _aabb.getMax().y);
}

void drawOBB(const OBB& obb, bool isect) {
    const auto& quad = obb.getQuad();

    for(int i = 0; i < 4; ++i) {
        if(isect) {
            glColor4f(0.5, 1.0, 0.5, 1.0);
        } else {
            glColor4f(1.0, 0.5, 0.5, 1.0);
        }

        auto start = quad[i];
        auto end = quad[(i + 1) % 4];

        line(start.x, start.y, end.x, end.y);

        glColor4f(1.0, 1.0, 1.0, 0.1);
        cross(obb.getCentroid().x, obb.getCentroid().y, 2);
    }
}

void render() {

    const int n1 = 4;
    const int n2 = 16;

    isect2d::ISect2D<Vec2> context;
    context.resize({n2, n2}, {800, 600});

    while (!glfwWindowShouldClose(window)) {
        update();

        if (isPause) {
            glfwPollEvents();
            continue;
        }

        glViewport(0, 0, width * dpiRatio, height * dpiRatio);
        glClearColor(0.18f, 0.18f, 0.22f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width, 0, height, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        for (auto& obb : obbs) {
            drawOBB(obb, false);
        }

        // bruteforce broadphase
        {
            std::unordered_set<std::pair<int, int>> pairs;
            std::vector<AABB> aabbs;

            for (auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                aabbs.push_back(aabb);
            }

            const clock_t beginBroadPhaseTime = clock();
            pairs = intersect(aabbs);
            float broadTime = (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000;

            // narrow phase
            clock_t beginNarrowTime = clock();
            int collisions = 0;
            for (auto& pair : pairs) {
                if (intersect(obbs[pair.first], obbs[pair.second]))
                    collisions++;
            }
            float narrowTime = (float(clock() - beginNarrowTime) / CLOCKS_PER_SEC) * 1000;

            std::cout << "0 - broadphase: " << broadTime
                      << "\t narrowphase: " << narrowTime << "ms"
                      << "\t pairs: " << pairs.size()
                      << "\t collision: " << collisions
                      << std::endl;
        }

        // grid broad phase
        {
            std::vector<AABB> aabbs;
            std::unordered_set<std::pair<int, int>> pairs;

            for (auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                aabbs.push_back(aabb);
            }

            const clock_t beginBroadPhaseTime = clock();
            pairs = intersect(aabbs, {n1, n1}, {800, 600});
            float broadTime = (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000;

            // narrow phase
            clock_t beginNarrowTime = clock();
            int collisions = 0;
            for (auto& pair : pairs) {
                if (intersect(obbs[pair.first], obbs[pair.second]))
                    collisions++;
            }
            float narrowTime = (float(clock() - beginNarrowTime) / CLOCKS_PER_SEC) * 1000;

            std::cout << "1 - broadphase: " << broadTime
                      << "\t narrowphase: " << narrowTime << "ms"
                      << "\t pairs: " << pairs.size()
                      << "\t collision: " << collisions
                      << std::endl;
        }

        // grid broad phase
        {
            std::vector<AABB> aabbs;
            for (auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                aabbs.push_back(aabb);
            }

            const clock_t beginBroadPhaseTime = clock();
            context.clear();
            context.intersect(aabbs);
            float broadTime = (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000;

            // narrow phase
            clock_t beginNarrowTime = clock();
            int collisions = 0;
            for (auto& pair : context.pairs) {
                if (intersect(obbs[pair.first], obbs[pair.second]))
                    collisions++;
            }
            float narrowTime = (float(clock() - beginNarrowTime) / CLOCKS_PER_SEC) * 1000;

            std::cout << "2 - broadphase: " << broadTime
                      << "\t narrowphase: " << narrowTime << "ms"
                      << "\t pairs: " << context.pairs.size()
                      << "\t collision: " << collisions
                      << std::endl;
        }

        std::cout << std::endl;

        // narrow phase
        {
            for (auto& pair : context.pairs) {
                auto obb1 = obbs[pair.first];
                auto obb2 = obbs[pair.second];

                bool isect = intersect(obb1, obb2);

                if (isect) {
                    drawOBB(obb1, true);
                    drawOBB(obb2, true);

                    line(obb1.getCentroid().x, obb1.getCentroid().y,
                         obb2.getCentroid().x, obb2.getCentroid().y);
                }
            }
        }

        glfwSwapBuffers(window);

        glfwPollEvents();
    }
}

int main() {

    init();
    render();

    return 0;
}
