
#include "obb.h"
#include "isect2d.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <memory>
#include <random>
#include <stack>

#include <GLFW/glfw3.h>

#define N_BOX 2000
#define AREA
#define SWEEPPRUNE
#define BRUTEFORCE
//#define CIRCLES
//#define DEBUG_DRAW
//#define FALL

GLFWwindow* window;
float width = 800;
float height = 600;
float dpiRatio = 1;

bool pause = false;

std::vector<OBB> obbs;

float rand_0_1(float scale) {
    return ((float)rand() / (float)(RAND_MAX)) * scale;
}

void update() {
    double time = glfwGetTime();
    
    if(!pause) {
        int i = 0;
        
#ifdef FALL
        for (auto& obb : obbs) {
            auto centroid = obb.getCentroid();
            
            if (++i % 2 == 0) {
                obb.move(centroid.x, centroid.y + .3);
            } 
        }
#else
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
#endif
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(key == 'P' && action == GLFW_PRESS) {
        pause = !pause;
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

        obbs.push_back(OBB(cos(o * i) * size + width / 2, sin(o * i) * size + height / 2, r, r + boxSize, r * boxSize / 4 + boxSize));
    }
#elif defined AREA
    int n = N_BOX;
    float boxSize = 8;
    
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xDistribution(-350.0,350.0);
    std::uniform_real_distribution<double> yDistribution(-250.0,250.0);
    std::uniform_real_distribution<double> boxScaleDist(0.0f,2.0f);
    
    for(int i = 0; i < n; i++) {
        float boxSizeFactor = boxScaleDist(generator);
        float xVal = xDistribution(generator) + width/2.0f;
        float yVal = yDistribution(generator) + height/2.0f;
        float angle = yVal/(xVal+1.0f);
        obbs.push_back(OBB(xVal, yVal, angle+M_PI*i/4, boxSizeFactor*8, boxSize-boxSizeFactor*2));
    }
#else
    int n = 1;
    float o = (2 * M_PI) / n;
    float size = 50;
    float boxSize = 15;

    for (int i = 0; i < n; ++i) {
        obbs.push_back(OBB(width/2, height/2, 20, boxSize*8, boxSize*4));
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

void drawAABB(const isect2d::AABB& _aabb) {
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMin().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMin().y);
    line(_aabb.getMax().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMax().y, _aabb.getMax().x, _aabb.getMax().y);
}

void drawOBB(const OBB& obb, bool isect) {
    const isect2d::Vec2* quad = obb.getQuad();

    for(int i = 0; i < 4; ++i) {
        if(isect) {
            glColor4f(0.5, 1.0, 0.5, 1.0);
        } else {
            glColor4f(1.0, 0.5, 0.5, 1.0);
        }

        isect2d::Vec2 start = quad[i];
        isect2d::Vec2 end = quad[(i + 1) % 4];
        
        line(start.x, start.y, end.x, end.y);
        
        glColor4f(1.0, 1.0, 1.0, 0.1);
        cross(obb.getCentroid().x, obb.getCentroid().y, 2);
    }
}

void render() {

    isect2d::SAP sap;
    
    while (!glfwWindowShouldClose(window)) {

        update();
        
        if (pause) {
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
        
        // broad phase
#ifdef SWEEPPRUNE
        std::vector<isect2d::AABB> aabbs;
        std::unordered_map<void*, isect2d::AABB> SAPaabbs;
        std::set<std::pair<void*, void*>> pairs;

        {
            const clock_t beginBroadPhaseTime = clock();

            for (auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                SAPaabbs[aabb.m_userData] = aabb;
            }
            
            sap.intersect(SAPaabbs);

            std::cout << "\tSAP broadphase: " << "Pairs: " << sap.getPairs().size() << " "<< (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000 << "ms ";
            
#ifdef DEBUG_DRAW
            for (int dim = 0; dim < Dimension::MAX_DIM; ++dim) {
                for (auto& ep : sap.m_sortList[dim]) {
                    if (ep->m_isMin) {
                        glColor4f(1.0, 0.5, 0.5, 1.0);
                    } else {
                        glColor4f(0.5, 1.0, 0.5, 1.0);
                    }
                    switch (dim) {
                        case Dimension::X:
                            line(ep->m_value, 0, ep->m_value, 10);
                            break;
                        case Dimension::Y:
                            line(0, ep->m_value, 10, ep->m_value);
                    }
                }
            }
#endif
        }
#endif

#ifdef BRUTEFORCE
        {
            const clock_t beginBroadPhaseTime = clock();

            for(auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                aabbs.push_back(aabb);
            }
            auto pairsBruteforce = intersect(aabbs);

            std::cout << " bruteforce broadphase: " << (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000 << "ms . Pairs: " << pairsBruteforce.size();
        }
#endif

        // narrow phase
        {
            clock_t narrowTime = 0;
            for (auto pair : sap.getPairs()) {
                clock_t beginNarrowTime;

                auto obb1 = *(OBB*)(pair.first);
                auto obb2 = *(OBB*)(pair.second);

                // narrow phase
                beginNarrowTime = clock();
                bool isect = intersect(obb1, obb2);
                narrowTime += (clock() - beginNarrowTime);

                if (isect) {
                    drawOBB(obb1, true);
                    drawOBB(obb2, true);

                    line(obb1.getCentroid().x, obb1.getCentroid().y, obb2.getCentroid().x, obb2.getCentroid().y);
                }
            }


            std::cout << "narrowphase: " << (float(narrowTime) / CLOCKS_PER_SEC) * 1000 << "ms" << std::endl;
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

