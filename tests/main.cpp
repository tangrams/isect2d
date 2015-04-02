
#include "obb.h"
#include "isect2d.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <stack>

#include <GLFW/glfw3.h>

//#define AXIS_DRAW
#define N_BOX 100
#define CIRCLES

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
        pause = !pause;
    }
}

void initBBoxes() {

#ifdef CIRCLES
    int n = N_BOX;
    float o = (2 * M_PI) / n;
    float size = 250;
    float boxSize = n / (0.5 * n);

    for (int i = 0; i < n; ++i) {
        float r = rand_0_1(20);

        obbs.push_back(OBB(cos(o * i) * size + width / 2,
                    sin(o * i) * size + height / 2, r,
                    r + boxSize * 8, r * boxSize / 3 + boxSize));
    }
#else
    int n = 4;
    float o = (2 * M_PI) / n;
    float size = 50;
    float boxSize = 10;

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

void drawAABB(const isect2d::AABB& _aabb) {
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMin().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMin().y);
    line(_aabb.getMax().x, _aabb.getMin().y, _aabb.getMax().x, _aabb.getMax().y);
    line(_aabb.getMin().x, _aabb.getMax().y, _aabb.getMax().x, _aabb.getMax().y);
}

void drawOBB(const OBB& obb, bool isect) {
    const isect2d::Vec2* quad = obb.getQuad();
    const isect2d::Vec2* axes = obb.getAxes();

    //drawAABB(isect2d::AABB(obb.getExtent()));

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

#ifdef AXIS_DRAW
        // draw separating axes and the projections
        for(int j = 0; j < 2; ++j) {
            isect2d::Vec2 proj = project(quad[i], axes[j]);

            glColor4f(1.0, 1.0, 1.0, 0.1);
            cross(proj.x, proj.y);
        }
#endif
    }

#ifdef AXIS_DRAW
    for(int i = 0; i < 2; ++i) {
        isect2d::Vec2 end = axes[i] * 1000.0;
        isect2d::Vec2 start = end * -1;

        glColor4f(0.4, 0.4, 0.4, 0.1);
        line(start.x, start.y, end.x, end.y);
    }
#endif
}

void render() {
    while (!glfwWindowShouldClose(window)) {
        update();
        glEnable(GL_MULTISAMPLE);

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
        
        

        {
            const clock_t startBruteForce = clock();
            std::set<std::pair<int, int>> p;
            for (int i = 0; i < obbs.size(); ++i) {
                for (int j = i + 1; j < obbs.size(); ++j) {
                    if (intersect(obbs[i], obbs[j])) {
                        p.insert({ i, j });
                    }
                }
            }
            std::cout << "bruteforce: " << float(clock() - startBruteForce) / CLOCKS_PER_SEC * 1000 << "ms ";
        }

        // broad phase
        std::vector<isect2d::AABB> aabbs;
        std::set<std::pair<int, int>> pairs;
        {
            const clock_t beginBroadPhaseTime = clock();

            for (auto& obb : obbs) {
                auto aabb = obb.getExtent();
                aabb.m_userData = (void*)&obb;
                aabbs.push_back(aabb);
            }
            pairs = intersect(&aabbs[0], aabbs.size(), &aabbs[0], aabbs.size());

            std::cout << "broadphase: " << (float(clock() - beginBroadPhaseTime) / CLOCKS_PER_SEC) * 1000 << "ms ";
        }

        
        {
            std::vector<isect2d::AABB*> aabbPointers;
            for (auto& aabb : aabbs) {
                aabbPointers.push_back(&aabb);
            }
            const clock_t bvhBegin = clock();
            isect2d::BVH bvh(aabbPointers);
            std::cout << "bvhtime: " << (float(clock() - bvhBegin) / CLOCKS_PER_SEC) * 1000 << "ms ";
            
            isect2d::BVHNode* node = bvh.getRoot();
            std::stack<isect2d::BVHNode*> todo;
            todo.push(node);
            
            while (todo.size() != 0) {
                node = todo.top();
                todo.pop();
                
                if (node == nullptr)
                    continue;
                if (node->m_proxy)
                    drawAABB(*node->m_proxy);
                if (node->isLeaf())
                    //drawAABB(*node->m_aabb);
                
                todo.push(node->m_leftChild);
                todo.push(node->m_rightChild);
            }
        }



        //isect2d::AABB extent;
        //for (auto& aabb : aabbs) {
        //    extent = isect2d::unionAABB(extent, aabb);
        //}
        //drawAABB(extent);

        //isect2d::Dimension dim = extent.maxExtent();
        //std::cout << dim << std::endl;
        //isect2d::Vec2 split = (extent.m_min[dim] + extent.m_max[dim]) * 0.5f;
        //if(dim == isect2d::X) {
        //    cross(split.length(), 10);
        //} else {
        //    cross(10, split.length());
        //}



        {
            clock_t narrowTime = 0;

            for (auto pair : pairs) {
                clock_t beginNarrowTime;
                clock_t endNarrowTime;

                auto obb1 = obbs[pair.first];
                auto obb2 = obbs[pair.second];

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

