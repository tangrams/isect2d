#include "isect2d.h"

#include <iostream>
#include <cmath>
#include <vector>

#include <GLFW/glfw3.h>

GLFWwindow* window;
float width = 800;
float height = 600;
float dpiRatio = 1;

isect2d::OBB obb1(300.0, 270.0, 1.4, 10.0, 10.0);
isect2d::OBB obb2(300.0, 300.0, 0.0, 40.0, 120.0);

bool pause = false;

std::vector<isect2d::OBB> obbs;

void update() {
    double time = glfwGetTime();
    if(!pause) {
        obbs.clear();

        obb1.move(300.0 + 100.0 * cos(time), 300.0);
        obb1.rotate(time / 10.0);
        obb2.move(300.0, 300.0 + 250.0 * cos(time));
        obb2.rotate(time / 30.0);

        obbs.push_back(obb1);
        obbs.push_back(obb2);
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(key == 'P' && action == GLFW_PRESS) {
        pause = !pause;
    }
}

void init() {
    glfwInit();

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 2);

    window = glfwCreateWindow(width, height, "isect2d", NULL, NULL);

    if (!window) {
        glfwTerminate();
    }

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    glfwSetKeyCallback(window, keyCallback);

    dpiRatio = fbWidth / width;

    glfwMakeContextCurrent(window);
}

void line(float sx, float sy, float ex, float ey) {
    glBegin(GL_LINES);
    glVertex2f(sx, sy);
    glVertex2f(ex, ey);
    glEnd();
}

void cross(float x, float y) {
    line(x - 3, y, x + 3, y);
    line(x, y - 3, x, y + 3);
}

void drawOBB(const isect2d::OBB& obb, const isect2d::OBB& other, bool isect) {
    const isect2d::Vec2* quad = obb.getQuad();
    const isect2d::Vec2* otherQuad = other.getQuad();
    const isect2d::Vec2* axes = obb.getAxes();

    for(int i = 0; i < 4; ++i) {
        if(isect) {
            glColor4f(0.5, 1.0, 0.5, 1.0);
        } else {
            glColor4f(1.0, 0.5, 0.5, 1.0);
        }

        isect2d::Vec2 start = quad[i];
        isect2d::Vec2 end = quad[(i + 1) % 4];
        line(start.x, start.y, end.x, end.y);

        // draw separating axes and the projections
        for(int j = 0; j < 2; ++j) {
            isect2d::Vec2 proj = project(quad[i], axes[j]);
            isect2d::Vec2 projOther = project(otherQuad[i], axes[j]);

            glColor4f(1.0, 1.0, 1.0, 0.5);
            cross(proj.x, proj.y);
            glColor4f(1.0, 0.5, 1.0, 0.5);
            cross(projOther.x, projOther.y);
        }
    }

    for(int i = 0; i < 2; ++i) {
        isect2d::Vec2 end = axes[i] * 1000.0;
        isect2d::Vec2 start = end * -1;

        glColor4f(0.4, 0.4, 0.4, 0.5);
        line(start.x, start.y, end.x, end.y);
    }
}

void render() {
    while (!glfwWindowShouldClose(window)) {
        update();

        glViewport(0, 0, width * dpiRatio, height * dpiRatio);
        glClearColor(0.2f, 0.2f, 0.22f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width, 0, height, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        bool isect = intersect(obb1, obb2);
        drawOBB(obb1, obb2, isect);
        drawOBB(obb2, obb1, isect);

        //auto pairs = intersect(&obbs[0], obbs.size(), &obbs[0], obbs.size());
        //for (auto pair : pairs) {
        //    drawOBB(obbs[pair.first], obbs[pair.second], true);
        //}

        glfwSwapBuffers(window);

        glfwPollEvents();
    }
}

int main() {

    init();
    render();

    return 0;
}


