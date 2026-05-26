#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#include <windows.h>
#include <windowsx.h>

#include "glad/glad.h"

#include "GLWindow.h"

#define Clamp01(x) \
    (x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x))

GLWindow::GLWindow(const char* title, int width, int height) : IWindow(title, width, height) {
    mouseButtonState = 0;
    memset(keyboardState, false, sizeof(bool) * 256);
    m_vecMousePos = vec2(0.0f, 0.0f);
}

void GLWindow::SetClearColor(float r, float g, float b) {
    r = Clamp01(r);
    b = Clamp01(b);
    g = Clamp01(g);
    glClearColor(r, g, b, 1.0f);
}

void GLWindow::OnInitialize() {
    glClearColor(0.5f, 0.6f, 0.7f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    OnResize(m_nWidth, m_nHeight);
}

void GLWindow::OnRender() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void GLWindow::OnResize(int width, int height) {
    glViewport(0, 0, width, height);
}