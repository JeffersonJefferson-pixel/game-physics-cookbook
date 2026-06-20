#if _DEBUG
    #define _CRTDBG_MAP_ALLOC
    #include <stdlib.h>
    #include <crtdbg.h>

    #include <float.h>
    unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);
#endif

#define MAIN_WIN_32_WINDOW_CLASS L"Win32GLSandbox"
#define ENABLE_RESIZE

#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#include <windows.h>
#include <windowsx.h>

#include <glad/glad.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <conio.h>

#include "IWindow.h"
#include "matrices.h"
#include "imgui/imgui.h"
#include "imgui/imgui_implementation.h"
#include "imgui/ImGuizmo.h"

bool CheckIfAlreadyRunning(IWindow* pWindowInstance);

int main(int argc, const char** argv) {
    IWindow* pWindowInstance = IWindow::GetInstance();

    if (CheckIfAlreadyRunning(pWindowInstance)) {
        std::cout << "Something went wrong\n";
        getch();
    }
    else if (CheckIfAlreadyRunning(pWindowInstance)) {
        std::cout << "Error, only on instance\n";
        getch();
    }
    else {
        WinMain(GetModuleHandle(NULL), NULL, GetCommandLineA(), SW_SHOWDEFAULT);
    }
    return 0;
}

HWND MakeNewWindow(IWindow* pWindowInstance, HINSTANCE hInstance, LPSTR szCmdLine) {
    std::wstring MAIN_WIN32_WINDOW_NAME;
    pWindowInstance->GetAndResetTitleDirtyFlag();
    std::string title(pWindowInstance->GetTitle());
    MAIN_WIN32_WINDOW_NAME = std::wstring(title.begin(), title.end());

    return CreateWindowEx(0, MAIN_WIN_32_WINDOW_CLASS, MAIN_WIN32_WINDOW_NAME.c_str(),
#ifdef ENABLE_RESIZE
        NORMAL_STYLE
#else
        NO_RESIZE_STYLE
#endif
        , windowRect.left, windowRect.top, 
        windowRect.right - windowRect.left, windowRect.bottom - windowRect.top,
        NULL, NULL, hInstance, szCmdLine);
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR szCmdLine, int iCmdShow) {
    IWindow* pWindowInstance = IWindow::GetInstance();

    if (CheckIfAlreadyRunning(pWindowInstance)) {
        MessageBox(NULL, L"Only one instance", NULL, NULL);
        return FALSE;
    }

    int width = pWindowInstance->GetWidth();
    int height = pWindowInstance->GetHeight();

    HINSTANCE hinstance = hInstance;

    WINDCLASSEX wndclass;
    ZeroMemory(&wndclass, sizeof(WNDCLASSEX));
    wndclass.cbSize = sizeof(WNDCLASSEX);
    wndclass.style = CS_HREDRAW | CS_VREDRAW;
    wndclass.lpfnWndProc = WndProc;
    wndclass.cbClsExtra = 0;
    wndclass.cbWndExtra = 0;
    wndclass.hInstance = hInstance;
    wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wndclass.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
    wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndclass.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wndclass.lpszMenuName = 0;
    wndclass.lpszClassName = MAIN_WIN_32_WINDOW_CLASS;
    RegisterClassEx(&wndclass);

    SetRect(&windowRect, (GetSystemMetrics(SM_CXSCREEN) / 2) - (width / 2), 
        (GetSystemMetrics(SM_CYSCREEN) / 2) - (height / 2), 
        (GetSystemMetrics(SM_CXSCREEN) / 2) + (width / 2), 
        (GetSystemMetrics(SM_CYSCREEN) / 2) + (height / 2));

    AdjustWindowRect(&windowRect, 
    #ifdef ENABLE_RESIZE
        NORMAL_STYLE
    #else
        NO_RESIZE_STYLE
    #endif
        , FALSE, 0);
    
    HWND hwnd = MakeNewWindow(pWindowInstance, hInstance, szCmdLine);

    HDC hdc = GetDC(hwnd);
    HGLRC hglrc = OpenGLBindContext(hdc);

    if (!gladLoadGL()) {
        std::cout << "Could not instantiate GLAD OpenGL 2.1 context\n";
        exit(-1);
    }
    else if (GLVersion.major < 2) {
        std::cout << "Your system  doesn't support OpenGL >= 2!\n";
        return -1;
    }

    std::cout << "OpenGL Context: " << GLVersion.major << ", " << GLVersion.minor << "\n";
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << "\n";
    std::cout << "GLSL Version: " < glGetString(GL_SHADING_LANGUAGE_VERSION) << "\n";

    ImGui_Implemntation_Init(hwnd);
    pWindowInstance->OnInitialize();

    bool fullscreen = pWindowInstance->GetFullScreen();
    if (fullscreen) {
        UpdateFullscreen(pWindowInstance, hwnd, hdc);
    } else {
        pWindowInstance->OnResize(width, height);
    }

    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);
    GetClientRect(hwnd, &clientRect);
    GetWindowRect(hwnd, &windowRect);
    GetWindowRect(hwnd, &borderRect);

    MSG msg;
    DWORD next_game_tick = GetTickCount();
    int sleep_Time = 0;
    double lastTime = GetMilliseconds();
    double fixed_millis = pWindowInstance->GetFixedFPS() / 1000.0;
    double fixed_ellapsed = 0.0;

    pWindowInstance->MarkAsShown();

    while (!pWindowInstance->GetQuitFlag()) {
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                break;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        ImGui_Implementation_NewFrame();
        ImGuizmo::BeginFrame();

        // change screen title
        if (pWindowInstance->GetAndResetTitleDirtyFlag()) {
            std::wstring MAIN_WIN32_WINDOW-NAME;
            std::string title(pWindowInstance->GetTitle());
            MAIN_WIN32_WINDOW_NAME = std::wstring(title.begin(), title.end());
            SetWindowText(hwnd, MAIN_WIN32_WINDOW_NAME.c_str());
        }

        // toggle full screen
        if (fullscreen != pWindowInstance->GetFullScreen()) {
            UpdateFullscreen(pWindowInstance, hwnd, hdc);
            fullscreen = pWindowInstance->GetFullScreen();
        }

        if (!fullscreen) {
            int windowWidth = borderRect.right - borderRect.left;
            int windowHeight = borderRect.bottom - borderRect.top;
            int clientWidth = clientRect.right - clientRect.left;
            int clientHeight  = clientRect.bottom - clientRect.top;
            int borderWidth = windowWidth - clientWidth;
            int borderHeight = windowHeight - clientHeight;

            if (clientWidth != pWindowInstance->GetWidth() || clientHeight != pWindowInstance->GetHeight()) {
                SetWindowPos(hwnd, 0, windowRect.left, windowRect.top, pWindowInstance->GetWidth() + borderWidth, pWindowInstance->GetHeight() + borderHeight, SWP_NOZORDER);
            }
        }

        double time = GetMilliseconds();
        float deltaTime = float(time - lastTime) * 0.001f;
        lastTime = time;

        pWindowInstance->OnUpdate(deltaTime);

        fixed_ellapsed += deltaTime;
        while (fixed_ellapsed > fixed_millis) {
            pWindowInstance->OnFixedUpdate(fixed_millis);
            fixed_ellapsed -= fixed_millis;
        }

        pWindowInstance->OnRender();
        ImGui::Render();
        SwapBuffers(hdc);

        int SKIP_TICKS = 1000 / pWindowInstance->GetTargetFPS();
        next_game_tick += SKIP_TICKS;
        sleep_time = next_game_tick - GetTickCount();
        if (sleep_time >= 0) {
            Sleep(sleep_time);
        }
    }

    pWindowInstance->OnShutdown();
    ImGui_Implementatin_Shutdown();
    OpenGLUnbindContext(hwnd, hdc, hdlrc);

    CleanupMemory(pWindowInstance);

#if _DEBUG
    _CrtDumpMemoryLeaks();
#endif

    return (int)msg.wParam;
}