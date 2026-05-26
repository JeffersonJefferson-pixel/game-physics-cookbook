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

