#include "renderer.h"

Renderer::Renderer() : WindowHandle(nullptr), WindowWidth(800), WindowHeight(600), RenderScale(1.0f) {}

void Renderer::CreateWindowMain(HINSTANCE Instance, int ShowCmd) {
    WNDCLASS WindowClass = {};
    WindowClass.lpfnWndProc = WindowProc;
    WindowClass.hInstance = Instance;
    WindowClass.lpszClassName = "RendererWindow";
    WindowClass.cbWndExtra = sizeof(Renderer*);
    RegisterClass(&WindowClass);

    WindowHandle = CreateWindowA("RendererWindow", "Renderer Window", WS_OVERLAPPEDWINDOW,
                                 CW_USEDEFAULT, CW_USEDEFAULT, WindowWidth, WindowHeight,
                                 nullptr, nullptr, Instance, this);

    SetWindowLongPtr(WindowHandle, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    ShowWindow(WindowHandle, ShowCmd);
    InitializeBuffer();

    MSG Message;

    while (GetMessage(&Message, nullptr, 0, 0)) {
        TranslateMessage(&Message);
        DispatchMessage(&Message);
    }
}

void Renderer::SetRenderScale(float Scale) {
    RenderScale = Scale;
    InitializeBuffer();
}

void Renderer::InitializeBuffer() {
    ScaledWidth = static_cast<int>(WindowWidth * RenderScale);
    ScaledHeight = static_cast<int>(WindowHeight * RenderScale);
    ColorBuffer.resize(static_cast<size_t>(ScaledWidth) * ScaledHeight);

    BitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    BitmapInfo.bmiHeader.biWidth = ScaledWidth;
    BitmapInfo.bmiHeader.biHeight = -ScaledHeight;
    BitmapInfo.bmiHeader.biPlanes = 1;
    BitmapInfo.bmiHeader.biBitCount = 32;
    BitmapInfo.bmiHeader.biCompression = BI_RGB;
}

void Renderer::SetPixel(int X, int Y, COLORREF Color) {
    if (ScaledWidth <= 0 || ScaledHeight <= 0) return;

    int ScaledX = static_cast<int>(X * RenderScale);
    int ScaledY = static_cast<int>(Y * RenderScale);

    if (ScaledX >= 0 && ScaledX < ScaledWidth && ScaledY >= 0 && ScaledY < ScaledHeight) {
        ColorBuffer[static_cast<size_t>(ScaledY) * ScaledWidth + ScaledX] =
            (GetBValue(Color)) | (GetGValue(Color) << 8) | (GetRValue(Color) << 16);
    }
}

COLORREF Renderer::GetPixel(int X, int Y) {
    if (ScaledWidth <= 0 || ScaledHeight <= 0) return RGB(0, 0, 0);

    int ScaledX = static_cast<int>(X * RenderScale);
    int ScaledY = static_cast<int>(Y * RenderScale);

    if (ScaledX < 0 || ScaledX >= ScaledWidth || ScaledY < 0 || ScaledY >= ScaledHeight) {
        return RGB(0, 0, 0);
    }

    uint32_t Color = ColorBuffer[static_cast<size_t>(ScaledY) * ScaledWidth + ScaledX];
    return RGB((Color >> 16) & 0xFF, (Color >> 8) & 0xFF, Color & 0xFF);
}

void Renderer::ClearBuffer(COLORREF ClearColor) {
    if (ColorBuffer.empty() || ScaledWidth <= 0 || ScaledHeight <= 0) return;

    uint32_t PackedColor = (GetRValue(ClearColor) << 16) | (GetGValue(ClearColor) << 8) | GetBValue(ClearColor);
    std::fill(ColorBuffer.begin(), ColorBuffer.end(), PackedColor);
}

void Renderer::RenderBuffer() {
    if (!WindowHandle) return;

    HDC DeviceContext = GetDC(WindowHandle);
    if (!DeviceContext) return;

    StretchDIBits(DeviceContext, 0, 0, WindowWidth, WindowHeight, 0, 0, ScaledWidth, ScaledHeight,
                  ColorBuffer.data(), &BitmapInfo, DIB_RGB_COLORS, SRCCOPY);

    ReleaseDC(WindowHandle, DeviceContext);
}

int Renderer::WinMain(HINSTANCE Instance, HINSTANCE, LPSTR, int ShowCmd) {
    CreateWindowMain(Instance, ShowCmd);
    return 0;
}

LRESULT CALLBACK Renderer::WindowProc(HWND WindowHandle, UINT Message, WPARAM WParam, LPARAM LParam) {
    Renderer* RendererInstance = reinterpret_cast<Renderer*>(GetWindowLongPtr(WindowHandle, GWLP_USERDATA));

    switch (Message) {
        case WM_CREATE: {
            auto* CreateStruct = reinterpret_cast<CREATESTRUCT*>(LParam);
            SetWindowLongPtr(WindowHandle, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(CreateStruct->lpCreateParams));
            return 0;
        }
        case WM_SIZE:
            if (RendererInstance) {
                RendererInstance->WindowWidth = LOWORD(LParam);
                RendererInstance->WindowHeight = HIWORD(LParam);
                RendererInstance->InitializeBuffer();
            }
            return 0;
        case WM_PAINT:
            if (RendererInstance) {
                PAINTSTRUCT PaintStruct;
                BeginPaint(WindowHandle, &PaintStruct);
                RendererInstance->RenderBuffer();
                EndPaint(WindowHandle, &PaintStruct);
            }
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(WindowHandle, Message, WParam, LParam);
}
