#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include <windows.h>
#include <chrono>
#include <iostream>

class Renderer {
public:
    Renderer();
    int WinMain(HINSTANCE Instance, HINSTANCE, LPSTR, int ShowCmd);
    void CreateWindowMain(HINSTANCE Instance, int ShowCmd);
    void InitializeBuffer();
    
    void SetRenderScale(float Scale);
    void SetPixel(int X, int Y, COLORREF Color);
    COLORREF GetPixel(int X, int Y);
    void ClearBuffer(COLORREF ClearColor = RGB(0, 0, 0));
    void RenderBuffer();

    static LRESULT CALLBACK WindowProc(HWND WindowHandle, UINT Message, WPARAM WParam, LPARAM LParam);

    [[nodiscard]] HWND GetWindowHandle() const { return WindowHandle; }
    [[nodiscard]] int GetWindowWidth() const { return WindowWidth; }
    [[nodiscard]] int GetWindowHeight() const { return WindowHeight; }
    [[nodiscard]] int GetScaledWidth() const { return ScaledWidth; }
    [[nodiscard]] int GetScaledHeight() const { return ScaledHeight; }
    [[nodiscard]] float GetRenderScale() const { return RenderScale; }

private:
    HWND WindowHandle;
    int WindowWidth, WindowHeight;
    int ScaledWidth, ScaledHeight;
    float RenderScale;
    std::vector<uint32_t> ColorBuffer;
    BITMAPINFO BitmapInfo;
};

#endif
