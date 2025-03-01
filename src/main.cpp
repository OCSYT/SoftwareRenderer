#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include "rendering/renderer.h"

//shaders
#include "shaders/raymarcher.h"

std::atomic<bool> Running(true);


void RenderThread(Renderer* RendererInstance) {
    RendererInstance->SetRenderScale(.1f);
    Raymarcher RaymarchShader;


    using Clock = std::chrono::high_resolution_clock;
    
    while (Running) {
        auto StartTime = Clock::now();

        RendererInstance->ClearBuffer();
        RaymarchShader.Render(RendererInstance);
        RendererInstance->RenderBuffer();

        auto EndTime = Clock::now();
        std::chrono::duration<float> FrameTime = EndTime - StartTime;
        
        float FPS = 1.0f / FrameTime.count();
        std::cout << "FPS: " << FPS << std::endl;
    }
}

int main()
{
    Renderer RendererInstance;

    std::thread RenderThreadInstance(RenderThread, &RendererInstance);

    RendererInstance.WinMain(GetModuleHandle(nullptr), nullptr, GetCommandLineA(), SW_SHOWDEFAULT);
    Running = false;
    RenderThreadInstance.join();

    return 0;
}
