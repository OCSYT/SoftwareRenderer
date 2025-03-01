#include "raymarcher.h"

float Raymarcher::SphereSDF(const Vec3& Position, const Vec3& SphereCenter, float Radius) {
    return (Position - SphereCenter).Length() - Radius;
}

Vec3 Raymarcher::GetNormal(const Vec3& P) {
    float Epsilon = 0.001f;
    return Vec3(
        SphereSDF(P + Vec3(Epsilon, 0, 0), Vec3(0, 0, 2), 0.5f) - SphereSDF(P - Vec3(Epsilon, 0, 0), Vec3(0, 0, 2), 0.5f),
        SphereSDF(P + Vec3(0, Epsilon, 0), Vec3(0, 0, 2), 0.5f) - SphereSDF(P - Vec3(0, Epsilon, 0), Vec3(0, 0, 2), 0.5f),
        SphereSDF(P + Vec3(0, 0, Epsilon), Vec3(0, 0, 2), 0.5f) - SphereSDF(P - Vec3(0, 0, Epsilon), Vec3(0, 0, 2), 0.5f)
    ).Normalize();
}

bool Raymarcher::Raymarch(const Vec3& RayOrigin, const Vec3& RayDirection, Vec3& HitPoint) {
    float MaxDistance = 10.0f;
    int MaxSteps = 64;
    float MinDistance = 0.001f;

    float DistanceTraveled = 0.0f;
    for (int i = 0; i < MaxSteps; i++) {
        Vec3 CurrentPosition = RayOrigin + RayDirection * DistanceTraveled;
        float DistanceToSphere = SphereSDF(CurrentPosition, Vec3(0, 0, 2), 0.5f);

        if (DistanceToSphere < MinDistance) {
            HitPoint = CurrentPosition;
            return true;
        }

        DistanceTraveled += DistanceToSphere;
        if (DistanceTraveled > MaxDistance) break;
    }
    return false;
}

void Raymarcher::Render(Renderer* RendererInstance) {
    float RenderScale = 1.0f/RendererInstance->GetRenderScale();
    int Width = RendererInstance->GetWindowWidth();
    int Height = RendererInstance->GetWindowHeight();
    float AspectRatio = (float)Width / Height;
    float FOV = 1.5f;

    Vec3 LightDirection = Vec3(-0.5f, -0.5f, -1.0f).Normalize(); // Light coming from top-left

    for (int Y = 0; Y < Height; Y += RenderScale) {
        for (int X = 0; X < Width; X += RenderScale) {
            float U = ((X / (float)Width) * 2.0f - 1.0f) * AspectRatio * FOV;
            float V = ((Y / (float)Height) * 2.0f - 1.0f) * FOV;

            Vec3 RayOrigin(0, 0, 0);
            Vec3 RayDirection = Vec3(U, V, 1).Normalize();
            Vec3 HitPoint;

            if (Raymarch(RayOrigin, RayDirection, HitPoint)) {
                Vec3 Normal = GetNormal(HitPoint);
                float Diffuse = std::max(0.0f, Dot(Normal, LightDirection)); // Basic Lambertian shading
                
                int ColorIntensity = static_cast<int>(Diffuse * 255);
                RendererInstance->SetPixel(X, Y, RGB(ColorIntensity, 0, 0));  // Red Sphere with lighting
            }
        }
    }
}
