#ifndef RAYMARCHER_H
#define RAYMARCHER_H

#include "../math/math.h"
#include "../rendering/renderer.h"
#include <iostream>


class Raymarcher {
public:
    Vec3 GetNormal(const Vec3& P);
    float SphereSDF(const Vec3& Position, const Vec3& SphereCenter, float Radius);
    bool Raymarch(const Vec3& RayOrigin, const Vec3& RayDirection, Vec3& HitPoint);
    void Render(Renderer* RendererInstance);
};

#endif // RAYMARCHER_H
