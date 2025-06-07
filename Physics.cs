using System;
using System.Numerics;

namespace SoftwareRenderer;

[Flags]
public enum RaycastFaceMask
{
    None = 0,
    IgnoreBackfaces = 1 << 0,
    IgnoreFrontfaces = 1 << 1,
}

public static class Physics
{
    public static bool Raycast(
        Vector3 rayOrigin,
        Vector3 rayDirection,
        Shaders.VertexInput[] vertices,
        int[] indices,
        Matrix4x4 model,
        out float hitDistance,
        out Vector3 hitPoint,
        out Vector3 hitNormal,
        RaycastFaceMask faceMask = RaycastFaceMask.IgnoreBackfaces)
    {
        hitDistance = float.MaxValue;
        hitPoint = Vector3.Zero;
        hitNormal = Vector3.Zero;

        rayDirection = Vector3.Normalize(rayDirection);
        bool hit = false;

        for (int i = 0; i < indices.Length; i += 3)
        {
            int i0 = indices[i];
            int i1 = indices[i + 1];
            int i2 = indices[i + 2];

            Vector3 v0 = Vector3.Transform(vertices[i0].Position, model);
            Vector3 v1 = Vector3.Transform(vertices[i1].Position, model);
            Vector3 v2 = Vector3.Transform(vertices[i2].Position, model);

            if (RayIntersectsTriangle(rayOrigin, rayDirection, v0, v1, v2, out float distance, out Vector3 bary,
                    faceMask))
            {
                if (distance < hitDistance)
                {
                    hitDistance = distance;
                    hitPoint = rayOrigin + rayDirection * distance;

                    Vector3 n0 = vertices[i0].Normal;
                    Vector3 n1 = vertices[i1].Normal;
                    Vector3 n2 = vertices[i2].Normal;

                    hitNormal = Vector3.Normalize(n0 * bary.X + n1 * bary.Y + n2 * bary.Z);
                    hit = true;
                }
            }
        }

        return hit;
    }

    private static bool RayIntersectsTriangle(
        Vector3 rayOrigin,
        Vector3 rayDirection,
        Vector3 v0,
        Vector3 v1,
        Vector3 v2,
        out float distance,
        out Vector3 barycentric,
        RaycastFaceMask faceMask)
    {
        const float Epsilon = 1e-8f;
        distance = 0;
        barycentric = Vector3.Zero;

        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 pvec = Vector3.Cross(rayDirection, edge2);
        float det = Vector3.Dot(edge1, pvec);

        // Face culling
        if (faceMask.HasFlag(RaycastFaceMask.IgnoreBackfaces) && det < Epsilon) return false;
        if (faceMask.HasFlag(RaycastFaceMask.IgnoreFrontfaces) && det > -Epsilon) return false;
        if (Math.Abs(det) < Epsilon) return false;

        float invDet = 1.0f / det;
        Vector3 tvec = rayOrigin - v0;

        float u = Vector3.Dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        Vector3 qvec = Vector3.Cross(tvec, edge1);
        float v = Vector3.Dot(rayDirection, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;

        distance = Vector3.Dot(edge2, qvec) * invDet;
        if (distance < 0) return false;

        barycentric = new Vector3(1 - u - v, u, v);
        return true;
    }
}