using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Raycast(
        Vector3 rayOrigin,
        Vector3 rayDirection,
        Shaders.VertexInput[] vertices,
        ushort[] indices,
        Matrix4x4 model,
        out float hitDistance,
        out Vector3 hitPoint,
        out Vector3 hitNormal,
        RaycastFaceMask faceMask = RaycastFaceMask.IgnoreBackfaces)
    {
        if (!Matrix4x4.Invert(model, out var invModel))
        {
            hitDistance = float.MaxValue;
            hitPoint = Vector3.Zero;
            hitNormal = Vector3.Zero;
            return false;
        }

        var normalMatrix = Matrix4x4.Transpose(invModel);
        var vertexCount = vertices.Length;
        Vector3[] transformedVertices = new Vector3[vertexCount];
        Vector3[] transformedNormals = new Vector3[vertexCount];

        for (int i = 0; i < vertexCount; i++)
        {
            ref var vert = ref vertices[i];
            transformedVertices[i] = Vector3.Transform(vert.Position, model);
            var normal = Vector4.Transform(new Vector4(vert.Normal, 0f), normalMatrix);
            transformedNormals[i] = Vector3.Normalize(new Vector3(normal.X, normal.Y, normal.Z));
        }

        return RaycastInternal(rayOrigin, rayDirection, transformedVertices, transformedNormals, indices, out hitDistance, out hitPoint, out hitNormal, faceMask);
    }

    private static bool RaycastInternal(
        Vector3 rayOrigin,
        Vector3 rayDirection,
        Vector3[] vertices,
        Vector3[] normals,
        ushort[] indices,
        out float hitDistance,
        out Vector3 hitPoint,
        out Vector3 hitNormal,
        RaycastFaceMask faceMask)
    {
        hitDistance = float.MaxValue;
        hitPoint = Vector3.Zero;
        hitNormal = Vector3.Zero;

        rayDirection = Vector3.Normalize(rayDirection);
        int triangleCount = indices.Length / 3;

        float closestDistance = float.MaxValue;
        Vector3 closestPoint = Vector3.Zero;
        Vector3 closestNormal = Vector3.Zero;
        bool hitFound = false;
        

        Parallel.For(0, triangleCount,
            () => (Found: false, Distance: float.MaxValue, Point: Vector3.Zero, Normal: Vector3.Zero),
            (i, state, local) =>
            {
                int i0 = indices[i * 3];
                int i1 = indices[i * 3 + 1];
                int i2 = indices[i * 3 + 2];

                var v0 = vertices[i0];
                var v1 = vertices[i1];
                var v2 = vertices[i2];

                if (RayIntersectsTriangle(rayOrigin, rayDirection, v0, v1, v2, out float distance, out Vector3 bary,
                        faceMask))
                {
                    if (distance < 0) return local;

                    Vector3 n0 = normals[i0];
                    Vector3 n1 = normals[i1];
                    Vector3 n2 = normals[i2];

                    Vector3 interpolatedNormal = Vector3.Normalize(n0 * bary.X + n1 * bary.Y + n2 * bary.Z);
                    Vector3 point = rayOrigin + rayDirection * distance;

                    if (distance < local.Distance)
                    {
                        local = (true, distance, point, interpolatedNormal);
                    }
                }

                return local;
            },
            local =>
            {
                if (local.Found)
                {
                    if (local.Distance < closestDistance)
                    {
                        closestDistance = local.Distance;
                        closestPoint = local.Point;
                        closestNormal = local.Normal;
                        hitFound = true;

                    }
                }
            });

        if (hitFound)
        {
            hitDistance = closestDistance;
            hitPoint = closestPoint;
            hitNormal = closestNormal;
            return true;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        bool ignoreBackfaces = faceMask.HasFlag(RaycastFaceMask.IgnoreBackfaces);
        bool ignoreFrontfaces = faceMask.HasFlag(RaycastFaceMask.IgnoreFrontfaces);

        // Robust face culling
        if (ignoreBackfaces && det < Epsilon) return false;
        if (ignoreFrontfaces && det > -Epsilon) return false;
        if (Math.Abs(det) < Epsilon) return false;

        float invDet = 1f / det;
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

