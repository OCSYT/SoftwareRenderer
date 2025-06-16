using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace SoftwareRenderer
{
    public readonly struct BoundingSphere
    {
        public readonly Vector3 Center;
        public readonly float Radius;

        public BoundingSphere(Vector3 center, float radius)
        {
            Center = center;
            Radius = radius;
        }
    }

    public readonly struct Plane
    {
        public readonly Vector3 Normal;
        public readonly float Distance;

        public Plane(Vector3 normal, float distance)
        {
            Normal = Vector3.Normalize(normal);
            Distance = distance;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetDistanceToPoint(Vector3 point)
        {
            return Vector3.Dot(Normal, point) + Distance;
        }
    }

    public readonly struct Frustum
    {
        public readonly Plane Near;
        public readonly Plane Far;
        public readonly Plane Left;
        public readonly Plane Right;
        public readonly Plane Top;
        public readonly Plane Bottom;

        public Frustum(Plane near, Plane far, Plane left, Plane right, Plane top, Plane bottom)
        {
            Near = near;
            Far = far;
            Left = left;
            Right = right;
            Top = top;
            Bottom = bottom;
        }
    }

    public static class FrustumCuller
    {
        public static BoundingSphere CalculateBoundingSphere(Shaders.VertexInput[] vertices)
        {
            if (vertices == null)
                throw new ArgumentNullException(nameof(vertices));

            int vertexCount = vertices.Length;
            if (vertexCount == 0)
                return new BoundingSphere(Vector3.Zero, 0f);
            if (vertexCount == 1)
                return new BoundingSphere(vertices[0].Position, 0f);
            
            Vector3 p0 = vertices[0].Position;
            Vector3 p1 = p0;
            float maxDistanceSq = 0f;
            object lockObj = new object();

            Parallel.For(1, vertexCount, () => (Vector3.Zero, 0f),
                (i, state, local) =>
                {
                    float distSq = Vector3.DistanceSquared(vertices[i].Position, p0);
                    if (distSq > local.Item2)
                        return (vertices[i].Position, distSq);
                    return local;
                },
                local =>
                {
                    lock (lockObj)
                    {
                        if (local.Item2 > maxDistanceSq)
                        {
                            maxDistanceSq = local.Item2;
                            p1 = local.Item1;
                        }
                    }
                });

            Vector3 p2 = p1;
            maxDistanceSq = 0f;

            Parallel.For(0, vertexCount, () => (Vector3.Zero, 0f),
                (i, state, local) =>
                {
                    float distSq = Vector3.DistanceSquared(vertices[i].Position, p1);
                    if (distSq > local.Item2)
                        return (vertices[i].Position, distSq);
                    return local;
                },
                local =>
                {
                    lock (lockObj)
                    {
                        if (local.Item2 > maxDistanceSq)
                        {
                            maxDistanceSq = local.Item2;
                            p2 = local.Item1;
                        }
                    }
                });
            
            Vector3 center = (p1 + p2) * 0.5f;
            float radius = MathF.Sqrt(maxDistanceSq) * 0.5f;
            
            Vector3 newCenter = center;
            float newRadius = radius;

            Parallel.For(0, vertexCount, () => (Vector3.Zero, 0f, false),
                (i, state, local) =>
                {
                    Vector3 pos = vertices[i].Position;
                    float distance = Vector3.Distance(pos, center);
                    if (distance > radius)
                        return (pos, distance, true);
                    return local;
                },
                local =>
                {
                    if (local.Item3)
                    {
                        lock (lockObj)
                        {
                            float distance = local.Item2;
                            if (distance > newRadius)
                            {
                                float updatedRadius = (newRadius + distance) * 0.5f;
                                newCenter += (local.Item1 - newCenter) * ((updatedRadius - newRadius) / distance);
                                newRadius = updatedRadius;
                            }
                        }
                    }
                });

            return new BoundingSphere(newCenter, newRadius);
        }
        
        public static Frustum CreateFrustumFromMatrix(Matrix4x4 viewProjection)
        {
            // Extract and normalize frustum planes
            return new Frustum(
                near: NormalizePlane(new Vector4(
                    viewProjection.M14 + viewProjection.M13,
                    viewProjection.M24 + viewProjection.M23,
                    viewProjection.M34 + viewProjection.M33,
                    viewProjection.M44 + viewProjection.M43)),
                far: NormalizePlane(new Vector4(
                    viewProjection.M14 - viewProjection.M13,
                    viewProjection.M24 - viewProjection.M23,
                    viewProjection.M34 - viewProjection.M33,
                    viewProjection.M44 - viewProjection.M43)),
                left: NormalizePlane(new Vector4(
                    viewProjection.M14 + viewProjection.M11,
                    viewProjection.M24 + viewProjection.M21,
                    viewProjection.M34 + viewProjection.M31,
                    viewProjection.M44 + viewProjection.M41)),
                right: NormalizePlane(new Vector4(
                    viewProjection.M14 - viewProjection.M11,
                    viewProjection.M24 - viewProjection.M21,
                    viewProjection.M34 - viewProjection.M31,
                    viewProjection.M44 - viewProjection.M41)),
                top: NormalizePlane(new Vector4(
                    viewProjection.M14 + viewProjection.M12,
                    viewProjection.M24 + viewProjection.M22,
                    viewProjection.M34 + viewProjection.M32,
                    viewProjection.M44 + viewProjection.M42)),
                bottom: NormalizePlane(new Vector4(
                    viewProjection.M14 - viewProjection.M12,
                    viewProjection.M24 - viewProjection.M22,
                    viewProjection.M34 - viewProjection.M32,
                    viewProjection.M44 - viewProjection.M42)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Plane NormalizePlane(Vector4 coefficients)
        {
            float magnitude = MathF.Sqrt(
                coefficients.X * coefficients.X +
                coefficients.Y * coefficients.Y +
                coefficients.Z * coefficients.Z);
            return new Plane(
                new Vector3(coefficients.X, coefficients.Y, coefficients.Z) / magnitude,
                coefficients.W / magnitude);
        }
        
        public static bool IsSphereInFrustum(BoundingSphere bounds, Matrix4x4 modelMatrix, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            Vector3 worldCenter = Vector3.Transform(bounds.Center, modelMatrix);
            float maxScale = MathF.Max(
                MathF.Max(
                    MathF.Sqrt(modelMatrix.M11 * modelMatrix.M11 + modelMatrix.M12 * modelMatrix.M12 + modelMatrix.M13 * modelMatrix.M13),
                    MathF.Sqrt(modelMatrix.M21 * modelMatrix.M21 + modelMatrix.M22 * modelMatrix.M22 + modelMatrix.M23 * modelMatrix.M23)),
                MathF.Sqrt(modelMatrix.M31 * modelMatrix.M31 + modelMatrix.M32 * modelMatrix.M32 + modelMatrix.M33 * modelMatrix.M33));

            float worldRadius = bounds.Radius * maxScale;
            Frustum frustum = CreateFrustumFromMatrix(Matrix4x4.Multiply(viewMatrix, projectionMatrix));
            return TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Left) &&
                   TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Right) &&
                   TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Top) &&
                   TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Bottom) &&
                   TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Near) &&
                   TestSphereAgainstPlane(worldCenter, worldRadius, frustum.Far);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool TestSphereAgainstPlane(Vector3 center, float radius, Plane plane)
        {
            return plane.GetDistanceToPoint(center) > -radius;
        }
    }
}