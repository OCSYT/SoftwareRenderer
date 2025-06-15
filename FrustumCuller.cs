using System.Numerics;
using System.Runtime.CompilerServices;
using SoftwareRenderer;

namespace SoftwareRenderer
{
     public struct BoundingSphere
    {
        public Vector3 Center;
        public float Radius;

        public BoundingSphere(Vector3 center, float radius)
        {
            Center = center;
            Radius = radius;
        }
    }
    public struct Plane
    {
        public Vector3 Normal;
        public float Distance;

        public Plane(Vector3 normal, float distance)
        {
            Normal = Vector3.Normalize(normal);
            Distance = distance;
        }

        public float DistanceToPoint(Vector3 point)
        {
            return Vector3.Dot(Normal, point) + Distance;
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

            // Find point furthest from p0
            Vector3 p1 = p0;
            float maxDistanceSq = 0f;
            for (int i = 1; i < vertexCount; i++)
            {
                float distSq = Vector3.DistanceSquared(vertices[i].Position, p0);
                if (distSq > maxDistanceSq)
                {
                    maxDistanceSq = distSq;
                    p1 = vertices[i].Position;
                }
            }

            // Find point furthest from p1
            Vector3 p2 = p1;
            maxDistanceSq = 0f;
            for (int i = 0; i < vertexCount; i++)
            {
                float distSq = Vector3.DistanceSquared(vertices[i].Position, p1);
                if (distSq > maxDistanceSq)
                {
                    maxDistanceSq = distSq;
                    p2 = vertices[i].Position;
                }
            }

            // Initial sphere center and radius
            Vector3 center = (p1 + p2) * 0.5f;
            float radius = (float)Math.Sqrt(maxDistanceSq) * 0.5f;

            // Grow sphere to include all points
            for (int i = 0; i < vertexCount; i++)
            {
                Vector3 pos = vertices[i].Position;
                float distance = Vector3.Distance(pos, center);

                if (distance > radius)
                {
                    float newRadius = (radius + distance) * 0.5f;
                    float k = (newRadius - radius) / distance;
                    center += (pos - center) * k;
                    radius = newRadius;
                }
            }

            return new BoundingSphere(center, radius);
        }
        
        public struct Frustum
        {
            public Plane Near;
            public Plane Far;
            public Plane Left;
            public Plane Right;
            public Plane Top;
            public Plane Bottom;
        }

        public static Frustum CreateFrustumFromMatrix(Matrix4x4 viewProjection)
        {
            Frustum frustum = new Frustum();
            
            // Extract rows from the combined view-projection matrix
            var row0 = new Vector4(viewProjection.M14 + viewProjection.M11, viewProjection.M24 + viewProjection.M21, viewProjection.M34 + viewProjection.M31, viewProjection.M44 + viewProjection.M41);
            var row1 = new Vector4(viewProjection.M14 - viewProjection.M11, viewProjection.M24 - viewProjection.M21, viewProjection.M34 - viewProjection.M31, viewProjection.M44 - viewProjection.M41);
            var row2 = new Vector4(viewProjection.M14 - viewProjection.M12, viewProjection.M24 - viewProjection.M22, viewProjection.M34 - viewProjection.M32, viewProjection.M44 - viewProjection.M42);
            var row3 = new Vector4(viewProjection.M14 + viewProjection.M12, viewProjection.M24 + viewProjection.M22, viewProjection.M34 + viewProjection.M32, viewProjection.M44 + viewProjection.M42);
            var row4 = new Vector4(viewProjection.M14 + viewProjection.M13, viewProjection.M24 + viewProjection.M23, viewProjection.M34 + viewProjection.M33, viewProjection.M44 + viewProjection.M43);
            var row5 = new Vector4(viewProjection.M14 - viewProjection.M13, viewProjection.M24 - viewProjection.M23, viewProjection.M34 - viewProjection.M33, viewProjection.M44 - viewProjection.M43);

            // Normalize planes
            frustum.Left = NormalizePlane(row0);
            frustum.Right = NormalizePlane(row1);
            frustum.Bottom = NormalizePlane(row2);
            frustum.Top = NormalizePlane(row3);
            frustum.Near = NormalizePlane(row4);
            frustum.Far = NormalizePlane(row5);

            return frustum;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Plane NormalizePlane(Vector4 planeCoefficients)
        {
            float magnitude = new Vector3(planeCoefficients.X, planeCoefficients.Y, planeCoefficients.Z).Length();
            return new Plane(
                new Vector3(planeCoefficients.X, planeCoefficients.Y, planeCoefficients.Z) / magnitude,
                planeCoefficients.W / magnitude);
        }

        public static bool IsSphereInFrustum(BoundingSphere bounds, Matrix4x4 modelMatrix, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            // Transform sphere center to world space (considering full transformation)
            Vector3 worldCenter = Vector3.Transform(bounds.Center, modelMatrix);
    
            // Calculate transformed radius (using the maximum scale factor)
            // More accurate than using separate axis vectors
            float maxScale = MathF.Max(
                MathF.Max(
                    new Vector3(modelMatrix.M11, modelMatrix.M12, modelMatrix.M13).Length(),
                    new Vector3(modelMatrix.M21, modelMatrix.M22, modelMatrix.M23).Length()),
                new Vector3(modelMatrix.M31, modelMatrix.M32, modelMatrix.M33).Length());
    
            float worldRadius = bounds.Radius * maxScale;

            // Create combined view-projection matrix (correct multiplication order)
            Matrix4x4 viewProjection = Matrix4x4.Multiply(viewMatrix, projectionMatrix);
    
            // Create frustum planes from the combined matrix
            Frustum frustum = CreateFrustumFromMatrix(viewProjection);

            // Test against each frustum plane
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
            float distance = plane.DistanceToPoint(center);
            return distance > -radius;
        }
    }
}