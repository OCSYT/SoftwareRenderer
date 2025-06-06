using System;
using System.Numerics;

namespace SoftwareRenderer
{
    public class Camera
    {
        public Vector3 Position = Vector3.Zero;
        public Quaternion Rotation = Quaternion.Identity;
        public float Sensitivity = 0.1f;

        public Matrix4x4 GetViewMatrix()
        {
            Vector3 forward = GetFront();
            Vector3 up = GetUp();
            return Matrix4x4.CreateLookAt(Position, Position + forward, up);
        }

        public Vector3 GetFront() =>
            Vector3.Transform(-Vector3.UnitZ, Rotation);

        public Vector3 GetRight() =>
            Vector3.Transform(Vector3.UnitX, Rotation);

        public Vector3 GetUp() =>
            Vector3.Transform(Vector3.UnitY, Rotation);

        private static float ToDegrees(float radians)
        {
            return radians * (180f / MathF.PI);
        }

        public Vector3 GetEulerAngles()
        {
            // Extract Euler angles from quaternion
            Vector3 angles = new Vector3();
    
            // Roll (Z)
            float sinr_cosp = 2 * (Rotation.W * Rotation.Z + Rotation.X * Rotation.Y);
            float cosr_cosp = 1 - 2 * (Rotation.Z * Rotation.Z + Rotation.X * Rotation.X);
            angles.Z = MathF.Atan2(sinr_cosp, cosr_cosp);
    
            // Pitch (X)
            float sinp = 2 * (Rotation.W * Rotation.X - Rotation.Y * Rotation.Z);
            if (MathF.Abs(sinp) >= 1)
                angles.X = MathF.CopySign(MathF.PI / 2, sinp);
            else
                angles.X = MathF.Asin(sinp);
    
            // Yaw (Y)
            float siny_cosp = 2 * (Rotation.W * Rotation.Y + Rotation.Z * Rotation.X);
            float cosy_cosp = 1 - 2 * (Rotation.X * Rotation.X + Rotation.Y * Rotation.Y);
            angles.Y = MathF.Atan2(siny_cosp, cosy_cosp);
    
            // Convert to degrees
            angles.X *= (180f / MathF.PI);
            angles.Y *= (180f / MathF.PI);
            angles.Z *= (180f / MathF.PI);
    
            return angles;
        }
    }
}