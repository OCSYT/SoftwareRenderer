using Silk.NET.Maths;
using System.Numerics;
using System.Collections.Generic;

namespace SoftwareRenderer
{
    public static class Shaders
    {
        public struct VertexInput
        {
            public Vector3 Position;
            public Vector2 TexCoord;
            public Vector3 Normal;
            public Vector4 Color;

            public VertexInput(Vector3 pos, Vector2 texCoord, Vector3 normal, Vector4 color)
            {
                Position = pos;
                TexCoord = texCoord;
                Normal = normal;
                Color = color;
            }
        }

        public struct VertexOutput
        {
            public Vector4 ClipPosition;
            public Vector4 Color;
            public Vector2 TexCoord;
            public Vector3 Normal;
            public Vector2 ScreenCoords;
            public Dictionary<string, object> Data;
            public bool Interpolate;
            public Vector3 Barycentric;

            public VertexOutput()
            {
                ClipPosition = default;
                Color = default;
                TexCoord = default;
                Normal = default;
                ScreenCoords = default;
                Data = new Dictionary<string, object>();
                Interpolate = true;
                Barycentric = default;
            }
        }

        public static VertexOutput Lerp(VertexOutput a, VertexOutput b, float t, bool interpolate)
        {
            Vector4 clipPos = Vector4.Lerp(a.ClipPosition, b.ClipPosition, t);
            Vector4 color = interpolate ? Vector4.Lerp(a.Color, b.Color, t) : a.Color;
            Vector2 texCoord = Vector2.Lerp(a.TexCoord, b.TexCoord, t);
            Vector3 normal = interpolate ? Vector3.Normalize(Vector3.Lerp(a.Normal, b.Normal, t)) : a.Normal;

            Dictionary<string, object>? data = a.Data != null ? new Dictionary<string, object>(a.Data) : null;

            return new VertexOutput
            {
                ClipPosition = clipPos,
                Color = color,
                TexCoord = texCoord,
                Normal = normal,
                Data = data,
                ScreenCoords = Vector2.Zero,
                Interpolate = interpolate,
                Barycentric = Vector3.Zero,
            };
        }

        public delegate VertexOutput VertexShader(VertexInput input, Matrix4x4 model, Matrix4x4 view, Matrix4x4 projection);
        public delegate Vector4? FragmentShader(VertexOutput input);
    }
}