using System;
using System.Numerics;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace SoftwareRenderer
{
    public static class Shaders
    {
        public struct VertexInput
        {
            public Vector3 Position;
            public Vector2 UV;
            public Vector3 Normal;
            public Vector4 Color;

            public VertexInput(Vector3 position, Vector2 uv, Vector3 normal, Vector4 color)
            {
                Position = position;
                UV = uv;
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
            public Dictionary<string, object>? Data = new Dictionary<string, object>();
            public bool Interpolate;
            public Vector3 Barycentric;

            public VertexOutput()
            {
                ClipPosition = default;
                Color = default;
                TexCoord = default;
                Normal = default;
                ScreenCoords = default;
                Interpolate = false;
                Barycentric = default;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static VertexOutput Lerp(in VertexOutput a, in VertexOutput b, float t, bool interpolate)
        {
            var clipPosition = Vector4.Lerp(a.ClipPosition, b.ClipPosition, t);
            var texCoord = Vector2.Lerp(a.TexCoord, b.TexCoord, t);
            var color = interpolate ? Vector4.Lerp(a.Color, b.Color, t) : a.Color;
            var normal = interpolate ? Vector3.Lerp(a.Normal, b.Normal, t) : a.Normal;

            Dictionary<string, object>? data = null;

            if (interpolate && a.Data != null && b.Data != null)
            {
                data = new Dictionary<string, object>(a.Data.Count);
                foreach (var kvp in a.Data)
                {
                    if (!b.Data.TryGetValue(kvp.Key, out var bValue))
                        continue;

                    var aValue = kvp.Value;
                    if (aValue is float af && bValue is float bf)
                        data[kvp.Key] = af * (1 - t) + bf * t;
                    else if (aValue is Vector2 av2 && bValue is Vector2 bv2)
                        data[kvp.Key] = Vector2.Lerp(av2, bv2, t);
                    else if (aValue is Vector3 av3 && bValue is Vector3 bv3)
                        data[kvp.Key] = Vector3.Lerp(av3, bv3, t);
                    else if (aValue is Vector4 av4 && bValue is Vector4 bv4)
                        data[kvp.Key] = Vector4.Lerp(av4, bv4, t);
                    else
                        data[kvp.Key] = aValue;
                }
                if (data.Count == 0) data = null;
            }
            else if (!interpolate && a.Data != null)
            {
                data = new Dictionary<string, object>(a.Data);
            }

            return new VertexOutput
            {
                ClipPosition = clipPosition,
                TexCoord = texCoord,
                Color = color,
                Normal = normal,
                Data = data,
                Interpolate = interpolate
            };
        }

        public delegate VertexOutput VertexShader(VertexInput input, Matrix4x4 model, Matrix4x4 view, Matrix4x4 projection);
        public delegate Vector4? FragmentShader(VertexOutput input);
    }
}