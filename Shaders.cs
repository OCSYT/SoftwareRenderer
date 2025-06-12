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

            Dictionary<string, object>? data = null;
            if (interpolate)
            {

                if (a.Data != null && b.Data != null)
                {
                    data = new Dictionary<string, object>();

                    foreach (var kvp in a.Data)
                    {
                        if (!b.Data.TryGetValue(kvp.Key, out var bValue))
                            continue;

                        var aValue = kvp.Value;

                        object? result = interpolate switch
                        {
                            true when aValue is float af && bValue is float bf => af * (1 - t) + bf * t,
                            true when aValue is Vector2 av2 && bValue is Vector2 bv2 => Vector2.Lerp(av2, bv2, t),
                            true when aValue is Vector3 av3 && bValue is Vector3 bv3 => Vector3.Lerp(av3, bv3, t),
                            true when aValue is Vector4 av4 && bValue is Vector4 bv4 => Vector4.Lerp(av4, bv4, t),
                            _ => aValue
                        };

                        data[kvp.Key] = result!;
                    }
                }
                else if (a.Data != null)
                {
                    data = new Dictionary<string, object>(a.Data);
                }
            }
            else
            {
                data = a.Data;
            }
            

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
        public static VertexOutput Lerp3(VertexOutput a, VertexOutput b, VertexOutput c, Vector3 t, bool interpolate)
        {
            VertexOutput ab = Lerp(a, b, t.Y / (t.X + t.Y + float.Epsilon), interpolate);
            VertexOutput ac = Lerp(a, c, t.Z / (t.X + t.Z + float.Epsilon), interpolate);
            
            float total = t.Y + t.Z;
            float lerpT = total > 0 ? t.Z / total : 0;
            return Lerp(ab, ac, lerpT, interpolate);
        }


        public delegate VertexOutput VertexShader(VertexInput input, Matrix4x4 model, Matrix4x4 view, Matrix4x4 projection);
        public delegate Vector4? FragmentShader(VertexOutput input);
    }
}