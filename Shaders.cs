using System;
using System.Numerics;
using System.Collections.Generic;

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

        public static VertexOutput ToOutput(VertexInput input)
        {
            return new VertexOutput
            {
                ClipPosition = new Vector4(input.Position, 1),
                Normal = input.Normal,
                TexCoord = input.UV,
                Color = input.Color,
                Data = new Dictionary<string, object>(),
                Interpolate = true,
                ScreenCoords = Vector2.Zero,
                Barycentric = Vector3.Zero
            };
        }

        public static VertexInput ToInput(VertexOutput output)
        {
            return new VertexInput(
                new Vector3(output.ClipPosition.X, output.ClipPosition.Y, output.ClipPosition.Z),
                output.TexCoord,
                output.Normal,
                output.Color
            );
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
                ClipPosition = Vector4.Zero;
                Color = Vector4.Zero;
                TexCoord = Vector2.Zero;
                Normal = Vector3.Zero;
                ScreenCoords = Vector2.Zero;
                Data = new Dictionary<string, object>();
                Interpolate = true;
                Barycentric = Vector3.Zero;
            }
        }

        public static VertexOutput Lerp(VertexOutput a, VertexOutput b, float t, bool interpolate)
        {
            Vector4 clipPosition = Vector4.Lerp(a.ClipPosition, b.ClipPosition, t);
            Vector2 texCoord = Vector2.Lerp(a.TexCoord, b.TexCoord, t);
            Vector4 color = interpolate ? Vector4.Lerp(a.Color, b.Color, t) : a.Color;
            Vector3 normal = interpolate ? Vector3.Lerp(a.Normal, b.Normal, t) : a.Normal;

            Dictionary<string, object> data = new();

            if (interpolate && a.Data != null && b.Data != null)
            {
                foreach (var kvp in a.Data)
                {
                    if (!b.Data.TryGetValue(kvp.Key, out var bValue))
                        continue;

                    var aValue = kvp.Value;
                    object result = aValue switch
                    {
                        float af when bValue is float bf => af * (1 - t) + bf * t,
                        Vector2 av2 when bValue is Vector2 bv2 => Vector2.Lerp(av2, bv2, t),
                        Vector3 av3 when bValue is Vector3 bv3 => Vector3.Lerp(av3, bv3, t),
                        Vector4 av4 when bValue is Vector4 bv4 => Vector4.Lerp(av4, bv4, t),
                        _ => aValue
                    };

                    data[kvp.Key] = result;
                }
            }
            else if (a.Data != null)
            {
                foreach (var kvp in a.Data)
                    data[kvp.Key] = kvp.Value;
            }

            return new VertexOutput
            {
                ClipPosition = clipPosition,
                TexCoord = texCoord,
                Color = color,
                Normal = normal,
                ScreenCoords = Vector2.Zero,
                Data = data,
                Interpolate = interpolate,
                Barycentric = Vector3.Zero
            };
        }

        public static VertexOutput Lerp3(VertexOutput a, VertexOutput b, VertexOutput c, Vector3 barycentric, bool interpolate)
        {
            VertexOutput result = new VertexOutput
            {
                ClipPosition = a.ClipPosition * barycentric.X + b.ClipPosition * barycentric.Y + c.ClipPosition * barycentric.Z,
                TexCoord = a.TexCoord * barycentric.X + b.TexCoord * barycentric.Y + c.TexCoord * barycentric.Z,
                Color = interpolate ? a.Color * barycentric.X + b.Color * barycentric.Y + c.Color * barycentric.Z : a.Color,
                Normal = interpolate ? a.Normal * barycentric.X + b.Normal * barycentric.Y + c.Normal * barycentric.Z : a.Normal,
                ScreenCoords = Vector2.Zero,
                Interpolate = interpolate,
                Barycentric = barycentric,
                Data = new Dictionary<string, object>()
            };

            if (interpolate)
            {
                foreach (var kvp in a.Data)
                {
                    if (!b.Data.TryGetValue(kvp.Key, out var bVal) || !c.Data.TryGetValue(kvp.Key, out var cVal))
                        continue;

                    var aVal = kvp.Value;
                    object resultVal = aVal switch
                    {
                        float af when bVal is float bf && cVal is float cf =>
                            af * barycentric.X + bf * barycentric.Y + cf * barycentric.Z,
                        Vector2 av2 when bVal is Vector2 bv2 && cVal is Vector2 cv2 =>
                            av2 * barycentric.X + bv2 * barycentric.Y + cv2 * barycentric.Z,
                        Vector3 av3 when bVal is Vector3 bv3 && cVal is Vector3 cv3 =>
                            av3 * barycentric.X + bv3 * barycentric.Y + cv3 * barycentric.Z,
                        Vector4 av4 when bVal is Vector4 bv4 && cVal is Vector4 cv4 =>
                            av4 * barycentric.X + bv4 * barycentric.Y + cv4 * barycentric.Z,
                        _ => aVal
                    };

                    result.Data[kvp.Key] = resultVal;
                }
            }

            return result;
        }

        public delegate VertexOutput VertexShader(VertexInput input, Matrix4x4 model, Matrix4x4 view, Matrix4x4 projection);
        public delegate Vector4? FragmentShader(VertexOutput input);
    }

    // Optional helper if you want XYZ() extension
    public static class VectorExtensions
    {
        public static Vector3 XYZ(this Vector4 v) => new Vector3(v.X, v.Y, v.Z);
    }
}