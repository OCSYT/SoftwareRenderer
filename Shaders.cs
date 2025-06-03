using Silk.NET.Maths;
using System.Numerics;

namespace SoftwareRenderer
{
    public static class Shaders
    {
        
        public struct VertexInput(Vector3 pos, Vector2 texCoord, Vector3 normal, Vector4 color)
        {
            public Vector3 Position = pos;
            public Vector4 Color = color;
            public Vector2 TexCoord = texCoord;
            public Vector3 Normal = normal;
        }
    
        public struct VertexOutput
        {
            public Vector4 ClipPosition;
            public Vector4 Color;
            public Vector2 TexCoord;
            public Vector3 Normal;
            public Vector2 ScreenCoords;
            public Dictionary<string, object> Data;
            public bool Interpolate = true;

            public VertexOutput()
            {
                ClipPosition = default;
                Color = default;
                TexCoord = default;
                Normal = default;
                ScreenCoords = default;
                Data = new Dictionary<string, object>();
            }
            public VertexOutput Clone()
            {
                return new VertexOutput
                {
                    ClipPosition = this.ClipPosition,
                    Color = this.Color,
                    TexCoord = this.TexCoord,
                    Normal = this.Normal,
                    Data = this.Data != null ? new Dictionary<string, object>(this.Data) : null,
                    ScreenCoords = this.ScreenCoords
                };
            }
        }
        
        public static Shaders.VertexOutput Lerp(Shaders.VertexOutput a, Shaders.VertexOutput b, float t, bool interpolate)
        {
            Vector4 clipPos = Vector4.Lerp(a.ClipPosition, b.ClipPosition, t);
            Vector4 color = interpolate ? Vector4.Lerp(a.Color, b.Color, t) : a.Color;
            Vector2 texCoord = Vector2.Lerp(a.TexCoord, b.TexCoord, t);
            Vector3 normal = interpolate ? Vector3.Normalize(Vector3.Lerp(a.Normal, b.Normal, t)) : a.Normal;

            Dictionary<string, object>? data = a.Data;
            return new Shaders.VertexOutput
            {
                ClipPosition = clipPos,
                Color = color,
                TexCoord = texCoord,
                Normal = normal,
                Data = data,
                ScreenCoords = Vector2.Zero
            };
        }

    
        public delegate Shaders.VertexOutput VertexShader(VertexInput input, Matrix4x4 model, Matrix4x4 view, Matrix4x4 projection);
        public delegate Vector4? FragmentShader(VertexOutput input);
    }
}