using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using System.Numerics;
using System.Runtime.CompilerServices;
using Assimp;
using SixLabors.ImageSharp.Processing;

namespace SoftwareRenderer
{
    public enum TextureSlot
    {
        Diffuse = TextureType.Diffuse,
        Specular = TextureType.Specular,
        Ambient = TextureType.Ambient,
        Emissive = TextureType.Emissive,
        Height = TextureType.Height,
        Normals = TextureType.Normals,
        Shininess = TextureType.Shininess,
        Opacity = TextureType.Opacity,
        Displacement = TextureType.Displacement,
        Lightmap = TextureType.Lightmap,
        Reflection = TextureType.Reflection,
        BaseColor = TextureType.BaseColor,
        NormalCamera = TextureType.NormalCamera,
        EmissionColor = TextureType.EmissionColor,
        Metalness = TextureType.Metalness,
        DiffuseRoughness = TextureType.Roughness,
        AmbientOcclusion = TextureType.AmbientOcclusion,
        Unknown = TextureType.Unknown
    }
    public class Texture : IDisposable
    {
        private Image<Rgba32> _image;
        public int Width => _image.Width;
        public int Height => _image.Height;

        public Texture(Image<Rgba32> image)
        {
            _image = image;
        }

        public Vector4 Sample(Vector2 uv)
        {
            float u = uv.X - (int)uv.X;
            float v = uv.Y - (int)uv.Y;
            u += (u < 0) ? 1f : 0f;
            v += (v < 0) ? 1f : 0f;

            int x = (int)(u * Width) % Width;
            int y = (int)(v * Height) % Height;

            if (x < 0) x += Width;
            if (y < 0) y += Height;

            Rgba32 pixel = _image[x, y];
            const float inv255 = 1f / 255f;
            return new Vector4(
                pixel.R * inv255,
                pixel.G * inv255,
                pixel.B * inv255,
                pixel.A * inv255);
        }

        public void Dispose()
        {
            _image?.Dispose();
        }

        public static Texture LoadTexture(string filePath, int maxResolution = 2048)
        {
            try
            {
                var image = Image.Load<Rgba32>(filePath);

                if (image.Width > maxResolution || image.Height > maxResolution)
                {
                    // Calculate new size preserving aspect ratio
                    float scale = Math.Min((float)maxResolution / image.Width, (float)maxResolution / image.Height);
                    int newWidth = (int)(image.Width * scale);
                    int newHeight = (int)(image.Height * scale);

                    image.Mutate(x => x.Resize(newWidth, newHeight));
                }

                Console.WriteLine("Loaded Texture: " + filePath + $" (resized to {image.Width}x{image.Height})");
                return new Texture(image);
            }
            catch
            {
                Console.WriteLine("Failed to load texture: " + filePath);
                return null;
            }
        }
    }
}