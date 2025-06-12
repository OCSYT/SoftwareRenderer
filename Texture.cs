using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SoftwareRenderer
{
    public class Texture
    {
        public byte[] Data { get; private set; } // RGBA bytes, 4 per pixel
        public int Width { get; private set; }
        public int Height { get; private set; }

        private Texture(byte[] data, int width, int height)
        {
            Data = data;
            Width = width;
            Height = height;
        }


        public Vector4 Sample(Vector2 uv)
        {
            float u = uv.X - (int)uv.X;
            float v = uv.Y - (int)uv.Y;
            u += (u < 0) ? 1f : 0f;
            v += (v < 0) ? 1f : 0f;

            // Convert to pixel coordinates
            int x = (int)(u * Width) % Width;
            int y = (int)(v * Height) % Height;

            if (x < 0) x += Width;
            if (y < 0) y += Height;

            int index = (y * Width + x) << 2;

            uint pixelData = BitConverter.ToUInt32(Data, index);
    
            const float inv255 = 1f / 255f;
            return new Vector4(
                (pixelData & 0xFF) * inv255,
                ((pixelData >> 8) & 0xFF) * inv255,
                ((pixelData >> 16) & 0xFF) * inv255,
                ((pixelData >> 24) & 0xFF) * inv255);
        }
        
        public static Texture LoadTexture(string FilePath)
        {
            var FinalPath = FilePath;
            try
            {
                using Image<Rgba32> image = Image.Load<Rgba32>(FinalPath);
                int width = image.Width;
                int height = image.Height;
                byte[] data = new byte[width * height * 4]; // 4 bytes per pixel (RGBA)

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        Rgba32 pixel = image[x, y];
                        int index = (y * width + x) * 4;
                        data[index] = pixel.R;
                        data[index + 1] = pixel.G;
                        data[index + 2] = pixel.B;
                        data[index + 3] = pixel.A;
                    }
                }
                Console.WriteLine("Loaded Texture: " + FinalPath);
                return new Texture(data, width, height);
            }
            catch
            {
                Console.WriteLine("Failed to load texture: " + FinalPath);
                return null;
            }
        }
    }
}