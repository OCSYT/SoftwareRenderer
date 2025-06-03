namespace SoftwareRenderer
{
    public class PerlinNoise
    {
        private readonly int[] Permutation;

        public PerlinNoise(int seed)
        {
            var rnd = new Random(seed);
            Permutation = new int[512];
            var p = new int[256];
            for (int i = 0; i < 256; i++) p[i] = i;

            for (int i = 0; i < 256; i++)
            {
                int swapIndex = rnd.Next(i, 256);
                (p[i], p[swapIndex]) = (p[swapIndex], p[i]);
            }

            for (int i = 0; i < 512; i++)
                Permutation[i] = p[i & 255];
        }

        public float Noise(float x, float y)
        {
            int xi = (int)MathF.Floor(x) & 255;
            int yi = (int)MathF.Floor(y) & 255;

            float xf = x - MathF.Floor(x);
            float yf = y - MathF.Floor(y);

            float u = Fade(xf);
            float v = Fade(yf);

            int aa = Permutation[Permutation[xi] + yi];
            int ab = Permutation[Permutation[xi] + ((yi + 1) & 255)];
            int ba = Permutation[Permutation[(xi + 1) & 255] + yi];
            int bb = Permutation[Permutation[(xi + 1) & 255] + ((yi + 1) & 255)];

            float x1 = Lerp(Grad(aa, xf, yf), Grad(ba, xf - 1, yf), u);
            float x2 = Lerp(Grad(ab, xf, yf - 1), Grad(bb, xf - 1, yf - 1), u);

            return (Lerp(x1, x2, v) + 1) / 2;
        }

        private static float Fade(float t) => t * t * t * (t * (t * 6 - 15) + 10);

        private static float Lerp(float a, float b, float t) => a + t * (b - a);

        private static float Grad(int hash, float x, float y)
        {
            int h = hash & 7;
            float u = h < 4 ? x : y;
            float v = h < 4 ? y : x;

            float res = ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
            return res;
        }
    }
}