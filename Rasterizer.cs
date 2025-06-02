using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using Silk.NET.Maths;

namespace SoftwareRenderer
{
    
    public static class Rasterizer
    {
        static ParallelOptions Options = new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount };
        public struct Triangle
        {
            public int Index0;
            public int Index1;
            public int Index2;
            public Vector4 Color;


            public Triangle(int index0, int index1, int index2, Vector4 color)
            {
                Index0 = index0;
                Index1 = index1;
                Index2 = index2;
                Color = color;
            }
        }

        public enum BlendMode { None, Alpha, Additive, Multiply }
        public enum DepthTest { Disabled, Less, LessEqual, Greater, GreaterEqual, Equal, NotEqual, Always }
        public enum CullMode { None, Back, Front }

        private const float Epsilon = 1e-6f;
        private const int TileSize = 32;
        private static object[] TileLocks;
        private static int TilesX, TilesY;

        private static Vector4 Blend(Vector4 src, Vector4 dst, BlendMode mode) => mode switch
        {
            BlendMode.None => src,
            BlendMode.Alpha => src * src.W + dst * (1 - src.W),
            BlendMode.Additive => Vector4.Min(src + dst, Vector4.One),
            BlendMode.Multiply => src * dst,
            _ => src
        };

        public static void InitializeTileLocks(int width, int height)
        {
            TilesX = (width + TileSize - 1) / TileSize;
            TilesY = (height + TileSize - 1) / TileSize;
            TileLocks = new object[TilesX * TilesY];

            for (int y = 0; y < TilesY; y++)
            {
                for (int x = 0; x < TilesX; x++)
                {
                    int index = y * TilesX + x;
                    TileLocks[index] = new object();
                }
            }
        }

        private static IEnumerable<Shaders.VertexOutput[]> ClipTriangleAgainstNearPlane(
            Shaders.VertexOutput v0, Shaders.VertexOutput v1, Shaders.VertexOutput v2)
        {
            const float ClipEpsilon = 1e-4f;
            Shaders.VertexOutput[] output = new Shaders.VertexOutput[5];
            int outputCount = 0;


            static bool IsInside(in Vector4 clipPosition)
            {
                float z = clipPosition.Z;
                float w = clipPosition.W;
                float bound = w * (1f + ClipEpsilon);
                return z >= -bound && z <= bound;
            }


            static Shaders.VertexOutput Intersect(in Shaders.VertexOutput a, in Shaders.VertexOutput b)
            {
                Vector4 p0 = a.ClipPosition;
                Vector4 p1 = b.ClipPosition;
                float d0 = p0.Z + p0.W;
                float d1 = p1.Z + p1.W;
                float denom = d0 - d1;

                float t = denom != 0f ? Math.Clamp(d0 / denom, 0f, 1f) : 0f;
                return Shaders.Lerp(a, b, t);
            }

            // Quick reject if all vertices are outside
            if (!IsInside(v0.ClipPosition) && !IsInside(v1.ClipPosition) && !IsInside(v2.ClipPosition))
                yield break;

            // Clip edges (v0-v1), (v1-v2), (v2-v0)
            void ClipEdge(in Shaders.VertexOutput prev, in Shaders.VertexOutput curr)
            {
                bool prevIn = IsInside(prev.ClipPosition);
                bool currIn = IsInside(curr.ClipPosition);

                if (prevIn)
                {
                    if (currIn)
                    {
                        output[outputCount++] = curr;
                    }
                    else
                    {
                        output[outputCount++] = Intersect(prev, curr);
                    }
                }
                else if (currIn)
                {
                    output[outputCount++] = Intersect(prev, curr);
                    output[outputCount++] = curr;
                }
            }

            ClipEdge(v2, v0);
            ClipEdge(v0, v1);
            ClipEdge(v1, v2);

            if (outputCount < 3) yield break;

            // Triangulate the clipped polygon fan-style
            var vStart = output[0];
            for (int i = 1; i < outputCount - 1; i++)
            {
                yield return new[] { vStart, output[i], output[i + 1] };
            }
        }


        public static void RenderMesh(
            MainWindow window,
            Shaders.VertexInput[] vertices,
            Matrix4x4 model,
            Matrix4x4 view,
            Matrix4x4 projection,
            Shaders.VertexShader vertexShader,
            Shaders.FragmentShader fragmentShader,
            CullMode cullMode = CullMode.Back,
            DepthTest depthTest = DepthTest.LessEqual,
            BlendMode blendMode = BlendMode.Alpha)
        {
            if (TileLocks == null || TilesX != (window.RenderWidth + TileSize - 1) / TileSize || 
                TilesY != (window.RenderHeight + TileSize - 1) / TileSize)
            {
                InitializeTileLocks(window.RenderWidth, window.RenderHeight);
            }

            int triangleCount = vertices.Length / 3;

            Parallel.For(0, triangleCount, Options, i =>
            {
                int baseIdx = i * 3;
                var v0 = vertexShader(vertices[baseIdx], model, view, projection);
                var v1 = vertexShader(vertices[baseIdx + 1], model, view, projection);
                var v2 = vertexShader(vertices[baseIdx + 2], model, view, projection);

                bool allPositiveW = v0.ClipPosition.W > 0 && v1.ClipPosition.W > 0 && v2.ClipPosition.W > 0;

                if (!allPositiveW)
                {
                    foreach (var triangle in ClipTriangleAgainstNearPlane(v0, v1, v2))
                    {
                        DrawTriangle(window, triangle[0], triangle[1], triangle[2], fragmentShader, cullMode, depthTest, blendMode);
                    }
                }
                else
                {
                    DrawTriangle(window, v0, v1, v2, fragmentShader, cullMode, depthTest, blendMode);
                }
            });
        }


        private static void DrawTriangle(
            MainWindow window,
            Shaders.VertexOutput v0,
            Shaders.VertexOutput v1,
            Shaders.VertexOutput v2,
            Shaders.FragmentShader fragmentShader,
            CullMode cullMode,
            DepthTest depthTest,
            BlendMode blendMode)
        {
            if (window.RenderWidth <= 0 || window.RenderHeight <= 0) return;

            float invWidth = 1f / (window.RenderWidth - 1);
            float invHeight = 1f / (window.RenderHeight - 1);

            Vector2D<int>[] screenCoords = new Vector2D<int>[3];
            float[] depths = new float[3];
            Shaders.VertexOutput[] outputs = new Shaders.VertexOutput[] { v2, v1, v0 };

            for (int i = 0; i < 3; i++)
            {
                float invW = 1f / outputs[i].ClipPosition.W;
                Vector3 viewPos = new Vector3(
                    outputs[i].ClipPosition.X * invW,
                    outputs[i].ClipPosition.Y * invW,
                    outputs[i].ClipPosition.Z * invW
                );

                screenCoords[i] = new Vector2D<int>(
                    (int)MathF.Round((viewPos.X * 0.5f + 0.5f) * window.RenderWidth),
                    (int)MathF.Round((-viewPos.Y * 0.5f + 0.5f) * window.RenderHeight)
                );
                
                depths[i] = viewPos.Z;
                outputs[i].ScreenCoords = new Vector2(screenCoords[i].X * invWidth, screenCoords[i].Y * invHeight);
            }

            if (EdgeFunction(screenCoords[0], screenCoords[1], screenCoords[2]) == 0) return;

            RasterizeTriangle(window, screenCoords, depths, outputs, fragmentShader, cullMode, depthTest, blendMode);
        }


        private static void RasterizeTriangle(
            MainWindow window,
            Vector2D<int>[] screen,
            float[] depths,
            Shaders.VertexOutput[] outputs,
            Shaders.FragmentShader fragmentShader,
            CullMode cullMode,
            DepthTest depthTest,
            BlendMode blendMode)
        {
            ref var p0 = ref screen[0];
            ref var p1 = ref screen[1];
            ref var p2 = ref screen[2];

            float area = EdgeFunction(p0, p1, p2);
            if (area == 0) return;

            bool isFrontFace = area < 0;
            if ((cullMode == CullMode.Back && !isFrontFace) ||
                (cullMode == CullMode.Front && isFrontFace))
                return;

            float invArea = 1f / area;

            int minX = Math.Max(Math.Min(Math.Min(p0.X, p1.X), p2.X), 0);
            int maxX = Math.Min(Math.Max(Math.Max(p0.X, p1.X), p2.X), window.RenderWidth - 1);
            int minY = Math.Max(Math.Min(Math.Min(p0.Y, p1.Y), p2.Y), 0);
            int maxY = Math.Min(Math.Max(Math.Max(p0.Y, p1.Y), p2.Y), window.RenderHeight - 1);
            if (minX > maxX || minY > maxY) return;

            int a01 = p0.Y - p1.Y, b01 = p1.X - p0.X;
            int a12 = p1.Y - p2.Y, b12 = p2.X - p1.X;
            int a20 = p2.Y - p0.Y, b20 = p0.X - p2.X;

            var depthFunc = GetDepthTestFunction(depthTest);
            int tileSize = TileSize;
            var tileLocks = TileLocks;

            int tileMinX = minX / tileSize;
            int tileMaxX = maxX / tileSize;
            int tileMinY = minY / tileSize;
            int tileMaxY = maxY / tileSize;

            var vector2D = p1;
            var vector2D1 = p0;
            var p3 = p2;

            Parallel.For(tileMinY, tileMaxY + 1, Options, tileY =>
            {
                for (int tileX = tileMinX; tileX <= tileMaxX; tileX++)
                {
                    int tileStartX = tileX * tileSize;
                    int tileEndX = Math.Min(tileStartX + tileSize - 1, window.RenderWidth - 1);
                    int tileStartY = tileY * tileSize;
                    int tileEndY = Math.Min(tileStartY + tileSize - 1, window.RenderHeight - 1);

                    int startX = Math.Max(minX, tileStartX);
                    int endX = Math.Min(maxX, tileEndX);
                    int startY = Math.Max(minY, tileStartY);
                    int endY = Math.Min(maxY, tileEndY);

                    if (startX > endX || startY > endY) continue;

                    int w0Row = a12 * (startX - vector2D.X) + b12 * (startY - vector2D.Y);
                    int w1Row = a20 * (startX - p3.X) + b20 * (startY - p3.Y);
                    int w2Row = a01 * (startX - vector2D1.X) + b01 * (startY - vector2D1.Y);

                    // Lock only once per tile to avoid overhead
                    lock (tileLocks[tileY * TilesX + tileX])
                    {
                        for (int y = startY; y <= endY; y++)
                        {
                            int w0 = w0Row;
                            int w1 = w1Row;
                            int w2 = w2Row;

                            for (int x = startX; x <= endX; x++)
                            {
                                // Edge function test
                                if (w0 >= 0 && w1 >= 0 && w2 >= 0)
                                {
                                    float w0f = w0 * invArea;
                                    float w1f = w1 * invArea;
                                    float w2f = w2 * invArea;

                                    float depth = depths[0] * w0f + depths[1] * w1f + depths[2] * w2f;

                                    float oldDepth = window.GetDepth(x, y);
                                    if (depthFunc(depth, oldDepth))
                                    {
                                        var interpolated = Interpolate(outputs[0], outputs[1], outputs[2], w0f, w1f,
                                            w2f);
                                        var finalColor = fragmentShader(interpolated);

                                        if (finalColor.HasValue && finalColor.Value.W != 0)
                                        {
                                            var dst = window.GetPixel(x, y);
                                            var blended = Blend(finalColor.Value, dst, blendMode);
                                            window.SetPixel(x, y, blended);

                                            if (depthTest != DepthTest.Disabled)
                                                window.SetDepth(x, y, depth);
                                        }
                                    }
                                }

                                w0 += a12;
                                w1 += a20;
                                w2 += a01;
                            }

                            w0Row += b12;
                            w1Row += b20;
                            w2Row += b01;
                        }
                    }
                }
            });
        }





        private static Func<float, float, bool> GetDepthTestFunction(DepthTest test)
        {
            return test switch
            {
                DepthTest.Disabled => (newDepth, oldDepth) => true,
                DepthTest.Less => (newDepth, oldDepth) => newDepth > oldDepth,
                DepthTest.LessEqual => (newDepth, oldDepth) => newDepth >= oldDepth,
                DepthTest.Greater => (newDepth, oldDepth) => newDepth < oldDepth,
                DepthTest.GreaterEqual => (newDepth, oldDepth) => newDepth <= oldDepth,
                DepthTest.Equal => (newDepth, oldDepth) => Math.Abs(newDepth - oldDepth) < Epsilon,
                DepthTest.NotEqual => (newDepth, oldDepth) => Math.Abs(newDepth - oldDepth) >= Epsilon,
                DepthTest.Always => (newDepth, oldDepth) => true,
                _ => (newDepth, oldDepth) => true
            };
        }


        private static float EdgeFunction(Vector2D<int> a, Vector2D<int> b, Vector2D<int> c) =>
            (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);


        public static Shaders.VertexOutput Interpolate(
            Shaders.VertexOutput a,
            Shaders.VertexOutput b,
            Shaders.VertexOutput c,
            float w0,
            float w1,
            float w2)
        {
            float invW0 = 1f / a.ClipPosition.W;
            float invW1 = 1f / b.ClipPosition.W;
            float invW2 = 1f / c.ClipPosition.W;

            float w0invW0 = w0 * invW0;
            float w1invW1 = w1 * invW1;
            float w2invW2 = w2 * invW2;
            
            float oneOverW = w0invW0 + w1invW1 + w2invW2;
            float w = 1f / oneOverW;

            var result = new Shaders.VertexOutput
            {
                ClipPosition = (a.ClipPosition * w0invW0 + b.ClipPosition * w1invW1 + c.ClipPosition * w2invW2) * w,
                Color = (a.Color * w0invW0 + b.Color * w1invW1 + c.Color * w2invW2) * w,
                TexCoord = (a.TexCoord * w0invW0 + b.TexCoord * w1invW1 + c.TexCoord * w2invW2) * w,
                Normal = Vector3.Normalize((a.Normal * w0invW0 + b.Normal * w1invW1 + c.Normal * w2invW2) * w),
                ScreenCoords = (a.ScreenCoords * w0invW0 + b.ScreenCoords * w1invW1 + c.ScreenCoords * w2invW2) * w,
                Data = InterpolateData(a.Data, b.Data, c.Data, w0invW0, w1invW1, w2invW2, w)
            };

            return result;
        }


        private static Dictionary<string, object> InterpolateData(
            Dictionary<string, object> aData,
            Dictionary<string, object> bData,
            Dictionary<string, object> cData,
            float w0invW0,
            float w1invW1,
            float w2invW2,
            float w)
        {
            if (aData == null || bData == null || cData == null) 
                return null;
            
            float w0 = w0invW0 * w;
            float w1 = w1invW1 * w;
            float w2 = w2invW2 * w;
            
            var result = new Dictionary<string, object>(aData.Count);
            foreach (string key in aData.Keys)
            {
                if (!bData.TryGetValue(key, out object bValue) || !cData.TryGetValue(key, out object cValue))
                    continue;

                object aValue = aData[key];
                
                if (aValue is float fa && bValue is float fb && cValue is float fc)
                {
                    result[key] = fa * w0 + fb * w1 + fc * w2;
                }
                else if (aValue is Vector2 va2 && bValue is Vector2 vb2 && cValue is Vector2 vc2)
                {
                    result[key] = va2 * w0 + vb2 * w1 + vc2 * w2;
                }
                else if (aValue is Vector3 va3 && bValue is Vector3 vb3 && cValue is Vector3 vc3)
                {
                    Vector3 interpolated = va3 * w0 + vb3 * w1 + vc3 * w2;
                    result[key] = interpolated != Vector3.Zero ? Vector3.Normalize(interpolated) : interpolated;
                }
                else if (aValue is Vector4 va4 && bValue is Vector4 vb4 && cValue is Vector4 vc4)
                {
                    result[key] = va4 * w0 + vb4 * w1 + vc4 * w2;
                }
                else
                {
                    result[key] = aValue;
                }
            }
    
            return result;
        }
    }
}