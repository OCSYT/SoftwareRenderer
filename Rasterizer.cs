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
        public enum DebugMode
        {
            None,
            Wireframe
        }

        public static DebugMode RenderDebugMode = DebugMode.None;
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

        public enum BlendMode
        {
            None,
            Alpha,
            Additive,
            Multiply
        }

        public enum DepthTest
        {
            Disabled,
            Less,
            LessEqual,
            Greater,
            GreaterEqual,
            Equal,
            NotEqual,
            Always
        }

        public enum CullMode
        {
            None,
            Back,
            Front
        }

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

        private static readonly object TileLocksInitLock = new object();

        public static void InitializeTileLocks(int width, int height)
        {
            if (width <= 0 || height <= 0)
            {
                throw new ArgumentException("Width and height must be positive non-zero values.");
            }

            int tilesX = (width + TileSize - 1) / TileSize;
            int tilesY = (height + TileSize - 1) / TileSize;

            lock (TileLocksInitLock)
            {
                if (tilesX == TilesX && tilesY == TilesY && TileLocks != null && TileLocks.Length == tilesX * tilesY)
                    return;

                TilesX = tilesX;
                TilesY = tilesY;
                TileLocks = new object[TilesX * TilesY];

                for (int i = 0; i < TileLocks.Length; i++)
                {
                    TileLocks[i] = new object();
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
                return Shaders.Lerp(a, b, t, a.Interpolate);
            }

            if (!IsInside(v0.ClipPosition) && !IsInside(v1.ClipPosition) && !IsInside(v2.ClipPosition))
                yield break;

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

            var vStart = output[0];
            for (int i = 1; i < outputCount - 1; i++)
            {
                yield return new[] { vStart, output[i], output[i + 1] };
            }
        }

        public static void RenderMesh(
            MainWindow window,
            Shaders.VertexInput[] vertices,
            int[] indices,
            Matrix4x4 model,
            Matrix4x4 view,
            Matrix4x4 projection,
            Shaders.VertexShader vertexShader,
            Shaders.FragmentShader fragmentShader,
            CullMode cullMode = CullMode.Back,
            DepthTest depthTest = DepthTest.LessEqual,
            BlendMode blendMode = BlendMode.Alpha)
        {
            if (window.RenderWidth <= 0 || window.RenderHeight <= 0) return;
            
            InitializeTileLocks(window.RenderWidth, window.RenderHeight);

            int triangleCount = indices.Length / 3;
            var triangleData = new (int, int, int)[triangleCount];
            
            for (int i = 0; i < triangleCount; i++)
            {
                int baseIdx = i * 3;
                triangleData[i] = (indices[baseIdx], indices[baseIdx + 1], indices[baseIdx + 2]);
            }

            Parallel.For(0, triangleCount, Options, i =>
            {
                var (i0, i1, i2) = triangleData[i];

                var v0 = vertexShader(vertices[i0], model, view, projection);
                var v1 = vertexShader(vertices[i1], model, view, projection);
                var v2 = vertexShader(vertices[i2], model, view, projection);

                bool v0Behind = v0.ClipPosition.W <= 0;
                bool v1Behind = v1.ClipPosition.W <= 0;
                bool v2Behind = v2.ClipPosition.W <= 0;

                if (v0Behind && v1Behind && v2Behind)
                {
                    return;
                }

                if (v0Behind || v1Behind || v2Behind)
                {
                    foreach (var triangle in ClipTriangleAgainstNearPlane(v0, v1, v2))
                    {
                        DrawTriangle(window, triangle[0], triangle[1], triangle[2], fragmentShader, cullMode,
                            depthTest, blendMode);
                    }
                }
                else
                {
                    DrawTriangle(window, v0, v1, v2, fragmentShader, cullMode, depthTest, blendMode);
                }
            });
        }

        private static void DrawLine(
            MainWindow window,
            Vector2D<int> p0,
            Vector2D<int> p1,
            float[] depths,
            Shaders.VertexOutput[] outputs,
            Shaders.FragmentShader fragmentShader,
            DepthTest depthTest,
            BlendMode blendMode)
        {
            int minX = Math.Max(Math.Min(p0.X, p1.X), 0);
            int maxX = Math.Min(Math.Max(p0.X, p1.X), window.RenderWidth - 1);
            int minY = Math.Max(Math.Min(p0.Y, p1.Y), 0);
            int maxY = Math.Min(Math.Max(p0.Y, p1.Y), window.RenderHeight - 1);

            if (minX > maxX || minY > maxY) return;

            var depthFunc = GetDepthTestFunction(depthTest);
            int tileSize = TileSize;
            var tileLocks = TileLocks;

            int tileMinX = minX / tileSize;
            int tileMaxX = maxX / tileSize;
            int tileMinY = minY / tileSize;
            int tileMaxY = maxY / tileSize;

            float dx = p1.X - p0.X;
            float dy = p1.Y - p0.Y;
            float lineLengthSq = dx * dx + dy * dy;

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

                    object[] tileLocksCopy;
                    int tilesXCopy, tilesYCopy;

                    lock (TileLocksInitLock)
                    {
                        tileLocksCopy = TileLocks;
                        tilesXCopy = TilesX;
                        tilesYCopy = TilesY;
                    }
                    
                    lock (tileLocksCopy[tileY * TilesX + tileX])
                    {
                        for (int y = startY; y <= endY; y++)
                        {
                            for (int x = startX; x <= endX; x++)
                            {
                                float px = x + 0.5f - p0.X;
                                float py = y + 0.5f - p0.Y;

                                float t = 0;
                                if (lineLengthSq > 0)
                                    t = (px * dx + py * dy) / lineLengthSq;

                                t = MathF.Max(0, MathF.Min(1, t));

                                float closestX = p0.X + t * dx;
                                float closestY = p0.Y + t * dy;

                                float distX = (x + 0.5f) - closestX;
                                float distY = (y + 0.5f) - closestY;
                                float distSq = distX * distX + distY * distY;

                                const float thresholdSq = 0.5f * 0.5f;
                                if (distSq <= thresholdSq)
                                {
                                    float depth = 1/(depths[0] * (1 - t) + depths[1] * t);
                                    float oldDepth = window.GetDepth(x, y);
                                    if (depthFunc(depth, oldDepth))
                                    {
                                        var interpolated = Interpolate(outputs[0], outputs[1], outputs[0], 1 - t, t, 0, outputs[0].Interpolate);
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
                            }
                        }
                    }
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
            
            
            if (TileLocks == null || TilesX != (window.RenderWidth + TileSize - 1) / TileSize ||
                TilesY != (window.RenderHeight + TileSize - 1) / TileSize)
            {
                InitializeTileLocks(window.RenderWidth, window.RenderHeight);
            }
            
            int renderWidth = window.RenderWidth;
            int renderHeight = window.RenderHeight;
            float invWidth = 1f / (renderWidth - 1);
            float invHeight = 1f / (renderHeight - 1);
            float halfRenderWidth = renderWidth * 0.5f;
            float halfRenderHeight = renderHeight * 0.5f;
            float halfWidthPlusHalf = halfRenderWidth + 0.5f;
            float halfHeightPlusHalf = halfRenderHeight + 0.5f;

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
                    (int)MathF.Round(viewPos.X * halfRenderWidth + halfWidthPlusHalf),
                    (int)MathF.Round(-viewPos.Y * halfRenderHeight + halfHeightPlusHalf)
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

            if (RenderDebugMode == DebugMode.Wireframe)
            {
                DrawLine(window, p0, p1, depths, outputs, fragmentShader, depthTest, blendMode);
                DrawLine(window, p1, p2, depths, outputs, fragmentShader, depthTest, blendMode);
                DrawLine(window, p2, p0, depths, outputs, fragmentShader, depthTest, blendMode);
                return;
            }

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

            int tileMinX = minX / tileSize;
            int tileMaxX = maxX / tileSize;
            int tileMinY = minY / tileSize;
            int tileMaxY = maxY / tileSize;

            var vector2D = p0;
            var p3 = p2;
            var vector2D1 = p1;
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

                    int w0Row = a12 * (startX - vector2D1.X) + b12 * (startY - vector2D1.Y);
                    int w1Row = a20 * (startX - p3.X) + b20 * (startY - p3.Y);
                    int w2Row = a01 * (startX - vector2D.X) + b01 * (startY - vector2D.Y);

                    object[] tileLocksCopy;
                    int tilesXCopy, tilesYCopy;

                    lock (TileLocksInitLock)
                    {
                        tileLocksCopy = TileLocks;
                        tilesXCopy = TilesX;
                        tilesYCopy = TilesY;
                    }
                    
                    lock (tileLocksCopy[tileY * TilesX + tileX])
                    {
                        for (int y = startY; y <= endY; y++)
                        {
                            int w0 = w0Row;
                            int w1 = w1Row;
                            int w2 = w2Row;

                            for (int x = startX; x <= endX; x++)
                            {
                                if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0))
                                {
                                    float w0f = w0 * invArea;
                                    float w1f = w1 * invArea;
                                    float w2f = w2 * invArea;

                                    float depth = depths[0] * w0f + depths[1] * w1f + depths[2] * w2f;

                                    float oldDepth = window.GetDepth(x, y);
                                    if (depthFunc(depth, oldDepth))
                                    {
                                        var interpolated = Interpolate(outputs[0], outputs[1], outputs[2], w0f, w1f,
                                            w2f, outputs[0].Interpolate);
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
            in Shaders.VertexOutput a,
            in Shaders.VertexOutput b,
            in Shaders.VertexOutput c,
            float w0,
            float w1,
            float w2,
            bool interpolate = true)
        {
            double invW0 = 1.0 / a.ClipPosition.W;
            double invW1 = 1.0 / b.ClipPosition.W;
            double invW2 = 1.0 / c.ClipPosition.W;
            double w0Inv = w0 * invW0;
            double w1Inv = w1 * invW1;
            double w2Inv = w2 * invW2;
            double oneOverW = w0Inv + w1Inv + w2Inv;
            float w = (float)(1.0 / oneOverW);

            float wa = (float)(w0Inv * w);
            float wb = (float)(w1Inv * w);
            float wc = (float)(w2Inv * w);

            Vector4 clipPos = (a.ClipPosition * (float)w0Inv + b.ClipPosition * (float)w1Inv + c.ClipPosition * (float)w2Inv) * w;
            Vector2 texCoord = (a.TexCoord * (float)w0Inv + b.TexCoord * (float)w1Inv + c.TexCoord * (float)(w2Inv)) * w;
            Vector2 screenCoords = (a.ScreenCoords * (float)w0Inv + b.ScreenCoords * (float)w1Inv + c.ScreenCoords * (float)(w2Inv)) * w;

            Vector3 normal = interpolate
                ? Vector3.Normalize((a.Normal * (float)w0Inv + b.Normal * (float)w1Inv + c.Normal * (float)w2Inv) * w)
                : a.Normal;

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPos,
                Color = interpolate
                    ? (a.Color * (float)w0Inv + b.Color * (float)w1Inv + c.Color * (float)w2Inv) * w
                    : a.Color,
                TexCoord = texCoord,
                Normal = normal,
                ScreenCoords = screenCoords,
                Data = interpolate
                    ? InterpolateData(a.Data, b.Data, c.Data, (float)(w0Inv), (float)(w1Inv), (float)(w2Inv), w)
                    : a.Data,
                Interpolate = interpolate,
                Barycentric = new Vector3(w0, w1, w2)
            };
        }

        private static Dictionary<string, object> InterpolateData(
            Dictionary<string, object> aData,
            Dictionary<string, object> bData,
            Dictionary<string, object> cData,
            float w0Inv,
            float w1Inv,
            float w2Inv,
            float w)
        {
            if (aData == null || bData == null || cData == null)
                return null;

            float w0 = w0Inv * w;
            float w1 = w1Inv * w;
            float w2 = w2Inv * w;

            var result = new Dictionary<string, object>(aData.Count);

            foreach (var kvp in aData)
            {
                string key = kvp.Key;
                if (!bData.TryGetValue(key, out var bValue) || !cData.TryGetValue(key, out var cValue))
                    continue;

                var aValue = kvp.Value;

                switch (aValue)
                {
                    case float fa when bValue is float fb && cValue is float fc:
                        result[key] = fa * w0 + fb * w1 + fc * w2;
                        break;

                    case Vector2 va2 when bValue is Vector2 vb2 && cValue is Vector2 vc2:
                        result[key] = va2 * w0 + vb2 * w1 + vc2 * w2;
                        break;

                    case Vector3 va3 when bValue is Vector3 vb3 && cValue is Vector3 vc3:
                        var vec3 = va3 * w0 + vb3 * w1 + vc3 * w2;
                        result[key] = vec3 != Vector3.Zero ? Vector3.Normalize(vec3) : vec3;
                        break;

                    case Vector4 va4 when bValue is Vector4 vb4 && cValue is Vector4 vc4:
                        result[key] = va4 * w0 + vb4 * w1 + vc4 * w2;
                        break;

                    default:
                        result[key] = aValue;
                        break;
                }
            }

            return result;
        }
    }
}