using System;
using System.Buffers;
using System.Collections.Concurrent;
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

        public static float NearClip = 0.1f;
        public static float FarClip = 1000f;
        public static DebugMode RenderDebugMode = DebugMode.None;
        static ParallelOptions Options = new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount };

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
        private const int TileSize = 16;
        private static object[] TileLocks;
        private static int TilesX, TilesY;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        private static List<Shaders.VertexOutput[]> ClipTriangleAgainstNearPlane(
            Shaders.VertexOutput v0,
            Shaders.VertexOutput v1,
            Shaders.VertexOutput v2)
        {
            var outputTriangles = new List<Shaders.VertexOutput[]>();

            // List to hold vertices after clipping
            var inputVertices = new List<Shaders.VertexOutput> { v0, v1, v2 };
            var clippedVertices = new List<Shaders.VertexOutput>();

            // Clip against near plane (z >= NearClip * w in clip space)
            for (int i = 0; i < inputVertices.Count; i++)
            {
                var current = inputVertices[i];
                var next = inputVertices[(i + 1) % inputVertices.Count];

                bool currentInside = current.ClipPosition.Z >= NearClip * current.ClipPosition.W;
                bool nextInside = next.ClipPosition.Z >= NearClip * next.ClipPosition.W;

                if (currentInside)
                {
                    clippedVertices.Add(current);
                }

                if (currentInside != nextInside)
                {
                    // Interpolate vertex at the near plane
                    float t;
                    float z0 = current.ClipPosition.Z;
                    float w0 = current.ClipPosition.W;
                    float z1 = next.ClipPosition.Z;
                    float w1 = next.ClipPosition.W;

                    // Linearly interpolate in clip space: z = NearClip * w
                    // Solve for t where: z0 + t * (z1 - z0) = NearClip * (w0 + t * (w1 - w0))
                    float denom = (z1 - z0) - NearClip * (w1 - w0);
                    if (Math.Abs(denom) < Epsilon)
                    {
                        t = 0.5f; // Fallback to avoid division by zero
                    }
                    else
                    {
                        t = (z0 - NearClip * w0) / (NearClip * (w1 - w0) - (z1 - z0));
                        t = Math.Clamp(t, 0f, 1f);
                    }

                    var interpolated = Shaders.Lerp(current, next, t, true);
                    clippedVertices.Add(interpolated);
                }
            }

            // If we have fewer than 3 vertices after clipping, no triangle is formed
            if (clippedVertices.Count < 3)
            {
                return outputTriangles;
            }

            // Triangulate the clipped polygon (fan triangulation)
            for (int i = 1; i < clippedVertices.Count - 1; i++)
            {
                outputTriangles.Add(new[] { clippedVertices[0], clippedVertices[i], clippedVertices[i + 1] });
            }

            return outputTriangles;
        }
        

        public static void RenderMesh(
            MainWindow window,
            Shaders.VertexInput[] vertices,
            ushort[] indices,
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
            var triangleData = new (int i0, int i1, int i2, float avgDepth)[triangleCount];

            // Compute triangle data and depths
            for (int i = 0; i < triangleCount; i++)
            {
                int baseIdx = i * 3;
                var v0 = vertexShader(vertices[indices[baseIdx]], model, view, projection);
                var v1 = vertexShader(vertices[indices[baseIdx + 1]], model, view, projection);
                var v2 = vertexShader(vertices[indices[baseIdx + 2]], model, view, projection);

                // Compute average depth (in NDC space)
                float avgDepth = (v0.ClipPosition.Z / v0.ClipPosition.W +
                                  v1.ClipPosition.Z / v1.ClipPosition.W +
                                  v2.ClipPosition.Z / v2.ClipPosition.W) / 3f;

                triangleData[i] = (indices[baseIdx], indices[baseIdx + 1], indices[baseIdx + 2], avgDepth);
            }


            Parallel.For(0, triangleCount, Options, i =>
            {
                var (i0, i1, i2, _) = triangleData[i];

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
            Vector2 p0,
            Vector2 p1,
            float[] depths,
            Shaders.VertexOutput[] outputs,
            Shaders.FragmentShader fragmentShader,
            DepthTest depthTest,
            BlendMode blendMode)
        {
            int minX = (int)Math.Max(Math.Min(p0.X, p1.X), 0);
            int maxX = (int)Math.Min(Math.Max(p0.X, p1.X), window.RenderWidth - 1);
            int minY = (int)Math.Max(Math.Min(p0.Y, p1.Y), 0);
            int maxY = (int)Math.Min(Math.Max(p0.Y, p1.Y), window.RenderHeight - 1);

            if (minX > maxX || minY > maxY) return;

            var depthFunc = GetDepthTestFunction(depthTest);
            int tileSize = TileSize;

            int tileMinX = minX / tileSize;
            int tileMaxX = maxX / tileSize;
            int tileMinY = minY / tileSize;
            int tileMaxY = maxY / tileSize;

            float dx = p1.X - p0.X;
            float dy = p1.Y - p0.Y;
            float lineLengthSq = dx * dx + dy * dy;

            // Compute line depth bounds for tile culling
            float minDepth = Math.Min(depths[0], depths[1]);
            float maxDepth = Math.Max(depths[0], depths[1]);

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
                    int tilesXCopy;

                    lock (TileLocksInitLock)
                    {
                        tileLocksCopy = TileLocks;
                        tilesXCopy = TilesX;
                    }

                    lock (tileLocksCopy[tileY * tilesXCopy + tileX])
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
                                    float depth = 1 / (depths[0] * (1 - t) + depths[1] * t);
                                    float oldDepth = window.GetDepth(x, y);

                                    if (!depthFunc(depth, oldDepth))
                                        continue;

                                    var interpolated = Interpolate(outputs[0], outputs[1], outputs[0], 1 - t, t, 0,
                                        outputs[0].Interpolate);
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

            Vector2[] screenCoords = new Vector2[3];
            float[] depths = new float[3];
            Shaders.VertexOutput[] outputs = new Shaders.VertexOutput[] { v2, v1, v0 };

            for (int i = 0; i < 3; i++)
            {
                float invW = 1f / outputs[i].ClipPosition.W;
                Vector3 ndc = new Vector3(
                    outputs[i].ClipPosition.X * invW,
                    outputs[i].ClipPosition.Y * invW,
                    outputs[i].ClipPosition.Z * invW
                );

                if (float.IsNaN(ndc.X) || float.IsNaN(ndc.Y) || float.IsNaN(ndc.Z) ||
                    float.IsInfinity(ndc.X) || float.IsInfinity(ndc.Y) || float.IsInfinity(ndc.Z))
                    return;

                // Remove vertex snapping - keep full floating point precision
                screenCoords[i] = new Vector2(
                    (ndc.X * 0.5f + 0.5f) * renderWidth,
                    (1.0f - (ndc.Y * 0.5f + 0.5f)) * renderHeight
                );

                depths[i] = (ndc.Z + 1.0f) * 0.5f;

                outputs[i].ScreenCoords = new Vector2(screenCoords[i].X * invWidth, screenCoords[i].Y * invHeight);
            }

            if (v0.ClipPosition.W == 0 || v1.ClipPosition.W == 0 || v2.ClipPosition.W == 0)
                return;

            if (EdgeFunction(screenCoords[0], screenCoords[1], screenCoords[2]) == 0) return;

            RasterizeTriangle(window, screenCoords, depths, outputs, fragmentShader, cullMode, depthTest, blendMode);
        }

        private static void RasterizeTriangle(
            MainWindow window,
            Vector2[] screen,
            float[] depths,
            Shaders.VertexOutput[] outputs,
            Shaders.FragmentShader fragmentShader,
            CullMode cullMode,
            DepthTest depthTest,
            BlendMode blendMode)
        {
            float area = EdgeFunction(screen[0], screen[1], screen[2]);
            if (area == 0) return;

            bool isFrontFace = area < 0;
            if ((cullMode == CullMode.Back && !isFrontFace) ||
                (cullMode == CullMode.Front && isFrontFace))
                return;

            if (RenderDebugMode == DebugMode.Wireframe)
            {
                DrawLine(window, screen[0], screen[1], depths, outputs, fragmentShader, depthTest, blendMode);
                DrawLine(window, screen[1], screen[2], depths, outputs, fragmentShader, depthTest, blendMode);
                DrawLine(window, screen[2], screen[0], depths, outputs, fragmentShader, depthTest, blendMode);
                return;
            }

            float invArea = 1f / area;
            var depthFunc = GetDepthTestFunction(depthTest);
            bool needsDepthTest = depthTest != DepthTest.Disabled;
            bool canEarlyOut = blendMode == BlendMode.None;

            float minXf = MathF.Min(MathF.Min(screen[0].X, screen[1].X), screen[2].X);
            float maxXf = MathF.Max(MathF.Max(screen[0].X, screen[1].X), screen[2].X);
            float minYf = MathF.Min(MathF.Min(screen[0].Y, screen[1].Y), screen[2].Y);
            float maxYf = MathF.Max(MathF.Max(screen[0].Y, screen[1].Y), screen[2].Y);

            int minX = Math.Max((int)MathF.Floor(minXf), 0);
            int maxX = Math.Min((int)MathF.Ceiling(maxXf), window.RenderWidth - 1);
            int minY = Math.Max((int)MathF.Floor(minYf), 0);
            int maxY = Math.Min((int)MathF.Ceiling(maxYf), window.RenderHeight - 1);

            if (minX > maxX || minY > maxY) return;

            // Precompute edge deltas
            float a01 = screen[0].Y - screen[1].Y, b01 = screen[1].X - screen[0].X;
            float a12 = screen[1].Y - screen[2].Y, b12 = screen[2].X - screen[1].X;
            float a20 = screen[2].Y - screen[0].Y, b20 = screen[0].X - screen[2].X;

            int tileMinX = minX / TileSize;
            int tileMaxX = maxX / TileSize;
            int tileMinY = minY / TileSize;
            int tileMaxY = maxY / TileSize;

            var depth0 = depths[0];
            var depth1 = depths[1];
            var depth2 = depths[2];

            var output0 = outputs[0];
            var output1 = outputs[1];
            var output2 = outputs[2];

            Parallel.For(tileMinY, tileMaxY + 1, tileY =>
            {
                for (int tileX = tileMinX; tileX <= tileMaxX; tileX++)
                {
                    int tileStartX = tileX * TileSize;
                    int tileEndX = Math.Min(tileStartX + TileSize - 1, window.RenderWidth - 1);
                    int tileStartY = tileY * TileSize;
                    int tileEndY = Math.Min(tileStartY + TileSize - 1, window.RenderHeight - 1);

                    int startX = Math.Max(minX, tileStartX);
                    int endX = Math.Min(maxX, tileEndX);
                    int startY = Math.Max(minY, tileStartY);
                    int endY = Math.Min(maxY, tileEndY);

                    if (startX > endX || startY > endY) continue;

                    object tileLock = TileLocks[tileY * TilesX + tileX];
                    lock (tileLock)
                    {
                        float w0Row = a12 * (startX - screen[1].X) + b12 * (startY - screen[1].Y);
                        float w1Row = a20 * (startX - screen[2].X) + b20 * (startY - screen[2].Y);
                        float w2Row = a01 * (startX - screen[0].X) + b01 * (startY - screen[0].Y);

                        for (int y = startY; y <= endY; y++)
                        {
                            float w0 = w0Row;
                            float w1 = w1Row;
                            float w2 = w2Row;

                            for (int x = startX; x <= endX; x++)
                            {
                                bool centerInside = (w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                                                    (w0 <= 0 && w1 <= 0 && w2 <= 0);

                                if (centerInside)
                                {
                                    float w0f = w0 * invArea;
                                    float w1f = w1 * invArea;
                                    float w2f = w2 * invArea;

                                    float depth = depth0 * w0f + depth1 * w1f + depth2 * w2f;
                                    float oldDepth = window.GetDepth(x, y);

                                    if (depthFunc(depth, oldDepth))
                                    {
                                        var interpolated = Interpolate(output0, output1, output2, w0f, w1f, w2f,
                                            output0.Interpolate);
                                        var sampleColor = fragmentShader(interpolated);

                                        if (sampleColor is { W: > 0f } color)
                                        {
                                            var dst = window.GetPixel(x, y);
                                            var blended = Blend(color, dst, blendMode);
                                            window.SetPixel(x, y, blended);

                                            if (needsDepthTest)
                                                window.SetDepth(x, y, depth);
                                        }
                                        else if (canEarlyOut)
                                        {
                                            break;
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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Func<float, float, bool> GetDepthTestFunction(DepthTest test)
        {
            if (test == DepthTest.LessEqual)
                return (newDepth, oldDepth) => newDepth >= oldDepth;

            return test switch
            {
                DepthTest.Disabled => (newDepth, oldDepth) => true,
                DepthTest.Less => (newDepth, oldDepth) => newDepth > oldDepth,
                DepthTest.Greater => (newDepth, oldDepth) => newDepth < oldDepth,
                DepthTest.GreaterEqual => (newDepth, oldDepth) => newDepth <= oldDepth,
                DepthTest.Equal => (newDepth, oldDepth) => Math.Abs(newDepth - oldDepth) < Epsilon,
                DepthTest.NotEqual => (newDepth, oldDepth) => Math.Abs(newDepth - oldDepth) >= Epsilon,
                DepthTest.Always => (newDepth, oldDepth) => true,
                _ => (newDepth, oldDepth) => true
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float EdgeFunction(Vector2 a, Vector2 b, Vector2 c) =>
            (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Shaders.VertexOutput Interpolate(
            in Shaders.VertexOutput a,
            in Shaders.VertexOutput b,
            in Shaders.VertexOutput c,
            float w0,
            float w1,
            float w2,
            bool interpolate = true)
        {
            // Single reciprocal calculation and reuse
            float rcpWa = w0 / a.ClipPosition.W;
            float rcpWb = w1 / b.ClipPosition.W;
            float rcpWc = w2 / c.ClipPosition.W;
            float invWSum = rcpWa + rcpWb + rcpWc;

            // Single reciprocal for perspective correction
            float w = 1.0f / invWSum;
            float wa = rcpWa * w;
            float wb = rcpWb * w;
            float wc = rcpWc * w;

            // Vectorized interpolation using SIMD-friendly operations
            Vector4 clipPosition = Vector4.Multiply(a.ClipPosition, rcpWa);
            clipPosition = Vector4.Add(clipPosition, Vector4.Multiply(b.ClipPosition, rcpWb));
            clipPosition = Vector4.Add(clipPosition, Vector4.Multiply(c.ClipPosition, rcpWc));
            clipPosition = Vector4.Multiply(clipPosition, w);

            Vector2 texCoord = Vector2.Multiply(a.TexCoord, rcpWa);
            texCoord = Vector2.Add(texCoord, Vector2.Multiply(b.TexCoord, rcpWb));
            texCoord = Vector2.Add(texCoord, Vector2.Multiply(c.TexCoord, rcpWc));
            texCoord = Vector2.Multiply(texCoord, w);

            Vector2 screenCoords = Vector2.Multiply(a.ScreenCoords, rcpWa);
            screenCoords = Vector2.Add(screenCoords, Vector2.Multiply(b.ScreenCoords, rcpWb));
            screenCoords = Vector2.Add(screenCoords, Vector2.Multiply(c.ScreenCoords, rcpWc));
            screenCoords = Vector2.Multiply(screenCoords, w);

            // Conditional interpolation with minimal branching
            Vector3 normal;
            Vector4 color;
            Dictionary<string, object>? data;

            if (interpolate)
            {
                normal = Vector3.Multiply(a.Normal, rcpWa);
                normal = Vector3.Add(normal, Vector3.Multiply(b.Normal, rcpWb));
                normal = Vector3.Add(normal, Vector3.Multiply(c.Normal, rcpWc));
                normal = Vector3.Multiply(normal, w);

                color = Vector4.Multiply(a.Color, rcpWa);
                color = Vector4.Add(color, Vector4.Multiply(b.Color, rcpWb));
                color = Vector4.Add(color, Vector4.Multiply(c.Color, rcpWc));
                color = Vector4.Multiply(color, w);

                data = InterpolateData(a.Data, b.Data, c.Data, wa, wb, wc);
            }
            else
            {
                normal = a.Normal;
                color = a.Color;
                data = a.Data;
            }

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPosition,
                TexCoord = texCoord,
                ScreenCoords = screenCoords,
                Normal = normal,
                Color = color,
                Data = data,
                Interpolate = interpolate,
                Barycentric = new Vector3(wa, wb, wc)
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Dictionary<string, object>? InterpolateData(
            Dictionary<string, object>? aData,
            Dictionary<string, object>? bData,
            Dictionary<string, object>? cData,
            float w0,
            float w1,
            float w2)
        {
            // Early exit for null cases
            if (aData == null) return bData ?? cData;
            if (bData == null || cData == null) return aData;

            // Pre-allocate with exact size to avoid resizing
            var result = new Dictionary<string, object>(aData.Count);

            // Use enumerator directly for better performance
            foreach (var kvp in aData)
            {
                var key = kvp.Key;
                var aValue = kvp.Value;

                // Use pattern matching for better branch prediction
                if (bData.TryGetValue(key, out var bValue) &&
                    cData.TryGetValue(key, out var cValue))
                {
                    // Inline type-specific interpolation for better performance
                    switch (aValue)
                    {
                        case float fa when bValue is float fb && cValue is float fc:
                            result[key] = fa * w0 + fb * w1 + fc * w2;
                            break;

                        case Vector2 va2 when bValue is Vector2 vb2 && cValue is Vector2 vc2:
                            result[key] = Vector2.Multiply(va2, w0) + Vector2.Multiply(vb2, w1) +
                                          Vector2.Multiply(vc2, w2);
                            break;

                        case Vector3 va3 when bValue is Vector3 vb3 && cValue is Vector3 vc3:
                            var interpolated = Vector3.Multiply(va3, w0) + Vector3.Multiply(vb3, w1) +
                                               Vector3.Multiply(vc3, w2);
                            // Avoid expensive square root when possible
                            var lengthSq = interpolated.LengthSquared();
                            result[key] = lengthSq > 1e-6f
                                ? Vector3.Multiply(interpolated, 1.0f / MathF.Sqrt(lengthSq))
                                : interpolated;
                            break;

                        case Vector4 va4 when bValue is Vector4 vb4 && cValue is Vector4 vc4:
                            result[key] = Vector4.Multiply(va4, w0) + Vector4.Multiply(vb4, w1) +
                                          Vector4.Multiply(vc4, w2);
                            break;

                        default:
                            result[key] = aValue;
                            break;
                    }
                }
                else
                {
                    result[key] = aValue;
                }
            }

            return result.Count > 0 ? result : null;
        }
    }
}