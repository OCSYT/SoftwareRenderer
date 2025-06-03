using System;
using System.Collections.Generic;
using System.Numerics;
using Silk.NET.Input;
using System.Threading.Tasks;
namespace SoftwareRenderer
{
    public class Renderer
    {
        public float Time;

        private class Camera
        {
            public Vector3 Position = new(0, 0, 0);
            public float Yaw;
            public float Pitch;
            public float Speed = 50f;
            public float Sensitivity = 0.1f;

            public Matrix4x4 GetViewMatrix() =>
                Matrix4x4.CreateLookAt(Position, Position + GetFront(), Vector3.UnitY);

            public Vector3 GetFront()
            {
                float yawRad = MathF.PI / 180f * Yaw;
                float pitchRad = MathF.PI / 180f * Pitch;

                return Vector3.Normalize(new Vector3(
                    MathF.Cos(yawRad) * MathF.Cos(pitchRad),
                    MathF.Sin(pitchRad),
                    MathF.Sin(yawRad) * MathF.Cos(pitchRad)));
            }
        }

        private static readonly Camera CameraObj = new();
        private static Vector2 LastMousePosition;
        private static bool FirstMouse = true;
        private Matrix4x4 ProjectionMatrix;

        private const int BaseChunkSize = 64;
        private const float ChunkScale = 0.25f;
        private const float HeightScale = 500f;
        private const int ViewDistanceInChunks = 64;
        private const int MaxLodLevel = 8;
        float skirtDepth = 5f;

        private readonly Dictionary<(int X, int Z, int Lod), (List<Shaders.VertexInput> Vertices, List<int> Indices, Matrix4x4 ModelMatrix)> LoadedChunks = new();
        private static readonly PerlinNoise Perlin = new(12345);
        private readonly object LoadedChunksLock = new();

        private (List<Shaders.VertexInput>, List<int>, Matrix4x4) CreateTerrainChunk(int chunkX, int chunkZ, int lodLevel)
        {
            int lodFactor = 1 << lodLevel;
            int effectiveChunkSize = BaseChunkSize / lodFactor;
            int width = effectiveChunkSize + 1;
            int depth = effectiveChunkSize + 1;
            float vertexSpacing = 2f * lodFactor;

            float[,] heightMap = new float[width, depth];
            var baseVertices = new List<(Vector3 Position, Vector2 TexCoord, Vector4 Color)>(width * depth);

            int octaves = 5;
            float persistence = 0.5f;
            float lacunarity = 2.0f;

            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < depth; z++)
                {
                    float worldX = ((chunkX * BaseChunkSize + x * lodFactor)) * 2f / 100;
                    float worldZ = ((chunkZ * BaseChunkSize + z * lodFactor) * 2f) / 100;

                    float amplitude = 1.0f;
                    float frequency = ChunkScale;
                    float noiseHeight = 0f;

                    for (int octave = 0; octave < octaves; octave++)
                    {
                        float sampleX = worldX * frequency;
                        float sampleZ = worldZ * frequency;

                        float perlinValue = Perlin.Noise(sampleX, sampleZ) * 2f - 1f;
                        noiseHeight += perlinValue * amplitude;

                        amplitude *= persistence;
                        frequency *= lacunarity;
                    }

                    heightMap[x, z] = noiseHeight * HeightScale * 0.5f;
                }
            }

            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < depth; z++)
                {
                    float y = heightMap[x, z];
                    if (y < 0) y = 0;

                    Vector4 color = y < 0.01f
                        ? new Vector4(0.0f, 0.3f, 0.7f, 1f)          // water
                        : y < HeightScale * 0.05f
                            ? new Vector4(0.76f, 0.7f, 0.5f, 1f)    // sand
                            : y < HeightScale * 0.2f
                                ? new Vector4(0.3f, 0.8f, 0.3f, 1f) // grass
                                : new Vector4(0.5f, 0.5f, 0.5f, 1f); // mountain (gray)


                    var position = new Vector3(x * vertexSpacing, y, z * vertexSpacing);
                    var texCoord = new Vector2((float)x / effectiveChunkSize, (float)z / effectiveChunkSize);
                    baseVertices.Add((position, texCoord, color));
                }
            }

            var normals = new Vector3[width, depth];
            var vertexCount = new int[width, depth];

            for (int x = 0; x < effectiveChunkSize; x++)
            {
                for (int z = 0; z < effectiveChunkSize; z++)
                {
                    Vector3 v0 = baseVertices[x * depth + z].Position;
                    Vector3 v1 = baseVertices[(x + 1) * depth + z].Position;
                    Vector3 v2 = baseVertices[x * depth + (z + 1)].Position;
                    Vector3 v3 = baseVertices[(x + 1) * depth + (z + 1)].Position;

                    Vector3 normal1 = Vector3.Normalize(Vector3.Cross(v2 - v0, v1 - v0));
                    Vector3 normal2 = Vector3.Normalize(Vector3.Cross(v2 - v1, v3 - v1));

                    normals[x, z] += normal1;
                    normals[x, z + 1] += normal1;
                    normals[x + 1, z] += normal1 + normal2;
                    normals[x, z + 1] += normal2;
                    normals[x + 1, z + 1] += normal2;

                    vertexCount[x, z]++;
                    vertexCount[x, z + 1]++;
                    vertexCount[x + 1, z]++;
                    vertexCount[x + 1, z + 1]++;
                }
            }

            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < depth; z++)
                {
                    if (vertexCount[x, z] > 0)
                    {
                        normals[x, z] = Vector3.Normalize(normals[x, z] / vertexCount[x, z]);
                    }
                }
            }

            var vertices = new List<Shaders.VertexInput>();
            var indices = new List<int>();

            for (int x = 0; x < effectiveChunkSize; x++)
            {
                for (int z = 0; z < effectiveChunkSize; z++)
                {
                    int i0 = x * depth + z;
                    int i1 = (x + 1) * depth + z;
                    int i2 = x * depth + (z + 1);
                    int i3 = (x + 1) * depth + (z + 1);

                    vertices.Add(new Shaders.VertexInput(
                        baseVertices[i0].Position,
                        baseVertices[i0].TexCoord,
                        normals[x, z],
                        baseVertices[i0].Color));

                    vertices.Add(new Shaders.VertexInput(
                        baseVertices[i2].Position,
                        baseVertices[i2].TexCoord,
                        normals[x, z + 1],
                        baseVertices[i2].Color));

                    vertices.Add(new Shaders.VertexInput(
                        baseVertices[i1].Position,
                        baseVertices[i1].TexCoord,
                        normals[x + 1, z],
                        baseVertices[i1].Color));

                    vertices.Add(new Shaders.VertexInput(
                        baseVertices[i3].Position,
                        baseVertices[i3].TexCoord,
                        normals[x + 1, z + 1],
                        baseVertices[i3].Color));

                    int baseIdx = vertices.Count - 4;
                    indices.Add(baseIdx);
                    indices.Add(baseIdx + 1);
                    indices.Add(baseIdx + 2);

                    indices.Add(baseIdx + 2);
                    indices.Add(baseIdx + 1);
                    indices.Add(baseIdx + 3);
                }
            }

            Matrix4x4 modelMatrix = Matrix4x4.CreateTranslation(
                chunkX * BaseChunkSize * 2f,
                0,
                chunkZ * BaseChunkSize * 2f);
            
            return (vertices, indices, modelMatrix);
        }

        private int DetermineLodLevel(Vector3 chunkCenter)
        {
            float distance = Vector3.Distance(chunkCenter, CameraObj.Position);
            float lodDistanceThreshold = 50f;
            
            for (int lod = 0; lod < MaxLodLevel; lod++)
            {
                if (distance < lodDistanceThreshold * (1 << lod))
                {
                    return lod;
                }
            }
            return MaxLodLevel;
        }

        private void RenderTerrain(MainWindow window)
        {
            int chunkWorldSize = BaseChunkSize * 2;
            int currentChunkX = (int)(CameraObj.Position.X / chunkWorldSize);
            int currentChunkZ = (int)(CameraObj.Position.Z / chunkWorldSize);

            var neededChunks = new HashSet<(int, int, int)>(ViewDistanceInChunks * ViewDistanceInChunks * (MaxLodLevel + 1));

            for (int dx = -ViewDistanceInChunks; dx <= ViewDistanceInChunks; dx++)
            {
                for (int dz = -ViewDistanceInChunks; dz <= ViewDistanceInChunks; dz++)
                {
                    int chunkX = currentChunkX + dx;
                    int chunkZ = currentChunkZ + dz;
                    
                    Vector3 chunkCenter = new Vector3(
                        chunkX * chunkWorldSize + chunkWorldSize / 2f,
                        0,
                        chunkZ * chunkWorldSize + chunkWorldSize / 2f);
                    
                    int lodLevel = DetermineLodLevel(chunkCenter);
                    neededChunks.Add((chunkX, chunkZ, lodLevel));
                    
                }
            }

            List<(int, int, int)> chunksToLoad;
            List<(int, int, int)> chunksToRemove;

            lock (LoadedChunksLock)
            {
                chunksToRemove = new List<(int, int, int)>(LoadedChunks.Count);
                chunksToLoad = new List<(int, int, int)>(neededChunks.Count);

                foreach (var key in LoadedChunks.Keys)
                {
                    if (!neededChunks.Contains(key))
                    {
                        chunksToRemove.Add(key);
                    }
                }

                foreach (var chunkCoord in neededChunks)
                {
                    if (!LoadedChunks.ContainsKey(chunkCoord))
                    {
                        chunksToLoad.Add(chunkCoord);
                    }
                }
            }

            if (chunksToRemove.Count > 0)
            {
                var batchSize = Math.Max(1, chunksToRemove.Count / Environment.ProcessorCount);
                Parallel.For(0, (chunksToRemove.Count + batchSize - 1) / batchSize, batchIndex =>
                {
                    int start = batchIndex * batchSize;
                    int end = Math.Min(start + batchSize, chunksToRemove.Count);

                    lock (LoadedChunksLock)
                    {
                        for (int i = start; i < end; i++)
                        {
                            LoadedChunks.Remove(chunksToRemove[i]);
                        }
                    }
                });
            }

            if (chunksToLoad.Count > 0)
            {
                var batchSize = Math.Max(1, chunksToLoad.Count / Environment.ProcessorCount);
                Parallel.For(0, (chunksToLoad.Count + batchSize - 1) / batchSize, batchIndex =>
                {
                    int start = batchIndex * batchSize;
                    int end = Math.Min(start + batchSize, chunksToLoad.Count);

                    for (int i = start; i < end; i++)
                    {
                        var chunkCoord = chunksToLoad[i];
                        var chunk = CreateTerrainChunk(chunkCoord.Item1, chunkCoord.Item2, chunkCoord.Item3);

                        lock (LoadedChunksLock)
                        {
                            if (!LoadedChunks.ContainsKey(chunkCoord))
                            {
                                LoadedChunks[chunkCoord] = chunk;
                            }
                        }
                    }
                });
            }

            List<(List<Shaders.VertexInput>, List<int>, Matrix4x4)> chunksSnapshot;
            lock (LoadedChunksLock)
            {
                chunksSnapshot = new List<(List<Shaders.VertexInput>, List<int>, Matrix4x4)>(LoadedChunks.Count);
                foreach (var chunk in LoadedChunks.Values)
                {
                    if (chunk.Vertices.Count > 0)
                    {
                        chunksSnapshot.Add(chunk);
                    }
                }
            }

            if (chunksSnapshot.Count > 0)
            {
                var batchSize = Math.Max(1, chunksSnapshot.Count / Environment.ProcessorCount);
                Parallel.For(0, (chunksSnapshot.Count + batchSize - 1) / batchSize, batchIndex =>
                {
                    int start = batchIndex * batchSize;
                    int end = Math.Min(start + batchSize, chunksSnapshot.Count);

                    for (int i = start; i < end; i++)
                    {
                        var chunk = chunksSnapshot[i];
                        Rasterizer.RenderMesh(
                            window,
                            chunk.Item1.ToArray(),
                            chunk.Item2.ToArray(),
                            chunk.Item3,
                            CameraObj.GetViewMatrix(),
                            ProjectionMatrix,
                            VertexShader,
                            input => FragmentShader(input));
                    }
                });
            }
        }

        private Shaders.VertexOutput VertexShader(Shaders.VertexInput vertex, Matrix4x4 modelMatrix,
            Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            Vector4 worldPosition = Vector4.Transform(new Vector4(vertex.Position, 1), modelMatrix);
            Vector4 viewPosition = Vector4.Transform(worldPosition, viewMatrix);
            Vector4 clipPosition = Vector4.Transform(viewPosition, projectionMatrix);

            Vector3 worldNormal = Vector3.TransformNormal(vertex.Normal, modelMatrix);
            worldNormal = Vector3.Normalize(worldNormal);

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPosition,
                Data = { ["WorldNormal"] = worldNormal },
                TexCoord = vertex.TexCoord,
                Color = vertex.Color,
                Normal = vertex.Normal,
                Interpolate = false
            };
        }

        private Vector4 FragmentShader(Shaders.VertexOutput input)
        {
            Vector3 lightDir = Vector3.Normalize(new Vector3(-0.3f, -1f, 0.3f));
            Vector3 worldNormal = (Vector3)input.Data["WorldNormal"];
            float diff = MathF.Max(0f, Vector3.Dot(worldNormal, -lightDir));
            
            Vector4 baseColor = input.Color;

            float fogStart = 500f;
            float fogEnd = 1000f;
            Vector4 fogColor = new Vector4(0.5f, 0.6f, 1f, 1f);
            float depth = input.ClipPosition.Z;
            float fogFactor = Math.Clamp((fogEnd - depth) / (fogEnd - fogStart), 0f, 1f);
            fogFactor = fogFactor * fogFactor * (3f - 2f * fogFactor);
            Vector4 finalColor = Vector4.Lerp(fogColor, baseColor * (0.1f + 0.9f * diff), fogFactor);

            return new Vector4(finalColor.X, finalColor.Y, finalColor.Z, 1f);
        }

        public void Main(string[] args)
        {
            var window = new MainWindow("Software Renderer Demo");
            window.RenderScale = 1/4f;

            IInputContext inputContext = null;
            IKeyboard keyboard = null;
            IMouse mouse = null;
            
            window.StartEvent += () =>
            {
                inputContext = window.InputContext;
                keyboard = inputContext.Keyboards[0];
                mouse = inputContext.Mice[0];
                mouse.Cursor.CursorMode = CursorMode.Disabled;

                mouse.MouseMove += (_, pos) =>
                {
                    if (FirstMouse)
                    {
                        LastMousePosition = pos;
                        FirstMouse = false;
                    }

                    var delta = pos - LastMousePosition;
                    LastMousePosition = pos;

                    CameraObj.Yaw += delta.X * CameraObj.Sensitivity;
                    CameraObj.Pitch -= delta.Y * CameraObj.Sensitivity;
                    CameraObj.Pitch = Math.Clamp(CameraObj.Pitch, -89f, 89f);
                };
            };

            Rasterizer.RenderDebugMode = Rasterizer.DebugMode.None;
            window.UpdateEvent += deltaTime =>
            {
                Console.WriteLine("FPS: " + MathF.Round((float)(1 / deltaTime)));
                ProjectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
                    MathF.PI / 3,
                    (float)window.RenderWidth / window.RenderHeight,
                    1f,
                    100f);

                Time += (float)deltaTime;

                var front = CameraObj.GetFront();
                var right = Vector3.Normalize(Vector3.Cross(front, Vector3.UnitY));

                if (keyboard.IsKeyPressed(Key.W))
                    CameraObj.Position += front * CameraObj.Speed * (float)deltaTime;
                if (keyboard.IsKeyPressed(Key.S))
                    CameraObj.Position -= front * CameraObj.Speed * (float)deltaTime;
                if (keyboard.IsKeyPressed(Key.A))
                    CameraObj.Position -= right * CameraObj.Speed * (float)deltaTime;
                if (keyboard.IsKeyPressed(Key.D))
                    CameraObj.Position += right * CameraObj.Speed * (float)deltaTime;

                window.ClearDepthBuffer();
                window.ClearColorBuffer(new Vector4(0.392f, 0.584f, 0.929f, 1.0f));

                RenderTerrain(window);
                window.RenderFrame();
            };

            window.Run();
        }
    }
}