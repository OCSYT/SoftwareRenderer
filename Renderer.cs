using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using Silk.NET.Input;
using Silk.NET.OpenGL.Extensions.ImGui;
using ImGuiNET;

namespace SoftwareRenderer
{
    public class Renderer
    {
        public float Time;
        public bool MouseLocked = true;
        private bool WasEscapePressed = false;

        private float NearClip = 0.1f;
        private float FarClip = 100f;
        private float FogStart = 1;
        private float FogEnd = 50;
        private Vector4 FogColor = new(0.5f, 0.6f, 1f, 1f);
        private Vector3 LightEulerDegrees = new Vector3(-45, -45, 0);
        private Vector3 LightDir = EulerToDirection(new Vector3(-45, -45, 0));
        private Vector4 LightColor = Vector4.One;
        private Vector3 ClearColor = Vector3.Zero;
        private float FOV = 90;
        private bool LoadedLayout = false;
        private class Camera
        {
            public Vector3 Position = new Vector3(-10.610103f, 0.44594023f, -36.2275f);
            public float Yaw = 90;
            public float Pitch;
            public float Speed = 10f;
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
        private SceneModel E1M1Model;
        private readonly ConcurrentDictionary<string, Texture> CachedTextures = new();

        private class SceneModel
        {
            public List<Mesh> Meshes { get; init; }
        }

        private static Vector3[] GetSphereOffsets(float radius)
        {
            List<Vector3> offsets = new(25) { Vector3.Zero };
            Vector3[] directions =
            {
                new(1, 0, 0), new(-1, 0, 0), new(0, 1, 0), new(0, -1, 0), new(0, 0, 1), new(0, 0, -1),
                new(1, 1, 0), new(-1, 1, 0), new(1, -1, 0), new(-1, -1, 0),
                new(1, 0, 1), new(-1, 0, 1), new(1, 0, -1), new(-1, 0, -1),
                new(0, 1, 1), new(0, -1, 1), new(0, 1, -1), new(0, -1, -1),
                new(1, 1, 1), new(-1, -1, -1), new(-1, 1, 1), new(1, -1, -1),
                new(1, -1, 1), new(-1, 1, -1)
            };

            foreach (var dir in directions)
                offsets.Add(Vector3.Normalize(dir) * radius);

            return offsets.ToArray();
        }

        private static readonly Vector3[] SphereOffsets = GetSphereOffsets(0.15f);

        private Vector3 MoveWithSlide(Vector3 currentPos, Vector3 desiredPos, float radius = 0.15f, int depth = 0)
        {
            const int MaxSlideAttempts = 3;
            if (depth >= MaxSlideAttempts || E1M1Model == null)
                return currentPos;

            Vector3 moveVector = desiredPos - currentPos;
            if (moveVector.LengthSquared() < 1e-6f)
                return currentPos;

            Vector3 direction = Vector3.Normalize(moveVector);
            Vector3 hitNormal = Vector3.Zero;
            float nearestHitDistance = float.MaxValue;
            bool collisionDetected = false;

            var modelMatrix = Matrix4x4.CreateScale(0.01f);

            foreach (var mesh in E1M1Model.Meshes)
            {
                var vertices = mesh.Vertices.ToArray();
                var indices = mesh.Indices.ToArray();

                foreach (var offset in SphereOffsets)
                {
                    Vector3 rayOrigin = currentPos + offset;

                    if (Physics.Raycast(rayOrigin, direction, vertices, indices, modelMatrix,
                        out float hitDistance, out _, out Vector3 normal))
                    {
                        if (hitDistance * hitDistance < moveVector.LengthSquared() && hitDistance < nearestHitDistance)
                        {
                            nearestHitDistance = hitDistance;
                            hitNormal = normal;
                            collisionDetected = true;
                        }
                    }
                }
            }

            if (!collisionDetected)
                return desiredPos;

            Vector3 stopPos = currentPos + direction * (nearestHitDistance - 0.01f);
            Vector3 remainingMove = desiredPos - stopPos;
            Vector3 projectedMove = remainingMove - Vector3.Dot(remainingMove, hitNormal) * hitNormal;

            return projectedMove.LengthSquared() < 1e-4f
                ? stopPos
                : MoveWithSlide(stopPos, stopPos + projectedMove, radius, depth + 1);
        }
        

        private int RenderedModels;
        private void RenderE1M1(MainWindow window)
        {
            RenderedModels = 0;
            if (E1M1Model == null)
            {
                var result = Model.LoadModel("./Assets/e1m1/doom_E1M1.obj");
                E1M1Model = new SceneModel { Meshes = result.Meshes };
            }

            var modelMatrix = Matrix4x4.CreateScale(0.01f);
            var viewMatrix = CameraObj.GetViewMatrix();

            Parallel.ForEach(E1M1Model.Meshes, mesh =>
            {
                if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, modelMatrix, viewMatrix, ProjectionMatrix))
                    return;

                Texture texture = null;

                if (mesh?.Material?.TexturePaths?.TryGetValue(TextureSlot.Diffuse, out var texturePath) == true)
                {
                    texture = CachedTextures.GetOrAdd(texturePath, SoftwareRenderer.Texture.LoadTexture);
                }

                Rasterizer.RenderMesh(
                    window,
                    mesh.Vertices.ToArray(),
                    mesh.Indices.ToArray(),
                    modelMatrix,
                    viewMatrix,
                    ProjectionMatrix,
                    VertexShader,
                    input => FragmentShader(input, texture),
                    Rasterizer.CullMode.Back);
                RenderedModels++;
            });
        }

        private Shaders.VertexOutput VertexShader(Shaders.VertexInput vertex, Matrix4x4 modelMatrix,
            Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            Vector4 worldPos = Vector4.Transform(new Vector4(vertex.Position, 1), modelMatrix);
            Vector4 viewPos = Vector4.Transform(worldPos, viewMatrix);
            Vector4 clipPos = Vector4.Transform(viewPos, projectionMatrix);

            Vector3 worldNormal = Vector3.Normalize(Vector3.TransformNormal(vertex.Normal, modelMatrix));

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPos,
                Data = { ["WorldNormal"] = worldNormal },
                TexCoord = vertex.TexCoord,
                Color = vertex.Color,
                Normal = vertex.Normal,
                Interpolate = true
            };
        }

        private Vector4 FragmentShader(Shaders.VertexOutput input, Texture texture = null)
        {
            Vector3 worldNormal = (Vector3)input.Data["WorldNormal"];
            float diffuse = MathF.Max(0.25f, Vector3.Dot(worldNormal, -LightDir));

            Vector4 textureColor = texture?.Sample(input.TexCoord) ?? Vector4.One;
            Vector4 baseColor = input.Color * textureColor;

            float depth = input.ClipPosition.Z;
            float fogFactor = Math.Clamp((FogEnd - depth) / (FogEnd - FogStart), 0f, 1f);
            fogFactor = fogFactor * fogFactor * (3f - 2f * fogFactor);

            Vector4 finalColor = Vector4.Lerp(FogColor, baseColor * (0.1f + 0.9f * diffuse) * LightColor, fogFactor);
            return new Vector4(finalColor.X, finalColor.Y, finalColor.Z, baseColor.W);
        }

        private static Vector3 EulerToDirection(Vector3 eulerDegrees)
        {
            Vector3 radians = eulerDegrees * (MathF.PI / 180f);

            Matrix4x4 rotation = Matrix4x4.CreateFromYawPitchRoll(
                radians.Y,
                radians.X,
                radians.Z
            );

            Vector3 forward = -Vector3.UnitZ;
            Vector3 direction = Vector3.Transform(forward, rotation);
            return Vector3.Normalize(direction);
        }

        public void Main(string[] args)
        {
            var window = new MainWindow("Software Renderer - E1M1")
            {
                RenderScale = 1/4f
            };

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
                    if(MouseLocked == false) return;
                    if (FirstMouse)
                    {
                        LastMousePosition = pos;
                        FirstMouse = false;
                    }

                    var delta = pos - LastMousePosition;
                    LastMousePosition = pos;

                    CameraObj.Yaw += delta.X * CameraObj.Sensitivity;
                    CameraObj.Pitch = Math.Clamp(CameraObj.Pitch - delta.Y * CameraObj.Sensitivity, -89f, 89f);
                };

            };

            Rasterizer.RenderDebugMode = Rasterizer.DebugMode.None;


            window.UpdateEvent += DeltaTime =>
            {
                if (MouseLocked == false)
                {
                    var io = ImGui.GetIO();
                    io.ConfigFlags |= ImGuiConfigFlags.DockingEnable | ImGuiConfigFlags.ViewportsEnable | ImGuiConfigFlags.DpiEnableScaleViewports;
                    
                    var Viewport = ImGui.GetMainViewport();
                    ImGui.SetNextWindowPos(Viewport.Pos);
                    ImGui.SetNextWindowSize(Viewport.Size);
                    ImGui.SetNextWindowViewport(Viewport.ID);

                    ImGui.DockSpaceOverViewport(Viewport.ID, Viewport, ImGuiDockNodeFlags.PassthruCentralNode);
                    //ImGui.SaveIniSettingsToDisk("./Layouts/DefaultLayout.ini");
                    ImGui.Begin("Renderer Controls", ImGuiWindowFlags.None);
                    if (ImGui.CollapsingHeader("Performance", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text($"FPS: {(int)(1 / DeltaTime)}");
                        ImGui.Text($"Frame Time: {DeltaTime * 1000:F2}ms");
                    }

                    if (ImGui.CollapsingHeader("Camera", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text("Camera Position:");

                        Vector3 camPos = CameraObj.Position;
                        if (ImGui.DragFloat3("Position", ref camPos, 0.1f))
                        {
                            CameraObj.Position = camPos;
                        }


                        ImGui.Text("Rotation:");
                        float Yaw = CameraObj.Yaw % 360;
                        if (ImGui.DragFloat("Yaw (°)", ref Yaw, 0.5f))
                        {
                            CameraObj.Yaw = Yaw;
                        }

                        float Pitch = CameraObj.Pitch;
                        if (ImGui.DragFloat("Pitch (°)", ref Pitch, 0.5f))
                        {
                            Pitch = Math.Clamp(Pitch, -89f, 89f);
                            CameraObj.Pitch = Pitch;
                        }

                        ImGui.SliderFloat("Move Speed", ref CameraObj.Speed, 0.1f, 50f);
                        ImGui.SliderFloat("Mouse Sensitivity", ref CameraObj.Sensitivity, 0.01f, 1f);
                        ImGui.SliderFloat("FOV", ref FOV, 1f, 179f);
                        if (ImGui.InputFloat("Near Clip", ref NearClip, 0.1f, 1.0f, "%.3f"))
                        {
                            NearClip = Math.Clamp(NearClip, 0.01f, 1000f);
                        }


                        if (ImGui.InputFloat("Far Clip", ref FarClip, 0.1f, 1.0f, "%.3f"))
                        {
                            FarClip = Math.Clamp(FarClip, 0.01f, 1000f);
                        }

                        if (FarClip <= NearClip)
                        {
                            FarClip = NearClip + 0.01f;
                        }
                        Vector3 frontVec = CameraObj.GetFront();
                        ImGui.Text($"Front: {frontVec.X:F2}, {frontVec.Y:F2}, {frontVec.Z:F2}");
                    }


                    if (ImGui.CollapsingHeader("Rendering", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text($"Render Scale: {(int)(window.RenderScale * 100)}%%");
                        if (ImGui.SliderFloat("##RenderScale", ref window.RenderScale, 0.1f, 1f))
                        {
                            window.UpdateRenderScale(window.RenderScale);
                        }

                        ImGui.Text("Clear Color:");
                        ImGui.ColorEdit3("Clear Color", ref ClearColor);

                        ImGui.Text("Debug Modes:");
                        int debugMode = (int)Rasterizer.RenderDebugMode;
                        ImGui.RadioButton("Normal", ref debugMode, (int)Rasterizer.DebugMode.None);
                        ImGui.SameLine();
                        ImGui.RadioButton("Wireframe", ref debugMode, (int)Rasterizer.DebugMode.Wireframe);
                        Rasterizer.RenderDebugMode = (Rasterizer.DebugMode)debugMode;

                        ImGui.Text("Fog Settings:");
                        ImGui.SliderFloat("Fog Start", ref FogStart, 1f, 100f);
                        ImGui.SliderFloat("Fog End", ref FogEnd, 10f, 500f);
                        ImGui.ColorEdit4("Fog Color", ref FogColor);

                        ImGui.Text("Light Settings:");

                        if (ImGui.DragFloat3("Light Rotation", ref LightEulerDegrees, 0.5f))
                        {
                            LightDir = EulerToDirection(LightEulerDegrees);
                        }

                        ImGui.ColorEdit4("Light Color", ref LightColor);

                        if (ImGui.CollapsingHeader("Scene Info", ImGuiTreeNodeFlags.DefaultOpen))
                        {
                            ImGui.Text($"Loaded Meshes: {E1M1Model?.Meshes?.Count ?? 0}");
                            ImGui.Text($"Rendered Meshes: {RenderedModels}");
                            ImGui.Text($"Cached Textures: {CachedTextures.Count}");
                            ImGui.Text($"Runtime: {Time:F2}s");
                            ImGui.Text($"Window Size: {window.WindowWidth}x{window.WindowHeight}");
                            ImGui.Text($"Render Size: {window.RenderWidth}x{window.RenderHeight}");
                        }
                    }

                    ImGui.End();
                    if (!LoadedLayout)
                    {
                        ImGui.LoadIniSettingsFromDisk("./Layouts/DefaultLayout.ini");
                        LoadedLayout = true;    
                    }

                }

                ProjectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
                    FOV * MathF.PI / 180,
                    (float)window.RenderWidth / window.RenderHeight,
                    NearClip,
                    FarClip);

                Time += (float)DeltaTime;

                Vector3 desiredPosition = CameraObj.Position;
                Vector3 front = CameraObj.GetFront();
                Vector3 right = Vector3.Normalize(Vector3.Cross(front, Vector3.UnitY));
                float moveStep = CameraObj.Speed * (float)DeltaTime;

                if (keyboard.IsKeyPressed(Key.W)) desiredPosition += front * moveStep;
                if (keyboard.IsKeyPressed(Key.S)) desiredPosition -= front * moveStep;
                if (keyboard.IsKeyPressed(Key.A)) desiredPosition -= right * moveStep;
                if (keyboard.IsKeyPressed(Key.D)) desiredPosition += right * moveStep;
                bool isEscapePressed = keyboard.IsKeyPressed(Key.Escape);

                if (isEscapePressed && !WasEscapePressed)
                {
                    MouseLocked = !MouseLocked;
                    mouse.Cursor.CursorMode = MouseLocked ? CursorMode.Disabled : CursorMode.Normal;
                    FirstMouse = true;
                }

                WasEscapePressed = isEscapePressed;

                CameraObj.Position = MoveWithSlide(CameraObj.Position, desiredPosition);

                window.ClearDepthBuffer();
                window.ClearColorBuffer(new Vector4(ClearColor.X, ClearColor.Y, ClearColor.Z, 1f));
                RenderE1M1(window);
                window.RenderFrame();
            };

            window.Run();
        }
    }
}