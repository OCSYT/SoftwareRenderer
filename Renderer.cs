using System;
using System.Collections.Generic;
using System.Numerics;
using Silk.NET.Input;

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
            public float Speed = 5f;
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

        private static (List<Mesh> Meshes, List<Light> Lights) LoadedModel;
        private static readonly List<Shaders.VertexInput[]> MeshVertexInputs = new();
        private static readonly Camera CameraObj = new();
        private static Vector2 LastMousePosition;
        private static bool FirstMouse = true;
        private Matrix4x4 ProjectionMatrix;

        private static void InitializeVertexInputs()
        {
            MeshVertexInputs.Clear();

            foreach (var mesh in LoadedModel.Meshes)
            {
                var vertexInputs = new Shaders.VertexInput[mesh.Triangles.Count * 3];
                int vIndex = 0;

                foreach (var tri in mesh.Triangles)
                {
                    vertexInputs[vIndex++] = new Shaders.VertexInput
                    {
                        Position = mesh.Vertices[tri.Index0].Position,
                        TexCoord = mesh.Vertices[tri.Index0].TexCoord,
                        Normal = mesh.Vertices[tri.Index0].Normal,
                        Color = tri.Color
                    };
                    vertexInputs[vIndex++] = new Shaders.VertexInput
                    {
                        Position = mesh.Vertices[tri.Index1].Position,
                        TexCoord = mesh.Vertices[tri.Index1].TexCoord,
                        Normal = mesh.Vertices[tri.Index1].Normal,
                        Color = tri.Color
                    };
                    vertexInputs[vIndex++] = new Shaders.VertexInput
                    {
                        Position = mesh.Vertices[tri.Index2].Position,
                        TexCoord = mesh.Vertices[tri.Index2].TexCoord,
                        Normal = mesh.Vertices[tri.Index2].Normal,
                        Color = tri.Color
                    };
                }

                MeshVertexInputs.Add(vertexInputs);
            }
        }

        private Shaders.VertexOutput VertexShader(
            Shaders.VertexInput input,
            Matrix4x4 model,
            Matrix4x4 view,
            Matrix4x4 projection)
        {
            Matrix4x4 modelView = model * view;
            Matrix4x4 mvp = modelView * projection;

            Vector4 clipPosition = Vector4.Transform(new Vector4(input.Position, 1f), mvp);

            Matrix4x4 normalMatrix = Matrix4x4.Invert(model, out var modelInverse) 
                ? Matrix4x4.Transpose(modelInverse) 
                : Matrix4x4.Identity;
                
            Vector3 normalWorld = Vector3.Normalize(Vector3.TransformNormal(input.Normal, normalMatrix));

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPosition,
                Color = input.Color,
                TexCoord = input.TexCoord,
                Normal = input.Normal,
                ScreenCoords = Vector2.Zero,
                Data = { ["NormalWorld"] = normalWorld }
            };
        }

        private static readonly float[,] BayerMatrix4x4 = 
        {
            { 0.0f/16,  8.0f/16,  2.0f/16, 10.0f/16 },
            { 12.0f/16, 4.0f/16, 14.0f/16,  6.0f/16 },
            { 3.0f/16, 11.0f/16,  1.0f/16,  9.0f/16 },
            { 15.0f/16,  7.0f/16, 13.0f/16,  5.0f/16 }
        };

        private Vector4? FragmentShader(
            Shaders.VertexOutput input,
            Texture texture,
            Vector3 color,
            int screenWidth,
            int screenHeight)
        {
            var lightDirection = Vector3.Normalize(new Vector3(100, 100, -100));
            const float ambientLight = 0.1f;

            input.Data.TryGetValue("NormalWorld", out var normalObj);
            var normalWorld = normalObj is Vector3 n ? n : Vector3.Zero;

            float lighting = MathF.Max(ambientLight, Vector3.Dot(normalWorld, lightDirection));
            Vector4 texColor = texture?.Sample(input.TexCoord) ?? Vector4.One;

            Vector3 shadedColor = new Vector3(texColor.X, texColor.Y, texColor.Z)
                                * lighting
                                * new Vector3(input.Color.X, input.Color.Y, input.Color.Z)
                                * color;

            int x = Math.Clamp((int)(input.ScreenCoords.X * screenWidth), 0, screenWidth - 1);
            int y = Math.Clamp((int)(input.ScreenCoords.Y * screenHeight), 0, screenHeight - 1);

            int bx = x % 4;
            int by = y % 4;
            float threshold = BayerMatrix4x4[by, bx];

            if (texColor.W < threshold)
                return null;

            return new Vector4(shadedColor, 1.0f);
        }

        private void PostProcess(MainWindow window)
        {
            static Vector3 ApplyAces(Vector3 color)
            {
                Vector3 a = color * (2.51f * color + new Vector3(0.03f));
                Vector3 b = color * (2.43f * color + new Vector3(0.59f)) + new Vector3(0.14f);
                return Vector3.Clamp(a / b, Vector3.Zero, Vector3.One);
            }

            static float ApplyGamma(float linear)
            {
                linear = float.Clamp(linear, 0f, 1f);
                return linear <= 0.0031308f 
                    ? linear * 12.92f 
                    : 1.055f * MathF.Pow(linear, 1f / 2.4f) - 0.055f;
            }

            int width = window.RenderWidth;
            int height = window.RenderHeight;

            Parallel.For(0, height, y =>
            {
                for (int x = 0; x < width; x++)
                {
                    Vector4 color = window.GetPixel(x, y);
                    Vector3 aces = ApplyAces(new Vector3(color.X, color.Y, color.Z));
                    Vector3 gamma = new Vector3(
                        ApplyGamma(aces.X),
                        ApplyGamma(aces.Y),
                        ApplyGamma(aces.Z)
                    );

                    window.SetPixel(x, y, new Vector4(gamma, color.W));
                }
            });
        }

        public void Main(string[] args)
        {
            var window = new MainWindow("Software Renderer Demo");
            window.RenderScale = 0.125f;

            IInputContext inputContext = null;
            IKeyboard keyboard = null;
            IMouse mouse = null;

            window.StartEvent += () =>
            {
                LoadedModel = Model.LoadModel("./Assets/models/sponza.obj");
                InitializeVertexInputs();

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

            window.UpdateEvent += deltaTime =>
            {
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

                var view = CameraObj.GetViewMatrix();
                var modelMatrix = Matrix4x4.CreateScale(.01f) *
                                 Matrix4x4.CreateRotationY(0) *
                                 Matrix4x4.CreateTranslation(Vector3.Zero);

                int count = 0;
                Parallel.For(0, LoadedModel.Meshes.Count, i =>
                {
                    var mesh = LoadedModel.Meshes[i];
                    if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, modelMatrix, view, ProjectionMatrix))
                        return;

                    count++;
                    var vertexInputs = MeshVertexInputs[i];

                    Texture texture = null;
                    Vector3 color = Vector3.One;

                    if (mesh.TextureIndex != -1)
                    {
                        texture = Model.LoadedTextures[mesh.TextureIndex];
                    }

                    Rasterizer.RenderMesh(
                        window,
                        vertexInputs,
                        modelMatrix,
                        view,
                        ProjectionMatrix,
                        VertexShader,
                        input => FragmentShader(input, texture, color, window.RenderWidth, window.RenderHeight),
                        Rasterizer.CullMode.Back,
                        Rasterizer.DepthTest.LessEqual,
                        Rasterizer.BlendMode.Alpha);
                });

                Console.WriteLine($"Meshes on screen: {count}");
                PostProcess(window);
                window.RenderFrame();
            };

            window.Run();
        }
    }
}