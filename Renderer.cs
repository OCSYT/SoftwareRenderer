using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Numerics;
using System.Text.Json;
using System.Threading.Tasks;
using Silk.NET.Input;
using Silk.NET.OpenGL.Extensions.ImGui;
using ImGuiNET;

namespace SoftwareRenderer
{
    public class Renderer
    {
        public class SceneModel
        {
            public List<Mesh> Meshes { get; init; }
        }

        private float Time;
        private Matrix4x4 ModelMatrix = Matrix4x4.CreateScale(0.01f);
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
        private CharacterController CharacterController;
        private bool IsRunning = false;
        private bool JumpRequested = false;
        private static readonly Camera CameraObj = new();
        private static Vector2 LastMousePosition;
        private static bool FirstMouse = true;
        private Matrix4x4 ProjectionMatrix;
        private List<Mesh> E1M1Model;
        private readonly ConcurrentDictionary<string, Texture> CachedTextures = new();
        private int RenderedModels;

        private List<Mesh> PlayerModel;


        private void RenderConnectedPlayers(MainWindow window, Matrix4x4 viewMatrix)
        {
            if (PlayerModel == null)
            {
                var PlayerModelTemp = Model.LoadModel("./Assets/gordon_freeman/scene.gltf");
                PlayerModel = PlayerModelTemp.Meshes;
            }
            

            if (PlayerModel == null || CharacterController == null) return;

            foreach (var Player in Players)
            {
                if(Player.Id == NetworkManager.ClientId) continue;
                Matrix4x4 PlayerModelMatrix =
                    Matrix4x4.CreateScale(CharacterController.Height/2) *
                    Matrix4x4.CreateFromQuaternion(Player.Rotation * Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI)) *
                    Matrix4x4.CreateTranslation(Player.Position - (Vector3.UnitY * CharacterController.Height/2));


                foreach (var Mesh in PlayerModel)
                {
                    Texture texture = null;

                    if (Mesh?.Material?.TexturePaths?.TryGetValue(TextureSlot.Diffuse, out var texturePath) == true)
                    {
                        texture = CachedTextures.GetOrAdd(texturePath, SoftwareRenderer.Texture.LoadTexture);
                    }

                    if (!FrustumCuller.IsSphereInFrustum(Mesh.SphereBounds, PlayerModelMatrix, viewMatrix,
                            ProjectionMatrix))
                        continue;

                    Rasterizer.RenderMesh(
                        window,
                        Mesh.Vertices.ToArray(),
                        Mesh.Indices.ToArray(),
                        PlayerModelMatrix,
                        viewMatrix,
                        ProjectionMatrix,
                        VertexShader,
                        input => FragmentShader(input, texture), // no texture for simple cube
                        Rasterizer.CullMode.Back);
                }
            }
        }

        private void InitPlayer()
        {
            CharacterController = new CharacterController(new Vector3(-10.610103f, 0.44594023f, -36.2275f), [E1M1Model],
                [ModelMatrix]);
            CameraObj.Rotation = Quaternion.CreateFromYawPitchRoll((float)(180 * (Math.PI / 180)), 0, 0);
        }

        private void RenderE1M1(MainWindow window)
        {
            RenderedModels = 0;
            if (E1M1Model == null)
            {
                var result = Model.LoadModel("./Assets/e1m1/doom_E1M1.obj");
                E1M1Model = result.Meshes;
                InitPlayer();
            }

            var viewMatrix = CameraObj.GetViewMatrix();

            object renderedModelsLock = new();

            Parallel.ForEach(E1M1Model, mesh =>
            {
                if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, ModelMatrix, viewMatrix, ProjectionMatrix))
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
                    ModelMatrix,
                    viewMatrix,
                    ProjectionMatrix,
                    VertexShader,
                    input => FragmentShader(input, texture),
                    Rasterizer.CullMode.Back);

                lock (renderedModelsLock)
                {
                    RenderedModels++;
                }
            });
        }

        private Shaders.VertexOutput VertexShader(Shaders.VertexInput vertex, Matrix4x4 ModelMatrix,
            Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            Vector4 worldPos = Vector4.Transform(new Vector4(vertex.Position, 1), ModelMatrix);
            Vector4 viewPos = Vector4.Transform(worldPos, viewMatrix);
            Vector4 clipPos = Vector4.Transform(viewPos, projectionMatrix);

            Vector3 worldNormal = Vector3.Normalize(Vector3.TransformNormal(vertex.Normal, ModelMatrix));

            return new Shaders.VertexOutput
            {
                ClipPosition = clipPos,
                Data = { ["WorldNormal"] = worldNormal },
                TexCoord = vertex.TexCoord,
                Color = vertex.Color,
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

        private void RenderPlayerNametags(MainWindow window, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            if (Players.Count == 0) return;

            Matrix4x4 viewProjection = viewMatrix * projectionMatrix;

            foreach (var player in Players)
            {
                Vector3 headPosition = player.Position + new Vector3(0, CharacterController.Height/2, 0);
                Vector4 worldPos = new Vector4(headPosition, 1.0f);
                Vector4 clipPos = Vector4.Transform(worldPos, viewProjection);

                if (clipPos.W <= 0.001f) continue;

                Vector3 ndc = new Vector3(clipPos.X, clipPos.Y, clipPos.Z) / clipPos.W;

                if (ndc.Z < 0 || ndc.Z > 1) continue;

                Vector2 screenPos = new Vector2(
                    (ndc.X + 1f) * 0.5f * window.WindowWidth,
                    (1f - ndc.Y) * 0.5f * window.WindowHeight);

                if (screenPos.X < 0 || screenPos.X > window.WindowWidth ||
                    screenPos.Y < 0 || screenPos.Y > window.WindowHeight)
                {
                    continue;
                }

                ImGui.SetNextWindowPos(screenPos, ImGuiCond.Always, new Vector2(0.5f, 1f));
                ImGui.SetNextWindowBgAlpha(0.35f);

                string hiddenLabel = $"###Nametag_{player.Id}";

                if (ImGui.Begin(hiddenLabel,
                        ImGuiWindowFlags.NoDecoration |
                        ImGuiWindowFlags.AlwaysAutoResize |
                        ImGuiWindowFlags.NoSavedSettings |
                        ImGuiWindowFlags.NoFocusOnAppearing |
                        ImGuiWindowFlags.NoNav))
                {
                    ImGui.Text("Player " + player.Id.ToString());
                    ImGui.End();
                }
            }
        }

        private string ChatInput = "";
        private List<string> ChatMessages = new List<string>();
        private bool ChatWindowOpen = true;
        private bool ChatInputActive = false;

        private unsafe int InputTextCallback(ImGuiInputTextCallbackData* data)
        {
            return 0;
        }
        
        private bool ScrollToBottom = false;


        private void RenderChatWindow()
        {
            var viewport = ImGui.GetMainViewport();
            Vector2 windowSize = new Vector2(400, 300);
            Vector2 windowPos = new Vector2(viewport.Pos.X + viewport.Size.X - windowSize.X - 10, viewport.Pos.Y + 10);

            ImGui.SetNextWindowPos(windowPos, ImGuiCond.Always);
            ImGui.SetNextWindowSize(windowSize, ImGuiCond.Always);

            if (ImGui.Begin("Chat", ref ChatWindowOpen,
                    ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoDocking))
            {
                unsafe
                {
                    ChatWindowOpen = true;

                    ImGui.BeginChild("ChatContent", new Vector2(0, -ImGui.GetFrameHeightWithSpacing()));

                    for (int i = 0; i < ChatMessages.Count; i++)
                    {
                        if (!string.IsNullOrEmpty(ChatMessages[i]))
                        {
                            ImGui.PushTextWrapPos(0.0f);
                            ImGui.TextUnformatted(ChatMessages[i]);
                            ImGui.PopTextWrapPos();
                        }
                    }

                    float inputBoxHeight = ImGui.GetFrameHeightWithSpacing();
                    float scrollMax = ImGui.GetScrollMaxY();
                    float scrollY = ImGui.GetScrollY();
                    float lastMessageHeight = 0f;

                    if (ChatMessages.Count > 0 && !string.IsNullOrEmpty(ChatMessages[ChatMessages.Count - 1]))
                    {
                        lastMessageHeight =
                            ImGui.CalcTextSize(ChatMessages[ChatMessages.Count - 1], false, ImGui.GetContentRegionAvail().X)
                                .Y + ImGui.GetStyle().ItemSpacing.Y;
                    }

                    if (ScrollToBottom || scrollY >= scrollMax - (lastMessageHeight + inputBoxHeight))
                    {
                        ImGui.SetScrollY(scrollMax);
                        ScrollToBottom = false;
                    }

                    ImGui.EndChild();
                    ImGui.Separator();

                    ImGuiInputTextFlags inputTextFlags = ImGuiInputTextFlags.EnterReturnsTrue |
                                                         ImGuiInputTextFlags.CallbackCompletion |
                                                         ImGuiInputTextFlags.CallbackHistory;
                    bool reclaimFocus = false;

                    if (ImGui.InputText("##ChatInput", ref ChatInput, 256, inputTextFlags, InputTextCallback))
                    {
                        if (!string.IsNullOrWhiteSpace(ChatInput))
                        {
                            if (NetworkManager.IsConnected)
                            {
                                NetworkManager.SendRPC("ChatMessage",
                                    new string[] { $"Player {NetworkManager.ClientId}", ChatInput });
                            }
                            else
                            {
                                ChatMessages.Add($"You: {ChatInput}");
                            }

                            ChatInput = "";
                            ScrollToBottom = true;
                        }

                        reclaimFocus = true;
                    }

                    ImGui.SetItemDefaultFocus();

                    if (reclaimFocus || (ChatInputActive && !ImGui.IsItemActive()))
                    {
                        ImGui.SetKeyboardFocusHere(-1);
                    }

                    ChatInputActive = ImGui.IsItemActive();
                }
            }
            ImGui.End();
        }

        private static Vector3 EulerToDirection(Vector3 eulerDegrees)
        {
            Vector3 radians = eulerDegrees * (MathF.PI / 180f);
            Matrix4x4 rotation = Matrix4x4.CreateFromYawPitchRoll(radians.Y, radians.X, radians.Z);
            Vector3 forward = -Vector3.UnitZ;
            Vector3 direction = Vector3.Transform(forward, rotation);
            return Vector3.Normalize(direction);
        }

        private Networking NetworkManager = null;

        class ConnectedPlayer
        {
            public int Id = 0;
            public Vector3 Position = Vector3.Zero;
            public Quaternion Rotation = Quaternion.Identity;
        }

        private List<ConnectedPlayer> Players = new List<ConnectedPlayer>();

        async void HandleConnection(string IP = "127.0.0.1")
        {
            NetworkManager = new Networking();
            bool Success = await NetworkManager.Connect(IP);
            if (!Success)
            {
                System.Environment.Exit(1);
            }
            if (NetworkManager.IsConnected)
            {
                NetworkManager.OnReceiveRPC += (Method, Params) =>
                {
                    if (Method == "ConnectedPlayer")
                    {
                        Players.Add(new ConnectedPlayer { Id = int.Parse(Params[0]) });
                    }

                    if (Method == "Update")
                    {
                        if (Params.Length >= 4
                            && int.TryParse(Params[0], out int playerId)
                            && float.TryParse(Params[1], out float posX)
                            && float.TryParse(Params[2], out float posY)
                            && float.TryParse(Params[3], out float posZ)
                            && float.TryParse(Params[4], out float rotX)
                            && float.TryParse(Params[5], out float rotY)
                            && float.TryParse(Params[6], out float rotZ)
                            && float.TryParse(Params[7], out float rotW))
                        {
                            var player = Players.Find(p => p.Id == playerId);
                            if (player != null)
                            {
                                player.Position = new Vector3(posX, posY, posZ);
                                player.Rotation = new Quaternion(rotX, rotY, rotZ, rotW);
                            }
                        }
                    }

                    if (Method == "DisconnectedPlayer")
                    {
                        if (Params.Length >= 1 && int.TryParse(Params[0], out int disconnectedId))
                        {
                            var player = Players.Find(p => p.Id == disconnectedId);
                            if (player != null)
                            {
                                Players.Remove(player);
                                Console.WriteLine($"Player disconnected: {disconnectedId}");
                            }
                        }
                    }

                    if (Method == "ChatMessage")
                    {
                        if (Params.Length >= 2)
                        {
                            string playerName = Params[0];
                            string message = Params[1];
                            ChatMessages.Add($"{playerName}: {message}");
                        }
                    }
                };
                NetworkManager.SendRPC("ConnectedPlayer", [NetworkManager.ClientId.ToString()], BufferRPC: true);
            }
        }

        public void Main(string[] args)
        {

            string IPAddress = "127.0.0.1"; // Default to localhost

            if (args.Length > 0)
            {
                IPAddress = args[0];
            }

            HandleConnection(IPAddress);

            var window = new MainWindow("Software Renderer - E1M1")
            {
                RenderScale = 1 / 4f
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
                    if (!MouseLocked) return;
                    if (FirstMouse)
                    {
                        LastMousePosition = pos;
                        FirstMouse = false;
                        return;
                    }

                    var delta = pos - LastMousePosition;
                    LastMousePosition = pos;

                    var euler = CameraObj.GetEulerAngles();
                    euler.Y -= delta.X * CameraObj.Sensitivity;
                    euler.X -= delta.Y * CameraObj.Sensitivity;
                    euler.X = Math.Clamp(euler.X, -89f, 89f);

                    CameraObj.Rotation = Quaternion.CreateFromYawPitchRoll(
                        euler.Y * (MathF.PI / 180f),
                        euler.X * (MathF.PI / 180f),
                        euler.Z * (MathF.PI / 180f));
                };
            };

            Rasterizer.RenderDebugMode = Rasterizer.DebugMode.None;

            window.UpdateEvent += DeltaTime =>
            {
                if (NetworkManager.IsConnected)
                {
                    if (CharacterController != null)
                    {
                        Vector3 EulerDegrees = CameraObj.GetEulerAngles();
                        Quaternion Rotation = Quaternion.CreateFromYawPitchRoll(EulerDegrees.Y * MathF.PI / 180f, 0, 0);
                        NetworkManager.SendRPC("Update", new string[]
                        {
                            NetworkManager.ClientId.ToString(),
                            CharacterController.Position.X.ToString(),
                            CharacterController.Position.Y.ToString(),
                            CharacterController.Position.Z.ToString(),
                            Rotation.X.ToString(),
                            Rotation.Y.ToString(),
                            Rotation.Z.ToString(),
                            Rotation.W.ToString()
                        });
                    }
                }


                var io = ImGui.GetIO();
                io.ConfigFlags |= ImGuiConfigFlags.DockingEnable | ImGuiConfigFlags.ViewportsEnable |
                                  ImGuiConfigFlags.DpiEnableScaleViewports;

                var Viewport = ImGui.GetMainViewport();
                ImGui.SetNextWindowPos(Viewport.Pos);
                ImGui.SetNextWindowSize(Viewport.Size);
                ImGui.SetNextWindowViewport(Viewport.ID);

                ImGui.DockSpaceOverViewport(Viewport.ID, Viewport, ImGuiDockNodeFlags.PassthruCentralNode);

                RenderChatWindow();

                if (!MouseLocked)
                {
                    ImGui.Begin("Renderer Controls", ImGuiWindowFlags.None);

                    if (ImGui.CollapsingHeader("Performance", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text($"FPS: {(int)(1 / DeltaTime)}");
                        ImGui.Text($"Frame Time: {DeltaTime * 1000:F2}ms");
                        ImGui.Text($"Rendered Meshes: {RenderedModels}");
                        ImGui.Text($"Cached Textures: {CachedTextures.Count}");
                    }


                    if (ImGui.CollapsingHeader("Scene Info", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text($"Loaded Meshes: {E1M1Model?.Count ?? 0}");
                        ImGui.Text($"Runtime: {Time:F2}s");
                        ImGui.Text($"Window Size: {window.WindowWidth}x{window.WindowHeight}");
                        ImGui.Text($"Render Size: {window.RenderWidth}x{window.RenderHeight}");
                    }

                    if (ImGui.CollapsingHeader("Multiplayer", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text("Connected Players:");
                        foreach (var player in Players)
                        {
                            ImGui.Text("Player ID: " + player.Id);
                            ImGui.Text("Player Position: " + player.Position.ToString());
                            ImGui.Text("Player Rotation: " + player.Rotation.ToString());
                        }
                    }

                    if (ImGui.CollapsingHeader("Camera", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        ImGui.Text("Camera Position:");
                        Vector3 camPos = CameraObj.Position;
                        if (ImGui.DragFloat3("Position", ref camPos, 0.1f))
                        {
                            CameraObj.Position = camPos;
                            if (CharacterController != null)
                                CharacterController.Position = camPos;
                        }

                        ImGui.Text("Rotation:");
                        Vector3 eulerDegrees = CameraObj.GetEulerAngles();
                        float pitch = eulerDegrees.X;
                        float yaw = eulerDegrees.Y;
                        float roll = eulerDegrees.Z;
                        bool updated = false;
                        if (ImGui.DragFloat("Pitch (°)", ref pitch, 0.5f))
                        {
                            pitch = Math.Clamp(pitch, -89f, 89f);
                            updated = true;
                        }

                        if (ImGui.DragFloat("Yaw (°)", ref yaw, 0.5f))
                        {
                            yaw = yaw % 360f;
                            updated = true;
                        }

                        if (ImGui.DragFloat("Roll (°)", ref roll, 0.5f))
                        {
                            updated = true;
                        }

                        if (updated)
                        {
                            Quaternion newRotation = Quaternion.CreateFromYawPitchRoll(
                                yaw * (MathF.PI / 180f),
                                pitch * (MathF.PI / 180f),
                                roll * (MathF.PI / 180f));
                            CameraObj.Rotation = newRotation;
                        }

                        ImGui.SliderFloat("Mouse Sensitivity", ref CameraObj.Sensitivity, 0.01f, 1f);
                        ImGui.SliderFloat("FOV", ref FOV, 1f, 179f);
                        Vector3 frontVec = CameraObj.GetFront();
                        ImGui.Text($"Front: {frontVec.X:F2}, {frontVec.Y:F2}, {frontVec.Z:F2}");
                    }

                    if (CharacterController != null &&
                        ImGui.CollapsingHeader("Character Controller", ImGuiTreeNodeFlags.DefaultOpen))
                    {
                        float moveSpeed = CharacterController.MoveSpeed;
                        if (ImGui.DragFloat("Move Speed", ref moveSpeed, 0.1f, 1f, 20f))
                            CharacterController.MoveSpeed = moveSpeed;

                        float maxAirSpeed = CharacterController.MaxAirSpeed;
                        if (ImGui.DragFloat("Max Air Speed", ref maxAirSpeed, 0.1f, 1f, 20f))
                            CharacterController.MaxAirSpeed = maxAirSpeed;

                        float jumpForce = CharacterController.JumpForce;
                        if (ImGui.DragFloat("Jump Force", ref jumpForce, 0.1f, 1f, 20f))
                            CharacterController.JumpForce = jumpForce;

                        float radius = CharacterController.Radius;
                        if (ImGui.DragFloat("Radius", ref radius, 0.1f, 1f, 20f))
                            CharacterController.Radius = radius;

                        float height = CharacterController.Height;
                        if (ImGui.DragFloat("Height", ref height, 0.1f, 0.5f, 3f))
                            CharacterController.Height = height;

                        float groundaccel = CharacterController.GroundAcceleration;
                        if (ImGui.DragFloat("Ground Acceleration", ref groundaccel, 0.1f, 1f, 20f))
                            CharacterController.GroundAcceleration = groundaccel;

                        float airaccel = CharacterController.AirAcceleration;
                        if (ImGui.DragFloat("Air Acceleration", ref airaccel, 0.1f, 1f, 20f))
                            CharacterController.AirAcceleration = airaccel;

                        float groundFriction = CharacterController.GroundFriction;
                        if (ImGui.DragFloat("Ground Friction", ref groundFriction, 0.1f, 1f, 20f))
                            CharacterController.GroundFriction = groundFriction;

                        float airControl = CharacterController.AirControl;
                        if (ImGui.DragFloat("Air Control", ref airControl, 0.01f, 0f, 1f))
                            CharacterController.AirControl = airControl;

                        float stepSize = CharacterController.StepSize;
                        if (ImGui.DragFloat("Step Size", ref stepSize, 0.1f, 0.5f, 3f))
                            CharacterController.StepSize = stepSize;

                        Vector3 gravity = CharacterController.Gravity;
                        if (ImGui.DragFloat3("Gravity", ref gravity, 0.1f, -20f, 20f))
                            CharacterController.Gravity = gravity;

                        ImGui.Text(
                            $"Velocity: {CharacterController.Velocity.X:F2}, {CharacterController.Velocity.Y:F2}, {CharacterController.Velocity.Z:F2}");
                        ImGui.Text($"Is Grounded: {CharacterController.IsGrounded}");
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
                    }

                    ImGui.End();
                }

                if (!LoadedLayout)
                {
                    ImGui.LoadIniSettingsFromDisk("./Layouts/DefaultLayout.ini");
                    LoadedLayout = true;
                }

                ProjectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
                    FOV * MathF.PI / 180,
                    (float)window.RenderWidth / window.RenderHeight,
                    NearClip,
                    FarClip);

                Time += (float)DeltaTime;

                if (CharacterController != null)
                {
                    Vector3 moveInput = Vector3.Zero;
                    Vector3 front = CameraObj.GetFront();
                    Vector3 right = Vector3.Normalize(Vector3.Cross(front, Vector3.UnitY));

                    front.Y = 0;
                    front = Vector3.Normalize(front);
                    right.Y = 0;
                    right = Vector3.Normalize(right);

                    if (!ChatInputActive)
                    {
                        if (keyboard.IsKeyPressed(Key.W)) moveInput += front;
                        if (keyboard.IsKeyPressed(Key.S)) moveInput -= front;
                        if (keyboard.IsKeyPressed(Key.A)) moveInput -= right;
                        if (keyboard.IsKeyPressed(Key.D)) moveInput += right;
                        JumpRequested = keyboard.IsKeyPressed(Key.Space);
                    }

                    CharacterController.Update((float)DeltaTime, moveInput, JumpRequested);

                    CameraObj.Position = CharacterController.Position;
                }

                bool isEscapePressed = keyboard.IsKeyPressed(Key.Escape);
                if (isEscapePressed && !WasEscapePressed)
                {
                    MouseLocked = !MouseLocked;
                    mouse.Cursor.CursorMode = MouseLocked ? CursorMode.Disabled : CursorMode.Normal;
                    FirstMouse = true;
                }

                WasEscapePressed = isEscapePressed;

                window.ClearDepthBuffer();
                window.ClearColorBuffer(new Vector4(ClearColor.X, ClearColor.Y, ClearColor.Z, 1f));
                RenderE1M1(window);
                RenderConnectedPlayers(window, CameraObj.GetViewMatrix());
                RenderPlayerNametags(window, CameraObj.GetViewMatrix(), ProjectionMatrix);
                window.RenderFrame();
            };
            window.CloseEvent += () =>
            {
                if (NetworkManager.IsConnected)
                {
                    NetworkManager.SendRPC("DisconnectedPlayer", new string[] { NetworkManager.ClientId.ToString() });
                }

                NetworkManager.Close();
            };

            window.Run();
        }
    }
}