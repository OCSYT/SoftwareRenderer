using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using Silk.NET.Input;
using Silk.NET.OpenGL.Extensions.ImGui;
using ImGuiNET;
using SDL2;

namespace SoftwareRenderer
{
    public class Renderer : IDisposable
    {
        private readonly ConcurrentDictionary<string, Texture> CachedTextures = new();
        private readonly Camera Camera = new();
        private readonly List<ConnectedPlayer> Players = new();
        private readonly List<string> ChatMessages = new();

        private MainWindow Window;
        private IInputContext InputContext;
        private IKeyboard Keyboard;
        private IMouse Mouse;
        private CharacterController CharacterController;
        private Networking NetworkManager;
        private List<Mesh> Dust2Model;
        private List<Mesh> PlayerModel;
        private List<Mesh> GunModel;

        private Vector3 SpawnPosition = new Vector3(-16.4f, 1.5f, 6.5f);
        private Vector3 SpawnPosition2 = new Vector3(-16.5f, 0.6f, -23);
        private Matrix4x4 ModelMatrix = Matrix4x4.CreateScale(0.5f);
        private Matrix4x4 GunMatrix = Matrix4x4.CreateScale(0.01f);
        private Matrix4x4 ProjectionMatrix;
        private Vector2 LastMousePosition;
        private Quaternion WeaponSway = Quaternion.Identity;
        private Quaternion Recoil = Quaternion.Identity;
        private float Time;
        private float FogStart = 1f;
        private float FogEnd = 25f;
        private Vector4 FogColor = new Vector4(1f, 0.62f, 0.5f, 1f);
        private Vector3 LightEulerDegrees = new(-45, -45, 0);
        private Vector3 LightDirection = EulerToDirection(new(-45, -45, 0));
        private Vector4 LightColor = Vector4.One;
        private Vector3 ClearColor = new Vector3(0.9137f, 0.7098f, 0.6588f);
        private float FieldOfView = 90f;
        private int RenderedModels;
        private bool IsMouseLocked = true;
        private bool IsFirstMouse = true;
        private bool WasEscapePressed;
        private bool WasVPressed;
        private bool IsLayoutLoaded;
        private bool IsJumpRequested;
        private string ChatInput = "";
        private bool IsChatWindowOpen = true;
        private bool IsChatInputActive;
        private bool ScrollToBottom;
        private float LastShotTime;
        private float ShotCooldown = 0.1f;
        private string PlayerName = "Player"; // Default name if file reading fails
        
        private class ConnectedPlayer
        {
            public int Id { get; set; }
            public Vector3 Position { get; set; } = Vector3.Zero;
            public Vector3 LocalPosition { get; set; } = Vector3.Zero;
            public Quaternion Rotation { get; set; } = Quaternion.Identity;
            public float Health { get; set; } = 100f;
            public string Name { get; set; } = "Player";
        }

        public void Main(string[] args)
        {
            Window = new MainWindow("Software Renderer - Dust2") { RenderScale = 0.25f };
            Task.Run(async () =>
            {
                string ipAddress = args.Length > 0 ? args[0] : "127.0.0.1";
                await InitializeNetworkingAsync(ipAddress);
                SetupWindowEvents();
                LoadPlayerNameFromFile();
                NetworkManager.SendRPC("ConnectedPlayer", new[] { NetworkManager.ClientId.ToString(), PlayerName }, BufferRPC: true);
            }).Wait();
            Window.Run();
        }

        private void LoadPlayerNameFromFile(string filePath = "./Playername.txt")
        {
            try
            {
                if (File.Exists(filePath))
                {
                    PlayerName = File.ReadAllText(filePath).Trim();
                    if (string.IsNullOrWhiteSpace(PlayerName))
                    {
                        PlayerName = $"Player {NetworkManager.ClientId}";
                        Console.WriteLine($"Warning: {filePath} is empty. Using default name: {PlayerName}");
                    }
                }
                else
                {
                    PlayerName = $"Player {NetworkManager.ClientId}";
                    Console.WriteLine($"Warning: {filePath} not found. Using default name: {PlayerName}");
                }
            }
            catch (Exception ex)
            {
                PlayerName = $"Player {NetworkManager.ClientId}";
                Console.WriteLine($"Error reading {filePath}: {ex.Message}. Using default name: {PlayerName}");
            }
        }
        
        private async Task InitializeNetworkingAsync(string ipAddress)
        {
            NetworkManager = new Networking();
            if (!await NetworkManager.Connect(ipAddress))
            {
                Environment.Exit(1);
            }
            RegisterNetworkCallbacks();
        }

        private void SetupWindowEvents()
        {
            Window.StartEvent += InitializeInput;
            Window.UpdateEvent += Update;
            Window.CloseEvent += Dispose;
        }

        private void InitializeInput()
        {
            InputContext = Window.InputContext;
            Keyboard = InputContext.Keyboards[0];
            Mouse = InputContext.Mice[0];
            Mouse.Cursor.CursorMode = CursorMode.Disabled;

            Mouse.MouseMove += HandleMouseMovement;
            Mouse.MouseDown += HandleMouseClick;
        }

        private void HandleMouseMovement(IMouse mouse, Vector2 position)
        {
            if (!IsMouseLocked) return;

            if (IsFirstMouse)
            {
                LastMousePosition = position;
                IsFirstMouse = false;
                return;
            }

            var delta = position - LastMousePosition;
            LastMousePosition = position;

            var euler = Camera.GetEulerAngles();
            euler.Y -= delta.X * Camera.Sensitivity;
            euler.X = Math.Clamp(euler.X - delta.Y * Camera.Sensitivity, -89f, 89f);
            Camera.Rotation = Quaternion.CreateFromYawPitchRoll(
                euler.Y * MathF.PI / 180f,
                euler.X * MathF.PI / 180f,
                euler.Z * MathF.PI / 180f);
        }

        private void HandleMouseClick(IMouse mouse, MouseButton button)
        {
            if (button == MouseButton.Left && IsMouseLocked && Time - LastShotTime >= ShotCooldown)
            {
                Shoot();
                LastShotTime = Time;
            }
        }

        private void Shoot()
        {
            var rayOrigin = Camera.Position;
            var rayDirection = Camera.GetFront();
            const float maxDistance = 100f;
            var closestHit = new { Distance = float.MaxValue, Player = (ConnectedPlayer)null, Point = Vector3.Zero, Normal = Vector3.Zero, IsLevel = false };

            Parallel.ForEach(Players, player =>
            {
                if (player.Id == NetworkManager.ClientId) return;

                var playerMatrix = CreatePlayerMatrix(player);
                foreach (var mesh in PlayerModel)
                {
                    if (Physics.Raycast(rayOrigin, rayDirection, mesh.Vertices.ToArray(), mesh.Indices.ToArray(), playerMatrix,
                        out float distance, out Vector3 point, out Vector3 normal, RaycastFaceMask.IgnoreBackfaces) && distance < closestHit.Distance)
                    {
                        lock (new object())
                        {
                            if (distance < closestHit.Distance)
                            {
                                closestHit = new { Distance = distance, Player = player, Point = point, Normal = normal, IsLevel = false };
                            }
                        }
                    }
                }
            });

            if (Dust2Model != null)
            {
                Parallel.ForEach(Dust2Model, mesh =>
                {
                    if (Physics.Raycast(rayOrigin, rayDirection, mesh.Vertices.ToArray(), mesh.Indices.ToArray(), ModelMatrix,
                        out float distance, out Vector3 point, out Vector3 normal, RaycastFaceMask.IgnoreBackfaces) && distance < closestHit.Distance)
                    {
                        lock (new object())
                        {
                            if (distance < closestHit.Distance)
                            {
                                closestHit = new { Distance = distance, Player = (ConnectedPlayer)null, Point = point, Normal = normal, IsLevel = true };
                            }
                        }
                    }
                });
            }
            
            NetworkManager.SendRPC("Shoot", [Camera.Position.X.ToString(), Camera.Position.Y.ToString(),Camera.Position.Z.ToString()]);

            if (closestHit.Distance < maxDistance)
            {
                if (closestHit.Player != null)
                {
                    const float damage = 10f;
                    NetworkManager.SendRPC("PlayerHit", new[]
                    {
                        closestHit.Player.Id.ToString(),
                        NetworkManager.ClientId.ToString(),
                        damage.ToString()
                    });
                }
                else if (closestHit.IsLevel)
                {
                    NetworkManager.SendRPC("LevelHit", new[]
                    {
                        NetworkManager.ClientId.ToString(),
                        closestHit.Point.X.ToString(),
                        closestHit.Point.Y.ToString(),
                        closestHit.Point.Z.ToString(),
                        closestHit.Normal.X.ToString(),
                        closestHit.Normal.Y.ToString(),
                        closestHit.Normal.Z.ToString()
                    });
                }
                ScrollToBottom = true;
            }

            Recoil *= Quaternion.CreateFromYawPitchRoll(0, 45, 0);
        }

        private Matrix4x4 CreatePlayerMatrix(ConnectedPlayer player)
        {
            return Matrix4x4.CreateScale(CharacterController.Height / 2) *
                   Matrix4x4.CreateFromQuaternion(player.Rotation * Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI)) *
                   Matrix4x4.CreateTranslation(player.LocalPosition - Vector3.UnitY * CharacterController.Height / 2);
        }

        private void Update(double deltaTime)
        {
            Time += (float)deltaTime;
            WeaponSway = Quaternion.Slerp(WeaponSway, Camera.Rotation, 15f * (float)deltaTime);
            Recoil = Quaternion.Slerp(Recoil, Quaternion.Identity, 5f * (float)deltaTime);
            UpdateNetwork();
            UpdateUserInterface((float)deltaTime);
            UpdateCharacterController((float)deltaTime);
            UpdateInput();
            RenderScene((float)deltaTime);
        }

        private void UpdateNetwork()
        {
            if (!NetworkManager.IsConnected || CharacterController == null) return;

            var eulerDegrees = Camera.GetEulerAngles();
            var rotation = Quaternion.CreateFromYawPitchRoll(eulerDegrees.Y * MathF.PI / 180f, 0, 0);
            NetworkManager.SendRPC("Update", new[]
            {
                NetworkManager.ClientId.ToString(),
                CharacterController.Position.X.ToString(),
                CharacterController.Position.Y.ToString(),
                CharacterController.Position.Z.ToString(),
                rotation.X.ToString(),
                rotation.Y.ToString(),
                rotation.Z.ToString(),
                rotation.W.ToString()
            });
        }

        private void UpdateUserInterface(float deltaTime)
        {
            var io = ImGui.GetIO();
            io.ConfigFlags |= ImGuiConfigFlags.DockingEnable | ImGuiConfigFlags.ViewportsEnable | ImGuiConfigFlags.DpiEnableScaleViewports;

            var viewport = ImGui.GetMainViewport();
            ImGui.SetNextWindowPos(viewport.Pos);
            ImGui.SetNextWindowSize(viewport.Size);
            ImGui.SetNextWindowViewport(viewport.ID);
            ImGui.DockSpaceOverViewport(viewport.ID, viewport, ImGuiDockNodeFlags.PassthruCentralNode);

            RenderChatWindow();
            RenderLocalPlayerHealth();
            RenderCrosshair();
            if (!IsMouseLocked) RenderDebugUserInterface(deltaTime);
            if (!IsLayoutLoaded)
            {
                ImGui.LoadIniSettingsFromDisk("./Layouts/DefaultLayout.ini");
                IsLayoutLoaded = true;
            }
        }
        private void RenderCrosshair()
        {
            var viewport = ImGui.GetMainViewport();
            var center = new Vector2(viewport.Pos.X + viewport.Size.X / 2, viewport.Pos.Y + viewport.Size.Y / 2);
            var drawList = ImGui.GetForegroundDrawList();

            float crosshairSize = 10f;
            float thickness = 2f;
            var color = ImGui.GetColorU32(ImGuiCol.Text);

            // Horizontal line
            drawList.AddLine(
                new Vector2(center.X - crosshairSize, center.Y),
                new Vector2(center.X + crosshairSize, center.Y),
                color,
                thickness
            );

            // Vertical line
            drawList.AddLine(
                new Vector2(center.X, center.Y - crosshairSize),
                new Vector2(center.X, center.Y + crosshairSize),
                color,
                thickness
            );
        }
        private void RenderLocalPlayerHealth()
        {
            var player = Players.Find(p => p.Id == NetworkManager.ClientId);
            if (player == null) return;

            var viewport = ImGui.GetMainViewport();
            var windowSize = new Vector2(200, 50);
            var windowPos = new Vector2(
                viewport.Pos.X + viewport.Size.X - windowSize.X - 10,
                viewport.Pos.Y + viewport.Size.Y - windowSize.Y - 10
            );

            ImGui.SetNextWindowPos(windowPos, ImGuiCond.Always);
            ImGui.SetNextWindowSize(windowSize, ImGuiCond.Always);

            if (ImGui.Begin("Player Health", ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoDocking))
            {
                ImGui.Text($"Health: {player.Health:F0}");
                ImGui.End();
            }
        }

        private void UpdateCharacterController(float deltaTime)
        {
            if (CharacterController == null) return;

            var moveInput = Vector3.Zero;
            var front = Camera.GetFront();
            var right = Vector3.Normalize(Vector3.Cross(front, Vector3.UnitY));
            front.Y = 0;
            front = Vector3.Normalize(front);
            right.Y = 0;
            right = Vector3.Normalize(right);

            if (!IsChatInputActive)
            {
                if (Keyboard.IsKeyPressed(Key.W)) moveInput += front;
                if (Keyboard.IsKeyPressed(Key.S)) moveInput -= front;
                if (Keyboard.IsKeyPressed(Key.A)) moveInput -= right;
                if (Keyboard.IsKeyPressed(Key.D)) moveInput += right;
                if (Keyboard.IsKeyPressed(Key.Space)) moveInput += Vector3.UnitY;
                if (Keyboard.IsKeyPressed(Key.ShiftLeft)) moveInput -= Vector3.UnitY;
                IsJumpRequested = Keyboard.IsKeyPressed(Key.Space);
            }

            CharacterController.Update(deltaTime, moveInput, IsJumpRequested);
            Camera.Position = CharacterController.Position + CharacterController.CamOffset;
        }

        private void UpdateInput()
        {
            bool isEscapePressed = Keyboard.IsKeyPressed(Key.Escape);
            if (isEscapePressed && !WasEscapePressed)
            {
                IsMouseLocked = !IsMouseLocked;
                Mouse.Cursor.CursorMode = IsMouseLocked ? CursorMode.Disabled : CursorMode.Normal;
                IsFirstMouse = true;
            }
            WasEscapePressed = isEscapePressed;

            bool isVPressed = Keyboard.IsKeyPressed(Key.V);
            if (isVPressed && !WasVPressed && !IsChatInputActive)
            {
                CharacterController.IsNoClipEnabled = !CharacterController.IsNoClipEnabled;
            }
            WasVPressed = isVPressed;
        }

        private void RenderScene(float DeltaTime)
        {
            ProjectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
                FieldOfView * MathF.PI / 180,
                (float)Window.RenderWidth / Window.RenderHeight,
                Rasterizer.NearClip,
                Rasterizer.FarClip);

            Window.ClearDepthBuffer();
            Window.ClearColorBuffer(new Vector4(ClearColor.X, ClearColor.Y, ClearColor.Z, 1f));
            RenderDust2();
            RenderGun();
            RenderConnectedPlayers(DeltaTime);
            RenderPlayerNametags();
            Window.RenderFrame();
        }

        private void RenderDust2()
        {
            if (Dust2Model == null)
            {
                Dust2Model = Model.LoadModel("./Assets/dust2/scene.gltf").Meshes;
                Random random = new Random();
                double randomValue = random.NextDouble();

                bool spawnAtFirst = randomValue > 0.5;

                Vector3 spawnPos = spawnAtFirst ? SpawnPosition : SpawnPosition2;
                Quaternion cameraRotation = spawnAtFirst ? Quaternion.Identity : Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI);

                Camera.Position = spawnPos;
                Camera.Rotation = cameraRotation;
                
                CharacterController = new CharacterController(spawnPos, [Dust2Model], [ModelMatrix]);
            }

            var viewMatrix = Camera.GetViewMatrix();
            RenderedModels = 0;

            Parallel.ForEach(Dust2Model, mesh =>
            {
                if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, ModelMatrix, viewMatrix, ProjectionMatrix))
                    return;

                var texture = LoadTexture(mesh);
                Rasterizer.RenderMesh(
                    Window,
                    mesh.Vertices.ToArray(),
                    mesh.Indices.ToArray(),
                    ModelMatrix,
                    viewMatrix,
                    ProjectionMatrix,
                    VertexShader,
                    input => FragmentShader(input, texture),
                    Rasterizer.CullMode.Back);

                lock (new object())
                {
                    RenderedModels++;
                }
            });
        }

        private void RenderGun()
        {
            if (GunModel == null)
            {
                GunModel = Model.LoadModel("./Assets/Gun/HL2 Pistol.obj").Meshes;
            }

            var viewMatrix = Camera.GetViewMatrix();
            var gunMatrix = GunMatrix * Matrix4x4.CreateFromQuaternion(WeaponSway * Recoil) *
                           Matrix4x4.CreateTranslation(Camera.Position + Vector3.Transform(new Vector3(0.1f, -0.05f, -0.15f + Math.Abs(Recoil.X / 5)), Camera.Rotation));

            Parallel.ForEach(GunModel, mesh =>
            {
                if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, gunMatrix, viewMatrix, ProjectionMatrix))
                    return;

                var texture = LoadTexture(mesh);
                Rasterizer.RenderMesh(
                    Window,
                    mesh.Vertices.ToArray(),
                    mesh.Indices.ToArray(),
                    gunMatrix,
                    viewMatrix,
                    ProjectionMatrix,
                    VertexShader,
                    input => FragmentShader(input, texture),
                    Rasterizer.CullMode.Back);

                lock (new object())
                {
                    RenderedModels++;
                }
            });
        }
        
        private void RenderConnectedPlayers(float DeltaTime)
        {
            if (PlayerModel == null)
            {
                PlayerModel = Model.LoadModel("./Assets/gordon_freeman/scene.gltf").Meshes;
            }

            if (PlayerModel == null || CharacterController == null) return;

            var viewMatrix = Camera.GetViewMatrix();
            foreach (var player in Players)
            {
                float InterpolationRate = 12f;
                float Factor = 1f - MathF.Exp(-InterpolationRate * DeltaTime);
                
                player.LocalPosition = Vector3.Lerp(player.LocalPosition, player.Position, Factor);


                if (player.Id == NetworkManager.ClientId) continue;
                
                var playerMatrix = CreatePlayerMatrix(player);
                foreach (var mesh in PlayerModel)
                {
                    if (!FrustumCuller.IsSphereInFrustum(mesh.SphereBounds, playerMatrix, viewMatrix, ProjectionMatrix))
                        continue;

                    var texture = LoadTexture(mesh);
                    Rasterizer.RenderMesh(
                        Window,
                        mesh.Vertices.ToArray(),
                        mesh.Indices.ToArray(),
                        playerMatrix,
                        viewMatrix,
                        ProjectionMatrix,
                        VertexShader,
                        input => FragmentShader(input, texture),
                        Rasterizer.CullMode.Back);
                }
            }
        }

        private void RenderPlayerNametags()
        {
            if (Players.Count == 0) return;

            var viewMatrix = Camera.GetViewMatrix();
            var viewProjection = viewMatrix * ProjectionMatrix;

            foreach (var player in Players)
            {
                if (player.Id == NetworkManager.ClientId) continue;

                var headPosition = player.LocalPosition + new Vector3(0, CharacterController.Height / 2f, 0);
                var clipPos = Vector4.Transform(new Vector4(headPosition, 1f), viewProjection);

                if (clipPos.W <= 0.001f) continue;

                var ndc = new Vector3(clipPos.X, clipPos.Y, clipPos.Z) / clipPos.W;
                if (ndc.Z < 0 || ndc.Z > 1) continue;

                var screenPos = new Vector2(
                    (ndc.X + 1f) * 0.5f * Window.WindowWidth,
                    (1f - ndc.Y) * 0.5f * Window.WindowHeight);

                if (screenPos.X < 0 || screenPos.X > Window.WindowWidth ||
                    screenPos.Y < 0 || screenPos.Y > Window.WindowHeight)
                    continue;

                ImGui.SetNextWindowPos(screenPos, ImGuiCond.Always, new Vector2(0.5f, 1f));
                ImGui.SetNextWindowBgAlpha(0.35f);

                if (ImGui.Begin($"###Nametag_{player.Name}",
                        ImGuiWindowFlags.NoDecoration |
                        ImGuiWindowFlags.AlwaysAutoResize |
                        ImGuiWindowFlags.NoSavedSettings |
                        ImGuiWindowFlags.NoFocusOnAppearing |
                        ImGuiWindowFlags.NoNav))
                {
                    ImGui.Text($"{player.Name} - Health: {player.Health:F0}");
                    ImGui.End();
                }
            }
        }

        private void RenderChatWindow()
        {
            unsafe
            {
                var viewport = ImGui.GetMainViewport();
                var windowSize = new Vector2(400, 300);
                var windowPos = new Vector2(viewport.Pos.X + viewport.Size.X - windowSize.X - 10, viewport.Pos.Y + 10);

                ImGui.SetNextWindowPos(windowPos, ImGuiCond.Always);
                ImGui.SetNextWindowSize(windowSize, ImGuiCond.Always);

                if (!ImGui.Begin("Chat", ref IsChatWindowOpen, ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoDocking))
                {
                    ImGui.End();
                    return;
                }

                ImGui.BeginChild("ChatContent", new Vector2(0, -ImGui.GetFrameHeightWithSpacing()));
                foreach (var message in ChatMessages)
                {
                    if (!string.IsNullOrEmpty(message))
                    {
                        ImGui.PushTextWrapPos(0f);
                        ImGui.TextUnformatted(message);
                        ImGui.PopTextWrapPos();
                    }
                }

                var inputBoxHeight = ImGui.GetFrameHeightWithSpacing();
                var scrollMax = ImGui.GetScrollMaxY();
                var scrollY = ImGui.GetScrollY();
                var lastMessageHeight = ChatMessages.Count > 0 && !string.IsNullOrEmpty(ChatMessages[^1])
                    ? ImGui.CalcTextSize(ChatMessages[^1], false, ImGui.GetContentRegionAvail().X).Y + ImGui.GetStyle().ItemSpacing.Y
                    : 0f;

                if (ScrollToBottom || scrollY >= scrollMax - (lastMessageHeight + inputBoxHeight))
                {
                    ImGui.SetScrollY(scrollMax);
                    ScrollToBottom = false;
                }

                ImGui.EndChild();
                ImGui.Separator();

                var inputTextFlags = ImGuiInputTextFlags.EnterReturnsTrue | ImGuiInputTextFlags.CallbackCompletion | ImGuiInputTextFlags.CallbackHistory;
                bool reclaimFocus = false;

                if (ImGui.InputText("##ChatInput", ref ChatInput, 256, inputTextFlags, _ => 0))
                {
                    if (!string.IsNullOrWhiteSpace(ChatInput))
                    {
                        if (NetworkManager.IsConnected)
                        {
                            NetworkManager.SendRPC("ChatMessage", new[] { $"{ Players.Find(p => p.Id == NetworkManager.ClientId)?.Name }", ChatInput });
                        }
                        ChatInput = "";
                        ScrollToBottom = true;
                    }
                    reclaimFocus = true;
                }

                ImGui.SetItemDefaultFocus();
                if (reclaimFocus || (IsChatInputActive && !ImGui.IsItemActive()))
                {
                    ImGui.SetKeyboardFocusHere(-1);
                }
                IsChatInputActive = ImGui.IsItemActive();
                ImGui.End();
            }
        }

        private void RenderDebugUserInterface(float deltaTime)
        {
            ImGui.Begin("Renderer Controls", ImGuiWindowFlags.None);

            if (ImGui.CollapsingHeader("Performance", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.Text($"FPS: {(int)(1 / deltaTime)}");
                ImGui.Text($"Frame Time: {deltaTime * 1000:F2}ms");
                ImGui.Text($"Rendered Meshes: {RenderedModels}");
                ImGui.Text($"Cached Textures: {CachedTextures.Count}");
            }

            if (ImGui.CollapsingHeader("Scene Info", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.Text($"Loaded Meshes: {Dust2Model?.Count ?? 0}");
                ImGui.Text($"Runtime: {Time:F2}s");
                ImGui.Text($"Window Size: {Window.WindowWidth}x{Window.WindowHeight}");
                ImGui.Text($"Render Size: {Window.RenderWidth}x{Window.RenderHeight}");
            }

            if (ImGui.CollapsingHeader("Multiplayer", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.Text("Connected Players:");
                foreach (var player in Players)
                {
                    ImGui.Text($"Player ID: {player.Id}");
                    ImGui.Text($"Player Name: {player.Name}");
                    ImGui.Text($"Position: {player.Position}");
                    ImGui.Text($"Rotation: {player.Rotation}");
                }
            }

            if (ImGui.CollapsingHeader("Camera", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.Text("Clipping:");
                ImGui.DragFloat("Near Clip", ref Rasterizer.NearClip, 0.01f, 0.001f, 1);
                Rasterizer.NearClip = Math.Max(Rasterizer.NearClip, 0.001f);
                ImGui.DragFloat("Far Clip", ref Rasterizer.FarClip, 0.5f, 0.001f, 1000);
                Rasterizer.FarClip = Math.Max(Rasterizer.FarClip, Rasterizer.NearClip + 0.01f);

                ImGui.Text("Rotation:");
                var eulerDegrees = Camera.GetEulerAngles();
                float pitch = eulerDegrees.X;
                float yaw = eulerDegrees.Y;
                float roll = eulerDegrees.Z;
                bool updated = false;

                if (ImGui.DragFloat("Pitch (°)", ref pitch, 0.5f))
                {
                    pitch = Math.Clamp(pitch, -89f, 89f);
                    updated = true;
                }
                if (ImGui.DragFloat("Yaw (°)", ref yaw, 0.5f)) updated = true;
                if (ImGui.DragFloat("Roll (°)", ref roll, 0.5f)) updated = true;

                if (updated)
                {
                    Camera.Rotation = Quaternion.CreateFromYawPitchRoll(
                        yaw * MathF.PI / 180f,
                        pitch * MathF.PI / 180f,
                        roll * MathF.PI / 180f);
                }

                ImGui.SliderFloat("Mouse Sensitivity", ref Camera.Sensitivity, 0.01f, 1f);
                ImGui.SliderFloat("FOV", ref FieldOfView, 1f, 179f);
                var frontVec = Camera.GetFront();
                ImGui.Text($"Front: {frontVec.X:F2}, {frontVec.Y:F2}, {frontVec.Z:F2}");
            }

            if (CharacterController != null && ImGui.CollapsingHeader("Character Controller", ImGuiTreeNodeFlags.DefaultOpen))
            {
                var camPos = Camera.Position;
                if (ImGui.DragFloat3("Position", ref camPos, 0.1f))
                {
                    Camera.Position = camPos;
                    CharacterController.Position = camPos;
                }

                var camOffset = CharacterController.CamOffset;
                if (ImGui.DragFloat3("Camera Offset", ref camOffset, 0.1f))
                    CharacterController.CamOffset = camOffset;

                var moveSpeed = CharacterController.MoveSpeed;
                if (ImGui.DragFloat("Move Speed", ref moveSpeed, 0.1f, 1f, 20f))
                    CharacterController.MoveSpeed = moveSpeed;

                var maxAirSpeed = CharacterController.MaxAirSpeed;
                if (ImGui.DragFloat("Max Air Speed", ref maxAirSpeed, 0.1f, 1f, 20f))
                    CharacterController.MaxAirSpeed = maxAirSpeed;

                var jumpForce = CharacterController.JumpForce;
                if (ImGui.DragFloat("Jump Force", ref jumpForce, 0.1f, 1f, 20f))
                    CharacterController.JumpForce = jumpForce;

                var radius = CharacterController.Radius;
                if (ImGui.DragFloat("Radius", ref radius, 0.1f, 1f, 20f))
                    CharacterController.Radius = radius;

                var height = CharacterController.Height;
                if (ImGui.DragFloat("Height", ref height, 0.1f, 0.5f, 3f))
                    CharacterController.Height = height;

                var groundAccel = CharacterController.GroundAcceleration;
                if (ImGui.DragFloat("Ground Acceleration", ref groundAccel, 0.1f, 1f, 20f))
                    CharacterController.GroundAcceleration = groundAccel;

                var airAccel = CharacterController.AirAcceleration;
                if (ImGui.DragFloat("Air Acceleration", ref airAccel, 0.1f, 1f, 20f))
                    CharacterController.AirAcceleration = airAccel;

                var groundFriction = CharacterController.GroundFriction;
                if (ImGui.DragFloat("Ground Friction", ref groundFriction, 0.1f, 1f, 20f))
                    CharacterController.GroundFriction = groundFriction;

                var airControl = CharacterController.AirControl;
                if (ImGui.DragFloat("Air Control", ref airControl, 0.01f, 0f, 1f))
                    CharacterController.AirControl = airControl;

                var stepSize = CharacterController.StepSize;
                if (ImGui.DragFloat("Step Size", ref stepSize, 0.1f, 0.5f, 3f))
                    CharacterController.StepSize = stepSize;

                var gravity = CharacterController.Gravity;
                if (ImGui.DragFloat3("Gravity", ref gravity, 0.1f, -20f, 20f))
                    CharacterController.Gravity = gravity;

                ImGui.Text($"Velocity: {CharacterController.Velocity.X:F2}, {CharacterController.Velocity.Y:F2}, {CharacterController.Velocity.Z:F2}");
                ImGui.Text($"Is Grounded: {CharacterController.IsGrounded}");
            }

            if (ImGui.CollapsingHeader("Rendering", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.Text($"Render Scale: {(int)(Window.RenderScale * 100)}%%");
                if (ImGui.SliderFloat("##RenderScale", ref Window.RenderScale, 0.1f, 1f))
                {
                    Window.UpdateRenderScale(Window.RenderScale);
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
                    LightDirection = EulerToDirection(LightEulerDegrees);
                }
                ImGui.ColorEdit4("Light Color", ref LightColor);
            }

            ImGui.End();
        }

        private Texture LoadTexture(Mesh mesh)
        {
            if (mesh?.Material?.TexturePaths?.TryGetValue(TextureSlot.Diffuse, out var texturePath) == true)
            {
                return CachedTextures.GetOrAdd(texturePath, SoftwareRenderer.Texture.LoadTexture);
            }
            return null;
        }

        private Shaders.VertexOutput VertexShader(Shaders.VertexInput vertex, Matrix4x4 modelMatrix, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix)
        {
            var worldPos = Vector4.Transform(new Vector4(vertex.Position, 1), modelMatrix);
            var viewPos = Vector4.Transform(worldPos, viewMatrix);
            var clipPos = Vector4.Transform(viewPos, projectionMatrix);
            var worldNormal = Vector3.Normalize(Vector3.TransformNormal(vertex.Normal, modelMatrix));

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
            var worldNormal = (Vector3)input.Data["WorldNormal"];
            var diffuse = MathF.Max(0.25f, Vector3.Dot(worldNormal, -LightDirection));
            var textureColor = texture?.Sample(input.TexCoord) ?? Vector4.One;
            var baseColor = input.Color * textureColor;
            var depth = input.ClipPosition.Z;
            var fogFactor = Math.Clamp((FogEnd - depth) / (FogEnd - FogStart), 0f, 1f);
            fogFactor = fogFactor * fogFactor * (3f - 2f * fogFactor);

            var finalColor = Vector4.Lerp(FogColor, baseColor * (0.1f + 0.9f * diffuse) * LightColor, fogFactor);
            return new Vector4(finalColor.X, finalColor.Y, finalColor.Z, baseColor.W);
        }

        private void RegisterNetworkCallbacks()
        {
            if (!NetworkManager.IsConnected) return;

            NetworkManager.OnReceiveRPC += async (method, parameters) =>
            {
                switch (method)
                {
                    case "ConnectedPlayer"
                        when parameters.Length >= 2 && int.TryParse(parameters[0], out int newPlayerId):
                        Players.Add(new ConnectedPlayer { Id = newPlayerId, Name = parameters[1], Health = 100f });
                        ChatMessages.Add($"{parameters[1]} has joined the game!");
                        ScrollToBottom = true;
                        break;

                    case "Update" when parameters.Length >= 8 && int.TryParse(parameters[0], out int playerId):
                        var player = Players.Find(p => p.Id == playerId);
                        if (player != null)
                        {
                            player.Position = new Vector3(
                                float.Parse(parameters[1]),
                                float.Parse(parameters[2]),
                                float.Parse(parameters[3]));
                            player.Rotation = new Quaternion(
                                float.Parse(parameters[4]),
                                float.Parse(parameters[5]),
                                float.Parse(parameters[6]),
                                float.Parse(parameters[7]));
                        }

                        break;

                    case "DisconnectedPlayer"
                        when parameters.Length >= 1 && int.TryParse(parameters[0], out int disconnectedId):
                        var disconnectedPlayer = Players.Find(p => p.Id == disconnectedId);
                        if (disconnectedPlayer != null)
                        {
                            Players.Remove(disconnectedPlayer);
                            Console.WriteLine($"Player disconnected: {disconnectedPlayer.Name}");
                        }

                        break;

                    case "ChatMessage" when parameters.Length >= 2:
                        ChatMessages.Add($"{parameters[0]}: {parameters[1]}");
                        ScrollToBottom = true;
                        break;

                    case "PlayerHit" when parameters.Length >= 3 && int.TryParse(parameters[0], out int hitPlayerId) &&
                                          float.TryParse(parameters[2], out float damage):
                        var hitPlayer = Players.Find(p => p.Id == hitPlayerId);
                        if (hitPlayer != null)
                        {
                            hitPlayer.Health = Math.Max(0f, hitPlayer.Health - damage);
                            if (hitPlayer.Health <= 0f)
                            {
                                ChatMessages.Add($"{hitPlayer.Name} was killed!");
                                if (NetworkManager.ClientId == hitPlayerId && CharacterController != null)
                                {
                                    Random random = new Random();
                                    double randomValue = random.NextDouble();

                                    bool spawnAtFirst = randomValue > 0.5;

                                    Vector3 spawnPos = spawnAtFirst ? SpawnPosition : SpawnPosition2;
                                    Quaternion cameraRotation = spawnAtFirst
                                        ? Quaternion.Identity
                                        : Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI);
                                    CharacterController.Position = spawnPos;
                                    Camera.Rotation = cameraRotation;
                                }

                                hitPlayer.Health = 100f;
                                NetworkManager.SendRPC("Update", new[]
                                {
                                    hitPlayer.Id.ToString(),
                                    hitPlayer.Position.X.ToString(),
                                    hitPlayer.Position.Y.ToString(),
                                    hitPlayer.Position.Z.ToString(),
                                    hitPlayer.Rotation.X.ToString(),
                                    hitPlayer.Rotation.Y.ToString(),
                                    hitPlayer.Rotation.Z.ToString(),
                                    hitPlayer.Rotation.W.ToString()
                                });
                            }

                            ScrollToBottom = true;
                        }

                        break;
                    case "Shoot":
                        Console.WriteLine("Shot sound");
                        Vector3 Position = new Vector3(
                            float.Parse(parameters[0]),
                            float.Parse(parameters[1]),
                            float.Parse(parameters[2]));
                        float Distance = Vector3.Distance(Camera.Position, Position);
                        float Volume = Math.Clamp(25f / (0.25f * Distance), 0f, 100f); // SFML style volume 0-100
                        string FilePath = "./Assets/pistol.wav";

                        try
                        {
                            if (SDL.SDL_Init(SDL.SDL_INIT_AUDIO) != 0)
                            {
                                Console.WriteLine("SDL_Init Error: " + SDL.SDL_GetError());
                                return;
                            }

                            if (SDL.SDL_LoadWAV(FilePath, out var Spec, out var AudioBuf, out var AudioLen) == IntPtr.Zero)
                            {
                                Console.WriteLine("Could not load WAV file.");
                                return;
                            }

                            AdjustVolume(AudioBuf, AudioLen, Volume / 100f);

                            Spec.callback = null;
                            Spec.userdata = IntPtr.Zero;

                            var DeviceId = SDL.SDL_OpenAudioDevice(null, 0, ref Spec, out _, 0);
                            if (DeviceId == 0)
                            {
                                Console.WriteLine("Failed to open audio device.");
                                SDL.SDL_FreeWAV(AudioBuf);
                                return;
                            }

                            SDL.SDL_QueueAudio(DeviceId, AudioBuf, AudioLen);
                            SDL.SDL_PauseAudioDevice(DeviceId, 0);
                            
                            int bytesPerSample = SDL.SDL_AUDIO_BITSIZE(Spec.format) / 8;
                            int durationMs = (int)((AudioLen / (Spec.channels * bytesPerSample * (float)Spec.freq)) * 1000);
                            
                            _ = Task.Run(async () =>
                            {
                                try
                                {
                                    await Task.Delay(durationMs);
                                }
                                finally
                                {
                                    SDL.SDL_CloseAudioDevice(DeviceId);
                                    SDL.SDL_FreeWAV(AudioBuf);
                                }
                            });
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Failed to play sound: {ex.Message}");
                        }
                        break;
                }
            };
        }
        
        unsafe void AdjustVolume(IntPtr buffer, uint length, float volume)
        {
            short* samples = (short*)buffer;
            int sampleCount = (int)(length / 2);
            for (int i = 0; i < sampleCount; i++)
            {
                int sample = (int)(samples[i] * volume);
                // Clamp to short range
                if (sample > short.MaxValue) sample = short.MaxValue;
                if (sample < short.MinValue) sample = short.MinValue;
                samples[i] = (short)sample;
            }
        }

        private static Vector3 EulerToDirection(Vector3 eulerDegrees)
        {
            var radians = eulerDegrees * MathF.PI / 180f;
            var rotation = Matrix4x4.CreateFromYawPitchRoll(radians.Y, radians.X, radians.Z);
            return Vector3.Normalize(Vector3.Transform(-Vector3.UnitZ, rotation));
        }

        public void Dispose()
        {
            if (NetworkManager?.IsConnected == true)
            {
                NetworkManager.SendRPC("DisconnectedPlayer", new[] { NetworkManager.ClientId.ToString() });
                NetworkManager.Close();
            }
            CachedTextures.Clear();
            InputContext?.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}