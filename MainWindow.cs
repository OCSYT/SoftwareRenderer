﻿﻿using Silk.NET.Windowing;
using Silk.NET.OpenGL;
using Silk.NET.Maths;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using Silk.NET.Input;
using System.Threading.Tasks;
using System.Numerics;
using System.Runtime.CompilerServices;
using Coroutine;
using Silk.NET.OpenGL.Extensions.ImGui;


namespace SoftwareRenderer
{
    public class MainWindow
    {
        public ImGuiController ImGuiController;
        public IInputContext InputContext;
        private IWindow Window;
        public static GL Gl { get; private set; }
        public int WindowWidth;
        public int WindowHeight;
        public int RenderWidth { get; set; } = 800;
        public int RenderHeight { get; set; } = 600;
        public float RenderScale = 1.0f;

        // Buffers as 1D arrays for speed
        public Vector4[] ColorBuffer;
        public float[] DepthBuffer;
        private uint TextureHandle;

        private uint VaoHandle;
        private uint VboHandle;
        private uint ShaderProgram;

        public delegate void StartEventHandler();
        public event StartEventHandler? StartEvent;
        public delegate void CloseEventHandler();
        public event CloseEventHandler? CloseEvent;
        public delegate void UpdateEventHandler(double DeltaTime);
        public event UpdateEventHandler? UpdateEvent;

        public MainWindow(string Title = "Renderer")
        {
            var options = WindowOptions.Default with
            {
                Size = new Vector2D<int>(800, 600),
                Title = Title,
                WindowState = WindowState.Normal,
                WindowBorder = WindowBorder.Resizable,
                VSync = false,
                API = new GraphicsAPI(ContextAPI.OpenGL, ContextProfile.Core, ContextFlags.ForwardCompatible, new APIVersion(4, 3))
            };
            WindowWidth = options.Size.X;
            WindowHeight = options.Size.Y;
            RenderWidth = options.Size.X;
            RenderHeight = options.Size.Y;

            Window = Silk.NET.Windowing.Window.Create(options);

            Window.Load += OnLoad;
            Window.Render += OnRender;
            Window.Resize += OnResize;
            Window.Closing += OnClosing;
        }

        public void Run() => Window.Run();

        private void OnLoad()
        {
            InputContext = Window.CreateInput();
            Gl = Window.CreateOpenGL();
            ImGuiController = new ImGuiController(Gl, Window, InputContext);
            Gl.ClearColor(0f, 0f, 0f, 1f);

            // Create and configure texture
            TextureHandle = Gl.GenTexture();
            Gl.BindTexture(TextureTarget.Texture2D, TextureHandle);

            Gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            Gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);
            Gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToEdge);
            Gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToEdge);

            SetupQuad();
            SetupShaders();

            StartEvent?.Invoke();

            // Allow render scale up to 4.0 for bigger resolutions
            RenderScale = Math.Clamp(RenderScale, 0.1f, 1f);

            RenderWidth = (int)(WindowWidth * RenderScale);
            RenderHeight = (int)(WindowHeight * RenderScale);

            HandleResize(new Vector2D<int>(WindowWidth, WindowHeight));
        }

        private void SetupQuad()
        {
            unsafe
            {
                float[] vertices =
                {
                    // Positions      // TexCoords
                    -1f, -1f, 0f,     0f, 1f,
                     1f, -1f, 0f,     1f, 1f,
                     1f,  1f, 0f,     1f, 0f,

                    -1f, -1f, 0f,     0f, 1f,
                     1f,  1f, 0f,     1f, 0f,
                    -1f,  1f, 0f,     0f, 0f
                };

                VaoHandle = Gl.GenVertexArray();
                Gl.BindVertexArray(VaoHandle);

                VboHandle = Gl.GenBuffer();
                Gl.BindBuffer(BufferTargetARB.ArrayBuffer, VboHandle);

                fixed (float* ptr = vertices)
                {
                    Gl.BufferData(BufferTargetARB.ArrayBuffer,
                        (nuint)(vertices.Length * sizeof(float)),
                        ptr,
                        BufferUsageARB.StaticDraw);
                }

                Gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 5 * sizeof(float), (void*)0);
                Gl.EnableVertexAttribArray(0);

                Gl.VertexAttribPointer(1, 2, VertexAttribPointerType.Float, false, 5 * sizeof(float), (void*)(3 * sizeof(float)));
                Gl.EnableVertexAttribArray(1);

                Gl.BindVertexArray(0);
            }
        }

        private void SetupShaders()
        {
            const string vertexShaderSource = @"
                #version 330 core
                layout(location = 0) in vec3 aPos;
                layout(location = 1) in vec2 aTexCoord;

                out vec2 TexCoord;

                void main()
                {
                    gl_Position = vec4(aPos, 1.0);
                    TexCoord = aTexCoord;
                }
            ";

            const string fragmentShaderSource = @"
                #version 330 core
                out vec4 FragColor;

                in vec2 TexCoord;

                uniform sampler2D screenTexture;

                void main()
                {
                    FragColor = texture(screenTexture, TexCoord);
                }
            ";

            uint vertexShader = Gl.CreateShader(ShaderType.VertexShader);
            Gl.ShaderSource(vertexShader, vertexShaderSource);
            Gl.CompileShader(vertexShader);
            CheckShaderCompileStatus(vertexShader);

            uint fragmentShader = Gl.CreateShader(ShaderType.FragmentShader);
            Gl.ShaderSource(fragmentShader, fragmentShaderSource);
            Gl.CompileShader(fragmentShader);
            CheckShaderCompileStatus(fragmentShader);

            ShaderProgram = Gl.CreateProgram();
            Gl.AttachShader(ShaderProgram, vertexShader);
            Gl.AttachShader(ShaderProgram, fragmentShader);
            Gl.LinkProgram(ShaderProgram);

            Gl.GetProgram(ShaderProgram, GLEnum.LinkStatus, out int status);
            if (status == 0)
            {
                Console.WriteLine($"Shader program link error: {Gl.GetProgramInfoLog(ShaderProgram)}");
            }

            Gl.DetachShader(ShaderProgram, vertexShader);
            Gl.DetachShader(ShaderProgram, fragmentShader);
            Gl.DeleteShader(vertexShader);
            Gl.DeleteShader(fragmentShader);
        }

        private void CheckShaderCompileStatus(uint shader)
        {
            Gl.GetShader(shader, ShaderParameterName.CompileStatus, out int status);
            if (status == 0)
            {
                Console.WriteLine($"Shader compile error: {Gl.GetShaderInfoLog(shader)}");
            }
        }

        private volatile bool ManualRenderRequested = false;

        public void RenderFrame()
        {
            ManualRenderRequested = true;
        }

        DateTime LastTime = DateTime.Now;
        DateTime CurrentTime = DateTime.Now;

        private void OnRender(double deltaTime)
        {
            CurrentTime = DateTime.Now;
            CoroutineHandler.Tick(CurrentTime - LastTime);
            LastTime = CurrentTime;


            ImGuiController.Update((float)deltaTime);
            UpdateEvent?.Invoke(deltaTime);
            if (ManualRenderRequested)
            {
                Gl.Clear(ClearBufferMask.ColorBufferBit);
                Gl.BindTexture(TextureTarget.Texture2D, TextureHandle);

                int width = RenderWidth;
                int height = RenderHeight;

                var flatColorBuffer = new Vector3[ColorBuffer.Length];

                Parallel.For(0, ColorBuffer.Length, i =>
                {
                    Vector4 pixel = ColorBuffer[i];
                    flatColorBuffer[i] = new Vector3(pixel.X, pixel.Y, pixel.Z);
                });

                GCHandle gcHandle = GCHandle.Alloc(flatColorBuffer, GCHandleType.Pinned);
                try
                {
                    unsafe
                    {
                        Gl.TexSubImage2D(TextureTarget.Texture2D, 0, 0, 0,
                            (uint)width, (uint)height,
                            PixelFormat.Rgb, PixelType.Float,
                            gcHandle.AddrOfPinnedObject().ToPointer());
                    }
                }
                finally
                {
                    gcHandle.Free();
                }

                Gl.UseProgram(ShaderProgram);
                Gl.BindVertexArray(VaoHandle);
                Gl.DrawArrays(PrimitiveType.Triangles, 0, 6);

                ManualRenderRequested = false;
            }

            ImGuiController.Render();
        }

        public void UpdateRenderScale(float scale)
        {
            RenderScale = scale;
            RenderWidth = (int)(WindowWidth * RenderScale);
            RenderHeight = (int)(WindowHeight * RenderScale);
            HandleResize(new Vector2D<int>(WindowWidth, WindowHeight));
        }
        
        private ActiveCoroutine? Active; // Track the running coroutine

        public void ScheduleResize(Vector2D<int> newSize)
        {
            if (Active != null)
            {
                Active.Cancel();
            }
            
            Active = CoroutineHandler.Start(ResizeCoroutine(newSize), "Resize Coroutine");
        }

        private IEnumerable<Wait> ResizeCoroutine(Vector2D<int> newSize)
        {
            yield return new Wait(.25f);
            Console.WriteLine("resizing");
            if (Active?.IsFinished == false)
            {
                HandleResize(newSize);
            }
        }
        
        private void HandleResize(Vector2D<int> newSize)
        {
            // Early validation with constants
            const int MinDimension = 1;
            if (newSize.X < MinDimension || newSize.Y < MinDimension)
            {
                Console.WriteLine("Resize ignored: dimensions must be positive.");
                return;
            }

            // Cache new dimensions
            int windowWidth = newSize.X;
            int windowHeight = newSize.Y;

            // Update render scale and dimensions
            const float MinRenderScale = 0.1f;
            const float MaxRenderScale = 1.0f;
            float clampedScale = Math.Clamp(RenderScale, MinRenderScale, MaxRenderScale);
            int renderWidth = Math.Max((int)(windowWidth * clampedScale), MinDimension);
            int renderHeight = Math.Max((int)(windowHeight * clampedScale), MinDimension);

            // Allocate new buffers
            var newColorBuffer = new Vector4[renderWidth * renderHeight];
            var newDepthBuffer = new float[renderWidth * renderHeight];

            // Copy existing buffer data if available
            if (ColorBuffer is not null && DepthBuffer is not null)
            {
                int oldWidth = ColorBuffer.Length > 0 ? (int)Math.Sqrt(ColorBuffer.Length) : 0;
                int oldHeight = oldWidth > 0 ? ColorBuffer.Length / oldWidth : 0;

                if (oldWidth > 0 && oldHeight > 0)
                {
                    int copyWidth = Math.Min(oldWidth, renderWidth);
                    int copyHeight = Math.Min(oldHeight, renderHeight);
                    Parallel.For(0, copyHeight, y =>
                    {
                        Array.Copy(ColorBuffer, y * oldWidth, newColorBuffer, y * renderWidth, copyWidth);
                        Array.Copy(DepthBuffer, y * oldWidth, newDepthBuffer, y * renderWidth, copyWidth);
                    });
                }
            }

            // Update instance fields
            WindowWidth = windowWidth;
            WindowHeight = windowHeight;
            RenderScale = clampedScale;
            RenderWidth = renderWidth;
            RenderHeight = renderHeight;
            ColorBuffer = newColorBuffer;
            DepthBuffer = newDepthBuffer;

            Gl.BindTexture(TextureTarget.Texture2D, TextureHandle);
            unsafe
            {
                Gl.TexImage2D(TextureTarget.Texture2D, 0, InternalFormat.Rgba32f,
                    (uint)renderWidth, (uint)renderHeight, 0,
                    PixelFormat.Rgba, PixelType.Float, null);
            }

            Gl.Viewport(0, 0, (uint)windowWidth, (uint)windowHeight);
            
            Console.WriteLine($"Resized to: {windowWidth}x{windowHeight} (Render: {renderWidth}x{renderHeight})");
        }

        private void OnResize(Vector2D<int> newSize)
        {
            Gl.Viewport(0, 0, (uint)newSize.X, (uint)newSize.Y);
            ScheduleResize(newSize);
        }

        private void OnClosing()
        {
            Gl.DeleteTexture(TextureHandle);
            Gl.DeleteBuffer(VboHandle);
            Gl.DeleteVertexArray(VaoHandle);
            Gl.DeleteProgram(ShaderProgram);
            CloseEvent?.Invoke();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetIndex(int x, int y) => y * RenderWidth + x;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetPixel(int x, int y, Vector4 color)
        {
            if (x >= 0 && x < RenderWidth && y >= 0 && y < RenderHeight)
            {
                ColorBuffer[GetIndex(x, y)] = color;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector4 GetPixel(int x, int y)
        {
            if (x >= 0 && x < RenderWidth && y >= 0 && y < RenderHeight)
            {
                return ColorBuffer[GetIndex(x, y)];
            }
            return Vector4.Zero;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearColorBuffer(Vector4 clearColor)
        {
            int count = ColorBuffer.Length;
            Parallel.For(0, count, i =>
            {
                ColorBuffer[i] = clearColor;
            });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetDepth(int x, int y, float depth)
        {
            if (x >= 0 && x < RenderWidth && y >= 0 && y < RenderHeight)
            {
                DepthBuffer[GetIndex(x, y)] = depth;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetDepth(int x, int y)
        {
            if (x >= 0 && x < RenderWidth && y >= 0 && y < RenderHeight)
            {
                return DepthBuffer[GetIndex(x, y)];
            }
            return float.MinValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearDepthBuffer()
        {
            int count = DepthBuffer.Length;
            Parallel.For(0, count, i =>
            {
                DepthBuffer[i] = float.MinValue;
            });
        }
    }
}