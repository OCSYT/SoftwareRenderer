using Silk.NET.Windowing;
using Silk.NET.OpenGL;
using Silk.NET.Maths;
using System;
using System.Runtime.InteropServices;
using Silk.NET.Input;
using System.Threading.Tasks;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SoftwareRenderer
{
    public class MainWindow
    {
        public IInputContext InputContext;
        private IWindow Window;
        private GL Gl;
        public int WindowWidth;
        public int WindowHeight;
        public int RenderWidth { get; set; } = 800;
        public int RenderHeight { get; set; } = 600;
        public float RenderScale = 1.0f;

        // Buffers as 1D arrays for speed
        private Vector4[] ColorBuffer;
        private float[] DepthBuffer;
        private uint TextureHandle;

        private uint VaoHandle;
        private uint VboHandle;
        private uint ShaderProgram;

        public delegate void StartEventHandler();
        public event StartEventHandler? StartEvent;
        public delegate void UpdateEventHandler(double DeltaTime);
        public event UpdateEventHandler? UpdateEvent;

        public MainWindow(string Title = "Renderer")
        {
            var options = WindowOptions.Default;
            options.Size = new Vector2D<int>(800, 600);
            options.Title = Title;
            options.WindowState = WindowState.Normal;
            options.WindowBorder = WindowBorder.Resizable;
            options.VSync = false;
            WindowWidth = options.Size.X;
            WindowHeight = options.Size.Y;
            RenderWidth = options.Size.X;
            RenderHeight = options.Size.Y;

            Window = Silk.NET.Windowing.Window.Create(options);

            Window.Load += OnLoad;
            Window.Update += OnUpdate;
            Window.Render += OnRender;
            Window.Resize += OnResize;
            Window.Closing += OnClosing;
        }

        private void OnUpdate(double DeltaTime)
        {
            UpdateEvent?.Invoke(DeltaTime);
        }

        public void Run() => Window.Run();

        private void OnLoad()
        {
            InputContext = Window.CreateInput();
            Gl = GL.GetApi(Window);
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
            RenderScale = Math.Clamp(RenderScale, 1 / 8f, 1f);

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

        private void OnRender(double deltaTime)
        {
            if (!ManualRenderRequested)
                return;

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

        void HandleResize(Vector2D<int> newSize)
        {
            RenderScale = Math.Clamp(RenderScale, 1 / 8f, 4.0f);

            WindowWidth = newSize.X;
            WindowHeight = newSize.Y;

            RenderWidth = (int)(WindowWidth * RenderScale);
            RenderHeight = (int)(WindowHeight * RenderScale);

            if (RenderWidth <= 0 || RenderHeight <= 0)
            {
                Console.WriteLine("Resize ignored due to zero render dimensions.");
                return;
            }

            Gl.BindTexture(TextureTarget.Texture2D, TextureHandle);

            Vector4[] oldColorBuffer = ColorBuffer;
            float[] oldDepthBuffer = DepthBuffer;

            int oldWidth = oldColorBuffer?.Length > 0 ? oldColorBuffer.Length / (oldDepthBuffer?.Length > 0 ? oldDepthBuffer.Length / oldColorBuffer.Length : 1) : 0;
            // But safer to keep old width/height:
            oldWidth = oldColorBuffer?.Length > 0 && RenderWidth != 0 ? oldColorBuffer.Length / RenderHeight : RenderWidth;
            int oldHeight = oldColorBuffer?.Length > 0 ? oldColorBuffer.Length / oldWidth : RenderHeight;

            Vector4[] newColorBuffer = new Vector4[RenderWidth * RenderHeight];
            float[] newDepthBuffer = new float[RenderWidth * RenderHeight];

            int copyHeight = Math.Min(oldHeight, RenderHeight);
            int copyWidth = Math.Min(oldWidth, RenderWidth);

            if (oldColorBuffer != null)
            {
                Parallel.For(0, copyHeight, y =>
                {
                    int newRowStart = y * RenderWidth;
                    int oldRowStart = y * oldWidth;
                    for (int x = 0; x < copyWidth; x++)
                    {
                        newColorBuffer[newRowStart + x] = oldColorBuffer[oldRowStart + x];
                    }
                });
            }

            if (oldDepthBuffer != null)
            {
                Parallel.For(0, copyHeight, y =>
                {
                    int newRowStart = y * RenderWidth;
                    int oldRowStart = y * oldWidth;
                    for (int x = 0; x < copyWidth; x++)
                    {
                        newDepthBuffer[newRowStart + x] = oldDepthBuffer[oldRowStart + x];
                    }
                });
            }

            ColorBuffer = newColorBuffer;
            DepthBuffer = newDepthBuffer;

            uint scaledWidth = (uint)(WindowWidth / RenderScale);
            uint scaledHeight = (uint)(WindowHeight / RenderScale);

            Gl.Viewport(0, (int)(WindowHeight - scaledHeight), scaledWidth, scaledHeight);

            unsafe
            {
                Gl.TexImage2D(TextureTarget.Texture2D, 0, InternalFormat.Rgb,
                    (uint)newSize.X, (uint)newSize.Y, 0,
                    PixelFormat.Rgb, PixelType.Float, null);
            }
        }

        private void OnResize(Vector2D<int> newSize)
        {
            HandleResize(newSize);
        }

        private void OnClosing()
        {
            Gl.DeleteTexture(TextureHandle);
            Gl.DeleteBuffer(VboHandle);
            Gl.DeleteVertexArray(VaoHandle);
            Gl.DeleteProgram(ShaderProgram);
        }

        private int GetIndex(int x, int y) => y * RenderWidth + x;

        // Pixel methods for ColorBuffer
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

        // Depth methods for DepthBuffer
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
