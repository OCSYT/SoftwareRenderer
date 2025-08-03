using Assimp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Linq;
using Matrix4x4 = System.Numerics.Matrix4x4;
using Quaternion = System.Numerics.Quaternion;

namespace SoftwareRenderer
{
    public struct VertexKey : IEquatable<VertexKey>
    {
        public readonly Vector3 Position;
        public readonly Vector3 Normal;
        public readonly Vector2 UV;

        public VertexKey(Vector3 position, Vector3 normal, Vector2 uv)
        {
            Position = position;
            Normal = normal;
            UV = uv;
        }

        public bool Equals(VertexKey other) =>
            Position.Equals(other.Position) && Normal.Equals(other.Normal) && UV.Equals(other.UV);

        public override bool Equals(object obj) => obj is VertexKey other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = Position.GetHashCode();
                hash = (hash * 397) ^ Normal.GetHashCode();
                hash = (hash * 397) ^ UV.GetHashCode();
                return hash;
            }
        }
    }

    public class Mesh
    {
        public BoundingSphere SphereBounds { get; set; }
        public Shaders.VertexInput[] Vertices { get; set; }
        public Shaders.VertexInput[] BaseVertices { get; private set; } // Original vertices
        public ushort[] Indices { get; set; }
        public Material Material { get; }
        public string ModelRootPath { get; set; }

        public Mesh(Shaders.VertexInput[] vertices, ushort[] indices, Material material = null)
        {
            Vertices = vertices;
            BaseVertices = (Shaders.VertexInput[])vertices.Clone();
            Indices = indices;
            Material = material;
        }
    }

    public class Model
    {
        private static readonly Dictionary<string, Model> _modelCache = new Dictionary<string, Model>();
        private static readonly Dictionary<string, Material> _materialCache = new Dictionary<string, Material>();

        public List<Mesh> Meshes { get; private set; } = new List<Mesh>();
        public List<Light> Lights { get; private set; } = new List<Light>();
        public List<Model> AnimationFrames { get; private set; } = new List<Model>();
        
        public Model LoadModel(string filePath)
        {
            // If filePath is not rooted, resolve relative to AppContext.BaseDirectory
            if (!Path.IsPathRooted(filePath))
            {
                filePath = Path.Combine(AppContext.BaseDirectory, filePath.TrimStart('.', '/', '\\'));
            }
            string normalizedPath = Path.GetFullPath(filePath);

            // Check if this is a directory containing animation frames
            if (Directory.Exists(normalizedPath))
            {
                if (_modelCache.TryGetValue(normalizedPath, out Model cachedModel))
                {
                    Meshes = cachedModel.Meshes;
                    Lights = cachedModel.Lights;
                    AnimationFrames = cachedModel.AnimationFrames;
                    return this;
                }

                AnimationFrames.Clear();

                var supportedExtensions = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                {
                    ".fbx", ".obj", ".dae", ".3ds", ".blend", ".gltf", ".glb"
                };

                var files = Directory.GetFiles(normalizedPath)
                    .Where(f => supportedExtensions.Contains(Path.GetExtension(f)))
                    .OrderBy(f => f)
                    .ToList();

                foreach (var file in files)
                {
                    var animFrame = new Model();
                    animFrame.LoadSingleModel(file);
                    AnimationFrames.Add(animFrame);
                }

                if (AnimationFrames.Count > 0)
                {
                    Meshes = AnimationFrames[0].Meshes;
                    Lights = AnimationFrames[0].Lights;
                }

                _modelCache[normalizedPath] = this;
            }
            else if (File.Exists(normalizedPath))
            {
                if (_modelCache.TryGetValue(normalizedPath, out Model cachedModel))
                {
                    Meshes = cachedModel.Meshes;
                    Lights = cachedModel.Lights;
                    AnimationFrames = cachedModel.AnimationFrames;
                    return this;
                }

                LoadSingleModel(normalizedPath);
                _modelCache[normalizedPath] = this;
            }
            else
            {
                throw new FileNotFoundException($"Model path not found: {normalizedPath}");
            }

            return this;
        }

        private void LoadSingleModel(string filePath)
        {
            Console.WriteLine("loading file: " + filePath);
            var meshes = new List<Mesh>();
            var lights = new List<Light>();
            var sceneMaterialCache = new Dictionary<int, Material>();

            var context = new AssimpContext();
            var scene = context.ImportFile(filePath,
                PostProcessSteps.Triangulate |
                PostProcessSteps.GenerateNormals |
                PostProcessSteps.FlipUVs |
                PostProcessSteps.CalculateTangentSpace |
                PostProcessSteps.JoinIdenticalVertices);

            Matrix4x4 ConvertMatrix(Assimp.Matrix4x4 m) =>
                new(
                    m.A1, m.B1, m.C1, m.D1,
                    m.A2, m.B2, m.C2, m.D2,
                    m.A3, m.B3, m.C3, m.D3,
                    m.A4, m.B4, m.C4, m.D4);

            void ProcessNode(Node node, Matrix4x4 parentTransform)
            {
                Matrix4x4 nodeTransform = ConvertMatrix(node.Transform);
                Matrix4x4 globalTransform = nodeTransform * parentTransform;

                Matrix4x4 rotationOnly = new Matrix4x4(
                    globalTransform.M11, globalTransform.M12, globalTransform.M13, 0,
                    globalTransform.M21, globalTransform.M22, globalTransform.M23, 0,
                    globalTransform.M31, globalTransform.M32, globalTransform.M33, 0,
                    0, 0, 0, 1);

                foreach (int meshIndex in node.MeshIndices)
                {
                    var mesh = scene.Meshes[meshIndex];
                    var vertexDict = new Dictionary<VertexKey, ushort>(mesh.VertexCount);
                    var vertices = new List<Shaders.VertexInput>(mesh.VertexCount);
                    var indices = new List<ushort>(mesh.FaceCount * 3);

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        var face = mesh.Faces[i];
                        if (face.IndexCount != 3) continue;

                        for (int j = 0; j < 3; j++)
                        {
                            int vi = face.Indices[j];
                            if (vi < 0 || vi >= mesh.VertexCount) continue;

                            var pos = mesh.Vertices[vi];
                            var normal = mesh.HasNormals ? mesh.Normals[vi] : new Vector3D(0, 0, 0);
                            var texCoord = mesh.HasTextureCoords(0)
                                ? mesh.TextureCoordinateChannels[0][vi]
                                : new Vector3D();
                            var color = (mesh.HasVertexColors(0) && vi < mesh.VertexColorChannels[0].Count)
                                ? mesh.VertexColorChannels[0][vi]
                                : new Color4D(1, 1, 1, 1);

                            var posVec = Vector3.Transform(new Vector3(pos.X, pos.Y, pos.Z), globalTransform);
                            var normalVec =
                                Vector3.Normalize(Vector3.TransformNormal(new Vector3(normal.X, normal.Y, normal.Z),
                                    rotationOnly));
                            var uv = new Vector2(texCoord.X, texCoord.Y);

                            var key = new VertexKey(posVec, normalVec, uv);

                            if (!vertexDict.TryGetValue(key, out ushort index))
                            {
                                index = (ushort)vertices.Count;
                                vertexDict[key] = index;

                                vertices.Add(new Shaders.VertexInput(
                                    posVec,
                                    uv,
                                    normalVec,
                                    new Vector4(color.R, color.G, color.B, color.A)
                                ));
                            }

                            indices.Add(index);
                        }
                    }

                    var assimpMaterial = scene.Materials[mesh.MaterialIndex];

                    // Create a material key that combines all material properties
                    string materialKey = $"{filePath}:{mesh.MaterialIndex}:{assimpMaterial.Name}";

                    if (!_materialCache.TryGetValue(materialKey, out Material cachedMaterial))
                    {
                        Vector4 baseColor = new Vector4(
                            assimpMaterial.ColorDiffuse.R,
                            assimpMaterial.ColorDiffuse.G,
                            assimpMaterial.ColorDiffuse.B,
                            assimpMaterial.ColorDiffuse.A);

                        float metallic = 0.0f;
                        if (assimpMaterial.HasProperty("$mat.metallicFactor"))
                            metallic = assimpMaterial.GetProperty("$mat.metallicFactor").GetFloatValue();

                        float roughness = 0.5f;
                        if (assimpMaterial.HasProperty("$mat.roughnessFactor"))
                            roughness = assimpMaterial.GetProperty("$mat.roughnessFactor").GetFloatValue();
                        else if (assimpMaterial.HasShininess)
                        {
                            float shininess = assimpMaterial.Shininess;
                            roughness = shininess > 0 ? 1.0f / shininess : 0.5f;
                        }

                        Vector3 emissive = new Vector3(
                            assimpMaterial.ColorEmissive.R,
                            assimpMaterial.ColorEmissive.G,
                            assimpMaterial.ColorEmissive.B);

                        var texturePaths = new Dictionary<TextureSlot, string>();
                        foreach (TextureType assimpType in Enum.GetValues(typeof(TextureType)))
                        {
                            if (assimpMaterial.GetMaterialTextureCount(assimpType) > 0)
                            {
                                if (assimpMaterial.GetMaterialTexture(assimpType, 0, out Assimp.TextureSlot slot))
                                {
                                    if (!string.IsNullOrWhiteSpace(slot.FilePath))
                                    {
                                        TextureSlot textureSlot = assimpType switch
                                        {
                                            TextureType.Diffuse => TextureSlot.Diffuse,
                                            TextureType.Specular => TextureSlot.Specular,
                                            TextureType.Normals => TextureSlot.Normals,
                                            TextureType.Height => TextureSlot.Height,
                                            TextureType.Emissive => TextureSlot.Emissive,
                                            _ => TextureSlot.Unknown
                                        };
                                        texturePaths[textureSlot] =
                                            Path.Combine(Path.GetDirectoryName(filePath) ?? string.Empty,
                                                slot.FilePath);
                                    }
                                }
                            }
                        }

                        cachedMaterial = new Material(baseColor, metallic, roughness, emissive, texturePaths);
                        _materialCache[materialKey] = cachedMaterial;
                        sceneMaterialCache[mesh.MaterialIndex] = cachedMaterial;
                    }
                    else if (!sceneMaterialCache.ContainsKey(mesh.MaterialIndex))
                    {
                        sceneMaterialCache[mesh.MaterialIndex] = cachedMaterial;
                    }

                    var meshResult = new Mesh(vertices.ToArray(), indices.ToArray(),
                        sceneMaterialCache[mesh.MaterialIndex])
                    {
                        ModelRootPath = Path.GetDirectoryName(filePath) ?? string.Empty,
                        SphereBounds = FrustumCuller.CalculateBoundingSphere(vertices.ToArray())
                    };

                    meshes.Add(meshResult);
                }

                foreach (var child in node.Children)
                {
                    ProcessNode(child, globalTransform);
                }
            }

            ProcessNode(scene.RootNode, Matrix4x4.Identity);

            for (int i = 0; i < scene.LightCount; i++)
            {
                var light = scene.Lights[i];
                Vector3 pos = new Vector3(light.Position.X, light.Position.Y, light.Position.Z);
                Vector3 dir = new Vector3(light.Direction.X, light.Direction.Y, light.Direction.Z);
                Vector3 color = new Vector3(light.ColorDiffuse.R, light.ColorDiffuse.G, light.ColorDiffuse.B);

                lights.Add(new Light(
                    pos,
                    dir,
                    color,
                    light.LightType,
                    light.AttenuationConstant,
                    light.AttenuationLinear,
                    light.AttenuationQuadratic,
                    light.AngleInnerCone,
                    light.AngleOuterCone));
            }

            Meshes = meshes;
            Lights = lights;
        }

        private int _currentFrameIndex = 0;
        private double _timeAccumulator = 0;

        public void PlayAnimation(Action<Model> OnFrameUpdate, double DeltaTime, int FPS = 30)
        {
            if (AnimationFrames.Count == 0) return;

            double frameDuration = 1.0 / FPS;
            _timeAccumulator += DeltaTime;

            while (_timeAccumulator >= frameDuration)
            {
                _timeAccumulator -= frameDuration;
                _currentFrameIndex = (_currentFrameIndex + 1) % AnimationFrames.Count;
            }


            var currentFrame = AnimationFrames[_currentFrameIndex];
            OnFrameUpdate?.Invoke(currentFrame);

        }
    }
}