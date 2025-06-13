using Assimp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Linq;
using Matrix4x4 = System.Numerics.Matrix4x4;

namespace SoftwareRenderer
{
    public enum TextureSlot
    {
        Diffuse = TextureType.Diffuse,
        Specular = TextureType.Specular,
        Ambient = TextureType.Ambient,
        Emissive = TextureType.Emissive,
        Height = TextureType.Height,
        Normals = TextureType.Normals,
        Shininess = TextureType.Shininess,
        Opacity = TextureType.Opacity,
        Displacement = TextureType.Displacement,
        Lightmap = TextureType.Lightmap,
        Reflection = TextureType.Reflection,
        BaseColor = TextureType.BaseColor,
        NormalCamera = TextureType.NormalCamera,
        EmissionColor = TextureType.EmissionColor,
        Metalness = TextureType.Metalness,
        DiffuseRoughness = TextureType.Roughness,
        AmbientOcclusion = TextureType.AmbientOcclusion,
        Unknown = TextureType.Unknown
    }

    public class Material : IEquatable<Material>
    {
        public Vector4 BaseColor { get; }
        public float Metallic { get; }
        public float Roughness { get; }
        public Vector3 EmissiveColor { get; }
        public IReadOnlyDictionary<TextureSlot, string> TexturePaths { get; }

        public Material(Vector4 baseColor, float metallic, float roughness, Vector3 emissiveColor, Dictionary<TextureSlot, string> texturePaths)
        {
            BaseColor = baseColor;
            Metallic = metallic;
            Roughness = roughness;
            EmissiveColor = emissiveColor;
            TexturePaths = texturePaths;
        }

        // Equality members to allow caching and reuse of Materials
        public bool Equals(Material other)
        {
            if (other == null) return false;
            if (!BaseColor.Equals(other.BaseColor)) return false;
            if (Metallic != other.Metallic || Roughness != other.Roughness) return false;
            if (!EmissiveColor.Equals(other.EmissiveColor)) return false;
            if (TexturePaths.Count != other.TexturePaths.Count) return false;
            foreach (var kvp in TexturePaths)
            {
                if (!other.TexturePaths.TryGetValue(kvp.Key, out var path)) return false;
                if (!string.Equals(kvp.Value, path, StringComparison.OrdinalIgnoreCase)) return false;
            }
            return true;
        }
        public override bool Equals(object obj) => Equals(obj as Material);
        public override int GetHashCode()
        {
            int hash = BaseColor.GetHashCode();
            hash = (hash * 397) ^ Metallic.GetHashCode();
            hash = (hash * 397) ^ Roughness.GetHashCode();
            hash = (hash * 397) ^ EmissiveColor.GetHashCode();
            foreach (var kvp in TexturePaths)
            {
                hash = (hash * 397) ^ kvp.Key.GetHashCode();
                hash = (hash * 397) ^ (kvp.Value?.GetHashCode() ?? 0);
            }
            return hash;
        }
    }

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
        public Shaders.VertexInput[] Vertices { get; }
        public ushort[] Indices { get; }
        public Material Material { get; }
        public string ModelRootPath;

        public Mesh(Shaders.VertexInput[] vertices, ushort[] indices, Material material = null)
        {
            Vertices = vertices;
            Indices = indices;
            Material = material;
        }
    }

    public class Light
    {
        public Vector3 Position { get; }
        public Vector3 Direction { get; }
        public Vector3 Color { get; }
        public LightSourceType Type { get; }
        public float AttenuationConstant { get; }
        public float AttenuationLinear { get; }
        public float AttenuationQuadratic { get; }
        public float SpotCutoffInner { get; }
        public float SpotCutoffOuter { get; }

        public Light(Vector3 position, Vector3 direction, Vector3 color, LightSourceType type,
            float attenuationConstant, float attenuationLinear, float attenuationQuadratic,
            float spotCutoffInner, float spotCutoffOuter)
        {
            Position = position;
            Direction = direction;
            Color = color;
            Type = type;
            AttenuationConstant = attenuationConstant;
            AttenuationLinear = attenuationLinear;
            AttenuationQuadratic = attenuationQuadratic;
            SpotCutoffInner = spotCutoffInner;
            SpotCutoffOuter = spotCutoffOuter;
        }
    }

    public static class Model
    {
        public static (List<Mesh> Meshes, List<Light> Lights) LoadModel(string filePath)
        {
            var meshes = new List<Mesh>();
            var lights = new List<Light>();
            var materialCache = new Dictionary<Material, Material>();

            var context = new AssimpContext();
            var scene = context.ImportFile(filePath,
                PostProcessSteps.Triangulate |
                PostProcessSteps.GenerateNormals |
                PostProcessSteps.FlipUVs);

            if (scene.MeshCount == 0)
                throw new Exception("No meshes found in model file");

            Matrix4x4 ConvertMatrix(Assimp.Matrix4x4 m) =>
                new(m.A1, m.B1, m.C1, m.D1,
                    m.A2, m.B2, m.C2, m.D2,
                    m.A3, m.B3, m.C3, m.D3,
                    m.A4, m.B4, m.C4, m.D4);

            void ProcessNode(Node node, Matrix4x4 parentTransform)
            {
                Matrix4x4 nodeTransform = ConvertMatrix(node.Transform);
                Matrix4x4 globalTransform = nodeTransform * parentTransform;

                // Rotation only for normals
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
                            var normal = mesh.Normals.Count > vi ? mesh.Normals[vi] : new Vector3D(0, 0, 0);
                            var texCoord = mesh.HasTextureCoords(0) ? mesh.TextureCoordinateChannels[0][vi] : new Vector3D();
                            var color = mesh.HasVertexColors(0) && vi < mesh.VertexColorChannels[0].Count
                                ? mesh.VertexColorChannels[0][vi]
                                : new Color4D(1, 1, 1, 1);

                            var posVec = Vector3.Transform(new Vector3(pos.X, pos.Y, pos.Z), globalTransform);
                            var normalVec = Vector3.Normalize(Vector3.TransformNormal(new Vector3(normal.X, normal.Y, normal.Z), rotationOnly));
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
                                    TextureSlot textureSlot = Enum.IsDefined(typeof(TextureSlot), (int)assimpType)
                                        ? (TextureSlot)(int)assimpType
                                        : TextureSlot.Unknown;
                                    texturePaths[textureSlot] = Path.Combine(Path.GetDirectoryName(filePath) ?? string.Empty, slot.FilePath);;
                                }
                            }
                        }
                    }

                    var material = new Material(baseColor, metallic, roughness, emissive, texturePaths);

                    // Reuse cached materials
                    if (!materialCache.TryGetValue(material, out Material cachedMaterial))
                    {
                        cachedMaterial = material;
                        materialCache[cachedMaterial] = cachedMaterial;
                    }

                    var meshResult = new Mesh(vertices.ToArray(), indices.ToArray(), cachedMaterial)
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

            // Load lights from scene (if any)
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

            return (meshes, lights);
        }
    }
}
