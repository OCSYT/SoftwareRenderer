using Assimp;
using System.Numerics;
using System.Collections.Generic;
using System.IO;
using System.Collections.Concurrent;
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

    public class Material
    {
        public Vector4 BaseColor { get; } // RGBA base color (albedo)
        public float Metallic { get; } // 0 (dielectric) to 1 (metal)
        public float Roughness { get; } // 0 (smooth) to 1 (rough)
        public Vector3 EmissiveColor { get; } // Emissive color (if no texture)
        public Dictionary<TextureSlot, string> TexturePaths { get; }

        public Material(Vector4 baseColor, float metallic, float roughness, Vector3 emissiveColor, Dictionary<TextureSlot, string> texturePaths)
        {
            BaseColor = baseColor;
            Metallic = metallic;
            Roughness = roughness;
            EmissiveColor = emissiveColor;
            TexturePaths = texturePaths;
        }
    }

    public class Mesh
    {
        public BoundingSphere SphereBounds { get; set; }
        public List<Shaders.VertexInput> Vertices { set; get; }
        public List<int> Indices { set; get; }
        public Material Material { get; }
        public string ModelRootPath;

        public Mesh(List<Shaders.VertexInput> vertices, List<int> indices, Material material = null)
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
            var meshes = new ConcurrentBag<Mesh>();
            var lights = new List<Light>();


            string finalPath = filePath;

            AssimpContext context = new();
            Scene scene = context.ImportFile(finalPath,
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

                // Create rotation-only matrix for normals
                Matrix4x4 rotationOnly = new Matrix4x4(
                    globalTransform.M11, globalTransform.M12, globalTransform.M13, 0,
                    globalTransform.M21, globalTransform.M22, globalTransform.M23, 0,
                    globalTransform.M31, globalTransform.M32, globalTransform.M33, 0,
                    0, 0, 0, 1);

                Parallel.ForEach(node.MeshIndices, meshIndex =>
                {
                    var mesh = scene.Meshes[meshIndex];
                    var vertexDict = new Dictionary<(Vector3 pos, Vector3 normal, Vector2 uv), int>();
                    var vertices = new List<Shaders.VertexInput>();
                    var indices = new List<int>();

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        var face = mesh.Faces[i];
                        if (face.IndexCount != 3) continue;

                        for (int j = 0; j < 3; j++)
                        {
                            int vi = face.Indices[j];
                            if (vi < 0 || vi >= mesh.VertexCount) continue; // Safety check

                            var pos = mesh.Vertices[vi];
                            var normal = mesh.Normals.Count > vi ? mesh.Normals[vi] : new Vector3D(0, 0, 0);
                            var texCoord = mesh.HasTextureCoords(0) ? mesh.TextureCoordinateChannels[0][vi] : new Vector3D();
                            var color = mesh.HasVertexColors(0) && vi < mesh.VertexColorChannels[0].Count
                                ? mesh.VertexColorChannels[0][vi]
                                : new Color4D(1, 1, 1, 1);

                            var posVec = new Vector3(pos.X, pos.Y, pos.Z);
                            var normalVec = new Vector3(normal.X, normal.Y, normal.Z);
                            var uv = new Vector2(texCoord.X, texCoord.Y);

                            var key = (posVec, normalVec, uv);

                            if (!vertexDict.TryGetValue(key, out int index))
                            {
                                index = vertices.Count;
                                vertexDict[key] = index;

                                vertices.Add(new Shaders.VertexInput(
                                    Vector3.Transform(posVec, globalTransform),
                                    uv,
                                    Vector3.Normalize(Vector3.TransformNormal(normalVec, rotationOnly)),
                                    new Vector4(color.R, color.G, color.B, color.A)
                                ));
                            }

                            indices.Add(index);
                        }
                    }

                    var assimpMaterial = scene.Materials[mesh.MaterialIndex];

                    // PBR material properties
                    Vector4 baseColor = new Vector4(
                        assimpMaterial.ColorDiffuse.R,
                        assimpMaterial.ColorDiffuse.G,
                        assimpMaterial.ColorDiffuse.B,
                        assimpMaterial.ColorDiffuse.A);

                    float metallic = 0.0f; // Default: non-metallic (dielectric)
                    if (assimpMaterial.HasProperty("$mat.metallicFactor"))
                    {
                        metallic = assimpMaterial.GetProperty("$mat.metallicFactor").GetFloatValue();
                    }

                    float roughness = 0.5f; // Default: moderate roughness
                    if (assimpMaterial.HasProperty("$mat.roughnessFactor"))
                    {
                        roughness = assimpMaterial.GetProperty("$mat.roughnessFactor").GetFloatValue();
                    }
                    else if (assimpMaterial.HasShininess)
                    {
                        // Convert shininess to roughness (approximation: higher shininess = lower roughness)
                        float shininess = assimpMaterial.Shininess;
                        roughness = shininess > 0 ? Math.Clamp(1.0f - (shininess / 100.0f), 0.0f, 1.0f) : 0.5f;
                    }

                    Vector3 emissiveColor = new Vector3(
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
                                    texturePaths[textureSlot] = Path.Combine(Path.GetDirectoryName(finalPath) ?? string.Empty, slot.FilePath);;
                                }
                            }
                        }
                    }

                    var material = new Material(baseColor, metallic, roughness, emissiveColor, texturePaths);
                    var meshToAdd = new Mesh(vertices, indices, material)
                    {
                        ModelRootPath = Path.GetDirectoryName(finalPath) ?? string.Empty,
                        SphereBounds = FrustumCuller.CalculateBoundingSphere(vertices.ToArray())
                    };
                    

                    meshes.Add(meshToAdd);
                });

                foreach (var child in node.Children)
                {
                    ProcessNode(child, globalTransform);
                }
            }

            ProcessNode(scene.RootNode, Matrix4x4.Identity);

            foreach (var assimpLight in scene.Lights)
            {
                var position = new Vector3(assimpLight.Position.X, assimpLight.Position.Y, assimpLight.Position.Z);
                var direction = new Vector3(assimpLight.Direction.X, assimpLight.Direction.Y, assimpLight.Direction.Z);
                var color = new Vector3(assimpLight.ColorDiffuse.R, assimpLight.ColorDiffuse.G, assimpLight.ColorDiffuse.B);

                lights.Add(new Light(
                    position,
                    direction,
                    color,
                    assimpLight.LightType,
                    assimpLight.AttenuationConstant,
                    assimpLight.AttenuationLinear,
                    assimpLight.AttenuationQuadratic,
                    assimpLight.AngleInnerCone,
                    assimpLight.AngleOuterCone
                ));
            }

            context.Dispose();
            return (meshes.ToList(), lights);
        }
    }
}