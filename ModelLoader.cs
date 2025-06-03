using Assimp;
using System.Numerics;
using System.Collections.Generic;
using System.IO;
using Matrix4x4 = System.Numerics.Matrix4x4;

namespace SoftwareRenderer
{

    public class Material
    {
        public Vector4 DiffuseColor { get; }
        public Vector4 SpecularColor { get; }
        public float Shininess { get; }
        public string DiffuseTexturePath { get; }

        public Material(Vector4 diffuse, Vector4 specular, float shininess, string diffuseTexPath)
        {
            DiffuseColor = diffuse;
            SpecularColor = specular;
            Shininess = shininess;
            DiffuseTexturePath = diffuseTexPath;
        }
    }

    public class Mesh
    {
        public BoundingSphere SphereBounds { get; set; }
        public List<Shaders.VertexInput> Vertices { set; get; }
        public List<int> Indices { set; get; }
        public Material Material { get; }
        public string ModelRootPath;

        public int TextureIndex { get; set; } = -1;

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
        public static List<Texture> LoadedTextures = new List<Texture>();

        public static (List<Mesh> Meshes, List<Light> Lights) LoadModel(string path)
        {
            var context = new AssimpContext();
            var scene = context.ImportFile(path,
                PostProcessSteps.Triangulate |
                PostProcessSteps.GenerateNormals |
                PostProcessSteps.FlipUVs);

            if (scene.MeshCount == 0)
                throw new System.Exception("No meshes found in model file");

            var meshes = new List<Mesh>();
            var lights = new List<Light>();

            Matrix4x4 ConvertMatrix(Assimp.Matrix4x4 m)
            {
                return new Matrix4x4(
                    m.A1, m.B1, m.C1, m.D1,
                    m.A2, m.B2, m.C2, m.D2,
                    m.A3, m.B3, m.C3, m.D3,
                    m.A4, m.B4, m.C4, m.D4
                );
            }

            void ProcessNode(Node node, Matrix4x4 parentTransform)
            {
                var nodeTransform = ConvertMatrix(node.Transform);
                var globalTransform = nodeTransform * parentTransform;

                Parallel.ForEach(node.MeshIndices, meshIndex =>
                {
                    var mesh = scene.Meshes[meshIndex];

                    var vertexDict = new Dictionary<(Vector3 pos, Vector3 normal, Vector2 uv), int>();
                    var vertices = new List<Shaders.VertexInput>();
                    var indices = new List<int>();
                    for (int i = 0; i < mesh.VertexCount; i++)
                    {
                        var pos = mesh.Vertices[i];
                        var normal = mesh.Normals.Count > i ? mesh.Normals[i] : new Vector3D(0, 0, 0);
                        var texCoord = mesh.HasTextureCoords(0)
                            ? mesh.TextureCoordinateChannels[0][i]
                            : new Vector3D(0, 0, 0);

                        var posVec = new Vector3(pos.X, pos.Y, pos.Z);
                        var normalVec = new Vector3(normal.X, normal.Y, normal.Z);

                        var transformedPos = Vector3.Transform(posVec, globalTransform);

                        var rotationOnly = new Matrix4x4(
                            globalTransform.M11, globalTransform.M12, globalTransform.M13, 0,
                            globalTransform.M21, globalTransform.M22, globalTransform.M23, 0,
                            globalTransform.M31, globalTransform.M32, globalTransform.M33, 0,
                            0, 0, 0, 1);

                        var transformedNormal = Vector3.TransformNormal(normalVec, rotationOnly);

                        var color = mesh.HasVertexColors(0)
                            ? mesh.VertexColorChannels[0][i]
                            : new Color4D(1, 1, 1, 1); 
                        
                        vertices.Add(new Shaders.VertexInput(
                            transformedPos,
                            new Vector2(texCoord.X, texCoord.Y),
                            Vector3.Normalize(transformedNormal),
                            new Vector4(color.R, color.G, color.B, color.A)
                        ));
                    }
                    

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        var face = mesh.Faces[i];
                        if (face.IndexCount != 3) continue;

                        for (int j = 0; j < 3; j++)
                        {
                            int vi = face.Indices[j];
                            var pos = mesh.Vertices[vi];
                            var normal = mesh.Normals[vi];
                            var texCoord = mesh.HasTextureCoords(0) ? mesh.TextureCoordinateChannels[0][vi] : new Vector3D();

                            var color = mesh.HasVertexColors(0)
                                ? mesh.VertexColorChannels[0][i]
                                : new Color4D(1, 1, 1, 1); 
                            
                            var key = (
                                new Vector3(pos.X, pos.Y, pos.Z),
                                new Vector3(normal.X, normal.Y, normal.Z),
                                new Vector2(texCoord.X, texCoord.Y)
                            );

                            if (!vertexDict.TryGetValue(key, out int index))
                            {
                                index = vertices.Count;
                                vertexDict[key] = index;
                                vertices.Add(new Shaders.VertexInput(
                                    Vector3.Transform(key.Item1, globalTransform),
                                    key.Item3,
                                    Vector3.Normalize(Vector3.TransformNormal(key.Item2, globalTransform)),
                                    new Vector4(color.R, color.G, color.B, color.A)
                                ));
                            }

                            indices.Add(index);
                        }
                    }

                    var assimpMaterial = scene.Materials[mesh.MaterialIndex];

                    var diffuse = assimpMaterial.ColorDiffuse;
                    var diffuseColor = new Vector4(diffuse.R, diffuse.G, diffuse.B, diffuse.A);

                    var specular = assimpMaterial.ColorSpecular;
                    var specularColor = new Vector4(specular.R, specular.G, specular.B, specular.A);

                    float shininess = assimpMaterial.Shininess;

                    string diffuseTexturePath = string.Empty;
                    if (assimpMaterial.HasTextureDiffuse)
                    {
                        var texSlot = assimpMaterial.TextureDiffuse;
                        diffuseTexturePath = texSlot.FilePath;
                    }

                    var material = new Material(diffuseColor, specularColor, shininess, diffuseTexturePath);

                    Mesh meshToAdd = new Mesh(vertices, indices, material)
                    {
                        ModelRootPath = Path.GetDirectoryName(path) ?? string.Empty
                    };

                    
                    meshToAdd.SphereBounds = FrustumCuller.CalculateBoundingSphere(vertices.ToArray());

                    // Load texture
                    if (!string.IsNullOrWhiteSpace(material.DiffuseTexturePath))
                    {
                        string textureFullPath = Path.Combine(meshToAdd.ModelRootPath, material.DiffuseTexturePath);

                        var textureToAdd = Texture.LoadTexture(textureFullPath);

                        if (textureToAdd?.Data != null)
                        {
                            LoadedTextures.Add(textureToAdd);
                            meshToAdd.TextureIndex = LoadedTextures.Count - 1;
                        }
                        else
                        {
                            meshToAdd.TextureIndex = -1;
                        }
                    }
                    else
                    {
                        meshToAdd.TextureIndex = -1;
                    }

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

            return (meshes, lights);
        }
    }
}