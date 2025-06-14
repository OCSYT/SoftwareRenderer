using System.Numerics;

namespace SoftwareRenderer
{

    public class Material
    {
        public Vector4 BaseColor { get; }
        public float Metallic { get; }
        public float Roughness { get; }
        public Vector3 EmissiveColor { get; }
        public IReadOnlyDictionary<TextureSlot, string> TexturePaths { get; }

        public Material(Vector4 baseColor, float metallic, float roughness, Vector3 emissiveColor,
            Dictionary<TextureSlot, string> texturePaths)
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
}