using System.Numerics;
using Assimp;

namespace SoftwareRenderer
{

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
}