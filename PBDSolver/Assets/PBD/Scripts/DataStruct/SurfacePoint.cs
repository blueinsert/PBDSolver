using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct SurfacePoint
    {
        public float4 bary;
        public float4 point;
        public float4 normal;
    }
}
