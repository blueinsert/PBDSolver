using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{

    public struct BurstRigidbody
    {
        public float4x4 inverseInertiaTensor;
        public float4 velocity;
        public float4 angularVelocity;
        public float4 com;
        public float inverseMass;

        private int pad0;
        private int pad1;
        private int pad2;
    }
}
