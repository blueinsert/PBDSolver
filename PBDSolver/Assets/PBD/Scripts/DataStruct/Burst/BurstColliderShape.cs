namespace bluebean.Physics.PBD.DataStruct
{

    public struct BurstColliderShape
    {
        //public float4 center;
        //public float4 size; 
        public ColliderShapeType type;
        public float contactOffset;
        public int dataIndex;
        //public int rigidbodyIndex;  // index of the associated rigidbody in the collision world.
        //public int materialIndex;   // index of the associated material in the collision world.
        //public int filter;
        //public int flags;           // for now, only used for trigger (1) or regular collider (0).
        //public int is2D;            // whether the collider is 2D (1) or 3D (0).
    }
}
