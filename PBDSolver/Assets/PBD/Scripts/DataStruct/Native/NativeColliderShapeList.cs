using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeColliderShapeList : NativeList<ColliderShape>
    {
        public NativeColliderShapeList() { }
        public NativeColliderShapeList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new ColliderShape();
        }

    }
}

