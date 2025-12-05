using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeInt3List : NativeList<VectorInt3>
    {
        public NativeInt3List() { }
        public NativeInt3List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new VectorInt3(0, 0, 0);
        }

        public NativeInt3List(int capacity, int alignment, VectorInt3 defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }
    }
}

