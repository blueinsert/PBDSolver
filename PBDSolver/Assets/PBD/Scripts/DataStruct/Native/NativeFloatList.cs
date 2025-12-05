using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeFloatList : NativeList<float>
    {
        public NativeFloatList() { }
        public NativeFloatList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = 0;
        }

    }
}
