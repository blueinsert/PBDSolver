using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeInt2List : NativeList<VectorInt2>
    {
        public NativeInt2List() { }
        public NativeInt2List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new VectorInt2(0, 0);
        }

        public NativeInt2List(int capacity, int alignment, VectorInt2 defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }
    }
}

