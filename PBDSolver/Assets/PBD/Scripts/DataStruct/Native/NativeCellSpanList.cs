using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeCellSpanList : NativeList<CellSpan>
    {
        public NativeCellSpanList() { }
        public NativeCellSpanList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new CellSpan();
        }

    }
}

