using System;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeBIHNodeList : NativeList<BIHNode>
    {
        public NativeBIHNodeList() { }
        public NativeBIHNodeList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new BIHNode();
        }
    }
}

