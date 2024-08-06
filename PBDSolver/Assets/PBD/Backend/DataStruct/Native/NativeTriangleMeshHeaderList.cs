using System;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeTriangleMeshHeaderList : NativeList<TriangleMeshHeader>
    {
        public NativeTriangleMeshHeaderList() { }
        public NativeTriangleMeshHeaderList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new TriangleMeshHeader();
        }

    }
}

