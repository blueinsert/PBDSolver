using System;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeAabbList : NativeList<Aabb>
    {
        public NativeAabbList() { }
        public NativeAabbList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new Aabb();
        }

    }
}

