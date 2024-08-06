using System;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeAffineTransformList : NativeList<AffineTransform>
    {
        public NativeAffineTransformList() { }
        public NativeAffineTransformList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new AffineTransform();
        }

    }
}

