using System;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeVector3List : NativeList<Vector3>
    {
        public NativeVector3List() { }
        public NativeVector3List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Vector3.zero;
        }

    }
}

