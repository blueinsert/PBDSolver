using System;
using Unity.Collections;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native

{
    [Serializable]
    public class NativeRigidbodyList : NativeList<ColliderRigidbody>
    {
        public NativeRigidbodyList() { }
        public NativeRigidbodyList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new ColliderRigidbody();
        }

    }
}

