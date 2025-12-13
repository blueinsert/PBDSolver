using System;
using Unity.Collections;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeQuaternionList : NativeList<Quaternion>
    {
        public NativeQuaternionList() { }
        public NativeQuaternionList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Quaternion.identity;
        }

        public NativeQuaternionList(int capacity, int alignment, Quaternion defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }

    }
}